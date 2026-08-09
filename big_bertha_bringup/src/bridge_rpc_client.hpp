// Copyright 2026 Jjateen Gundesha
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BRIDGE_RPC_CLIENT_HPP_
#define BRIDGE_RPC_CLIENT_HPP_

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// Minimal MessagePack RPC client for the Arduino Router Bridge protocol.
//
// Wire format (MsgPack-RPC):
//   Request:       [0, msgid, "method", [params...]]
//   Response:      [1, msgid, error_or_nil, result_or_nil]
//   Notification:  [2, "method", [params...]]
//
// Registration: call $/register on the router to start receiving messages.
//
// The sketch on the STM32U585 registers providers and pushes notifications:
//   provide_str("imu", ...) -> comma CSV "qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz,sample,ts"
//   provide_str("hw_status", ...) -> [scan, ai, servo_calls, ...] (11 numeric)
//   provide("i2c_scan", ...)     -> [addr, ...]
//   provide_str("imu_diag", ...) -> single string
//   provide_str("servo_diag_result", ...) -> single string
//   provide("pong", ...)         -> [count]
//
// Outbound:
//   notify_str("set_servo_pwms", "307,153,...")  (comma string, single arg)
//   notify("scan_i2c") / notify("servo_diag") / notify("ping")   (no args)

class BridgeRPCClient
{
public:
  using FloatHandler = std::function<void(const std::vector<double> &)>;
  using StringHandler = std::function<void(const std::string &)>;

  // A message parsed from the router: numeric params and/or string params.
  struct Message
  {
    std::string method;
    std::vector<double> numbers;
    std::vector<std::string> strings;
  };

  explicit BridgeRPCClient(std::vector<std::string> socket_candidates)
  : socket_candidates_(std::move(socket_candidates))
  {
    if (socket_candidates_.empty()) {
      socket_candidates_.push_back("/run/arduino-router/rpc.sock");
    }
  }

  ~BridgeRPCClient() { stop(); }

  BridgeRPCClient(const BridgeRPCClient &) = delete;
  BridgeRPCClient & operator=(const BridgeRPCClient &) = delete;

  // Connect to the first reachable candidate, register handlers, and start
  // the reader thread. Registration happens BEFORE the reader thread starts:
  // the reader consumes bytes from the same socket, so it would otherwise
  // eat the $/register responses and each register_method() would time out.
  bool start()
  {
    if (running_) return true;
    if (!connect_socket()) return false;

    std::vector<std::string> methods;
    {
      std::lock_guard<std::mutex> lk(handlers_mutex_);
      for (const auto & kv : float_handlers_) methods.push_back(kv.first);
      for (const auto & kv : string_handlers_) methods.push_back(kv.first);
    }
    for (const auto & method : methods) {
      register_method(method);
    }

    running_ = true;
    reader_thread_ = std::thread(&BridgeRPCClient::reader_loop, this);
    return true;
  }

  void stop()
  {
    running_ = false;
    if (reader_thread_.joinable()) reader_thread_.join();
    std::lock_guard<std::mutex> lk(sock_mutex_);
    if (sock_fd_ >= 0) {
      ::shutdown(sock_fd_, SHUT_RDWR);
      ::close(sock_fd_);
      sock_fd_ = -1;
    }
  }

  // Fire-and-forget notification, no params.
  void notify(const std::string & method) { send_bytes(pack_notification(method)); }

  // Fire-and-forget notification with double params.
  void notify(const std::string & method, const std::vector<double> & params)
  {
    send_bytes(pack_notification(method, params));
  }

  // Fire-and-forget notification with integer params.
  void notify_int(const std::string & method, const std::vector<int64_t> & params)
  {
    send_bytes(pack_notification_int(method, params));
  }

  // Fire-and-forget notification with a single string param (e.g. the
  // comma-separated PWM payload for set_servo_pwms).
  void notify_str(const std::string & method, const std::string & text)
  {
    send_bytes(pack_notification_str(method, text));
  }

  // Register a handler for inbound numeric notifications on `method`.
  // Handlers must be registered before start() so all are $/registered with
  // the router before the reader thread begins consuming socket bytes.
  void provide(const std::string & method, FloatHandler handler)
  {
    std::lock_guard<std::mutex> lk(handlers_mutex_);
    float_handlers_[method] = std::move(handler);
  }

  // Register a handler for inbound string notifications on `method`.
  // Must be registered before start().
  void provide_str(const std::string & method, StringHandler handler)
  {
    std::lock_guard<std::mutex> lk(handlers_mutex_);
    string_handlers_[method] = std::move(handler);
  }

  // The socket path that is currently connected (for diagnostics).
  std::string connected_path()
  {
    std::lock_guard<std::mutex> lk(sock_mutex_);
    return connected_path_;
  }

  bool is_connected()
  {
    std::lock_guard<std::mutex> lk(sock_mutex_);
    return sock_fd_ >= 0;
  }

private:
  // ── Socket ──────────────────────────────────────────────────────────

  bool connect_socket()
  {
    std::lock_guard<std::mutex> lk(sock_mutex_);
    return connect_socket_locked();
  }

  // Caller MUST hold sock_mutex_.
  bool connect_socket_locked()
  {
    if (sock_fd_ >= 0) return true;

    for (const auto & path : socket_candidates_) {
      int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
      if (fd < 0) continue;

      struct sockaddr_un addr;
      std::memset(&addr, 0, sizeof(addr));
      addr.sun_family = AF_UNIX;
      std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);

      if (::connect(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) == 0) {
        sock_fd_ = fd;
        connected_path_ = path;
        return true;
      }
      ::close(fd);
    }
    return false;
  }

  void send_bytes(const std::vector<uint8_t> & buf)
  {
    std::lock_guard<std::mutex> lk(sock_mutex_);
    if (sock_fd_ < 0) return;
    ::send(sock_fd_, buf.data(), buf.size(), MSG_NOSIGNAL);
  }

  // Send a request and wait for a response (blocking, for $/register).
  bool register_method(const std::string & method)
  {
    auto buf = pack_register_call(method);
    std::vector<uint8_t> resp;
    resp.reserve(64);

    {
      std::lock_guard<std::mutex> lk(sock_mutex_);
      if (sock_fd_ < 0) return false;
      ::send(sock_fd_, buf.data(), buf.size(), MSG_NOSIGNAL);

      // Read response: [1, msgid, nil, true]
      // Encodes as: [0x94, 0x01, 0x01, 0xc0, 0xc3] = 5 bytes
      // Previous check at >= 4 left 0xc3 in socket, breaking next registration.
      auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
      while (std::chrono::steady_clock::now() < deadline) {
        uint8_t byte;
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sock_fd_, &rfds);
        struct timeval tv = {0, 50000};  // 50 ms
        if (select(sock_fd_ + 1, &rfds, nullptr, nullptr, &tv) <= 0) continue;
        ssize_t n = ::read(sock_fd_, &byte, 1);
        if (n <= 0) break;
        resp.push_back(byte);

        // Check for complete response: 5 bytes minimum
        // [0x94] [type] [msgid] [error] [result]
        if (resp.size() >= 5 && resp[0] == 0x94) {
          // Validate structure:
          // resp[1] should be 0x01 (type=1 for response)
          // resp[2] should be 0x01 (msgid=1, fixint)
          // resp[3] should be 0xc0 (nil error)
          // resp[4] should be 0xc3 (bool true result)
          if (resp[1] == 0x01 && resp[3] == 0xc0 && resp[4] == 0xc3) {
            return true;  // Success
          }
        }
      }
    }
    return !resp.empty();
  }

  // ── MsgPack packers ─────────────────────────────────────────────────

  static void pack_byte(std::vector<uint8_t> & buf, uint8_t b) { buf.push_back(b); }

  static void pack_array(std::vector<uint8_t> & buf, size_t n)
  {
    if (n <= 15) {
      pack_byte(buf, 0x90 | n);
      return;
    }
    if (n <= 0xFFFF) {
      pack_byte(buf, 0xdc);
      pack_byte(buf, (n >> 8) & 0xFF);
      pack_byte(buf, n & 0xFF);
      return;
    }
    pack_byte(buf, 0xdd);
    pack_byte(buf, (n >> 24) & 0xFF);
    pack_byte(buf, (n >> 16) & 0xFF);
    pack_byte(buf, (n >> 8) & 0xFF);
    pack_byte(buf, n & 0xFF);
  }

  static void pack_str(std::vector<uint8_t> & buf, const std::string & s)
  {
    size_t len = s.size();
    if (len <= 31) {
      pack_byte(buf, 0xa0 | len);
    } else if (len <= 0xFFFF) {
      pack_byte(buf, 0xda);
      pack_byte(buf, (len >> 8) & 0xFF);
      pack_byte(buf, len & 0xFF);
    } else {
      pack_byte(buf, 0xdb);
      pack_byte(buf, (len >> 24) & 0xFF);
      pack_byte(buf, (len >> 16) & 0xFF);
      pack_byte(buf, (len >> 8) & 0xFF);
      pack_byte(buf, len & 0xFF);
    }
    buf.insert(buf.end(), s.begin(), s.end());
  }

  static void pack_int(std::vector<uint8_t> & buf, int64_t val)
  {
    if (val >= 0 && val <= 0x7F) {
      pack_byte(buf, val);
      return;
    }
    if (val >= -32 && val <= -1) {
      pack_byte(buf, 0xe0 | (val & 0x1F));
      return;
    }
    if (val >= -128 && val <= 127) {
      pack_byte(buf, 0xd0);
      pack_byte(buf, val & 0xFF);
      return;
    }
    if (val >= -32768 && val <= 32767) {
      pack_byte(buf, 0xd1);
      pack_byte(buf, (val >> 8) & 0xFF);
      pack_byte(buf, val & 0xFF);
      return;
    }
    pack_byte(buf, 0xd2);
    pack_byte(buf, (val >> 24) & 0xFF);
    pack_byte(buf, (val >> 16) & 0xFF);
    pack_byte(buf, (val >> 8) & 0xFF);
    pack_byte(buf, val & 0xFF);
  }

  static void pack_float64(std::vector<uint8_t> & buf, double val)
  {
    pack_byte(buf, 0xcb);
    uint64_t bits;
    std::memcpy(&bits, &val, sizeof(bits));
    bits = htobe64(bits);
    buf.insert(
      buf.end(), reinterpret_cast<uint8_t *>(&bits), reinterpret_cast<uint8_t *>(&bits) + 8);
  }

  static void pack_nil(std::vector<uint8_t> & buf) { pack_byte(buf, 0xc0); }

  // Notification: [2, "method", []]
  static std::vector<uint8_t> pack_notification(const std::string & method)
  {
    std::vector<uint8_t> buf;
    pack_array(buf, 3);
    pack_int(buf, 2);
    pack_str(buf, method);
    pack_array(buf, 0);
    return buf;
  }

  // Notification: [2, "method", [double_params...]]
  static std::vector<uint8_t> pack_notification(
    const std::string & method, const std::vector<double> & params)
  {
    std::vector<uint8_t> buf;
    pack_array(buf, 3);
    pack_int(buf, 2);
    pack_str(buf, method);
    pack_array(buf, params.size());
    for (double p : params) pack_float64(buf, p);
    return buf;
  }

  // Notification with ints: [2, "method", [int_params...]]
  static std::vector<uint8_t> pack_notification_int(
    const std::string & method, const std::vector<int64_t> & params)
  {
    std::vector<uint8_t> buf;
    pack_array(buf, 3);
    pack_int(buf, 2);
    pack_str(buf, method);
    pack_array(buf, params.size());
    for (int64_t p : params) pack_int(buf, p);
    return buf;
  }

  // Notification with a single string param: [2, "method", ["text"]]
  static std::vector<uint8_t> pack_notification_str(
    const std::string & method, const std::string & text)
  {
    std::vector<uint8_t> buf;
    pack_array(buf, 3);
    pack_int(buf, 2);
    pack_str(buf, method);
    pack_array(buf, 1);
    pack_str(buf, text);
    return buf;
  }

  // Register call: [0, msgid, "$/register", ["method"]]
  static std::vector<uint8_t> pack_register_call(const std::string & method)
  {
    std::vector<uint8_t> buf;
    pack_array(buf, 4);
    pack_int(buf, 0);
    pack_int(buf, 1);  // msgid
    pack_str(buf, "$/register");
    pack_array(buf, 1);
    pack_str(buf, method);
    return buf;
  }

  // ── MsgPack unpacker (stream-based) ─────────────────────────────────

  // Returns the number of bytes consumed, or 0 if incomplete.
  // On success, `out` is filled for notifications.
  size_t try_parse_message(const uint8_t * data, size_t len, Message & out)
  {
    if (len == 0) return 0;
    size_t off = 0;

    // Must start with a fixarray (we skip array16/32 for simplicity).
    // Notifications are [2, "method", [params]] — 3 elements (0x93).
    // Responses are [1, msgid, error, result] — 4 elements (0x94).
    if ((data[off] & 0xf0) != 0x90) return 1;  // skip garbage byte
    size_t arr_len = data[off] & 0x0f;
    off++;

    // If this is a response (4 elements), parse its length and consume only
    // those bytes, not the entire buffer. We handle responses synchronously
    // in register_method(), but batched notifications following a response
    // must be preserved for dispatch.
    if (arr_len == 4) {
      size_t resp_off = off;

      // Element 0: message type (should be 1, fixint = 1 byte)
      if (resp_off >= len) return 0;
      resp_off++;

      // Element 1: msgid (parse variable-length int)
      int64_t dummy_msgid;
      size_t msgid_used = scan_int(data + resp_off, len - resp_off, dummy_msgid);
      if (msgid_used == 0) return 0;
      resp_off += msgid_used;

      // Element 2: error (should be nil = 0xc0, 1 byte)
      if (resp_off >= len) return 0;
      resp_off++;

      // Element 3: result (should be bool = 0xc3 or 0xc2, 1 byte)
      if (resp_off >= len) return 0;
      resp_off++;

      // Return only the bytes consumed by this response frame
      return resp_off;
    }

    // Must be a notification (3 elements)
    if (arr_len != 3) return off;

    // Element 0: message type (must be 2)
    {
      int64_t t;
      size_t used = scan_int(data + off, len - off, t);
      if (used == 0) return 0;
      off += used;
      if (t != 2) return off;
    }

    // Element 1: method name (string)
    {
      size_t used = scan_str(data + off, len - off, out.method);
      if (used == 0) return 0;
      off += used;
    }

    // Element 2: params array
    {
      if (off >= len) return 0;
      if ((data[off] & 0xf0) != 0x90) return off + 1;
      size_t param_count = data[off] & 0x0f;
      off++;

      out.numbers.clear();
      out.strings.clear();
      for (size_t i = 0; i < param_count; ++i) {
        // Try a string param first (fixstr/str8/str16).
        {
          std::string s;
          size_t used = scan_str_full(data + off, len - off, s);
          if (used > 0) {
            out.strings.push_back(s);
            off += used;
            continue;
          }
        }
        double val;
        size_t used = scan_number(data + off, len - off, val);
        if (used == 0) return 0;
        out.numbers.push_back(val);
        off += used;
      }
    }

    return off;
  }

  // Scan a MsgPack integer, return bytes consumed (0 = need more data).
  static size_t scan_int(const uint8_t * data, size_t len, int64_t & out)
  {
    if (len == 0) return 0;
    uint8_t b = data[0];
    if (b <= 0x7f) {
      out = b;
      return 1;
    }
    if (b >= 0xe0) {
      out = static_cast<int8_t>(b);
      return 1;
    }
    if (b == 0xd0 && len >= 2) {
      out = static_cast<int8_t>(data[1]);
      return 2;
    }
    if (b == 0xd1 && len >= 3) {
      out = static_cast<int16_t>((data[1] << 8) | data[2]);
      return 3;
    }
    if (b == 0xd2 && len >= 5) {
      out = static_cast<int32_t>(
        (static_cast<uint32_t>(data[1]) << 24) | (static_cast<uint32_t>(data[2]) << 16) |
        (static_cast<uint32_t>(data[3]) << 8) | data[4]);
      return 5;
    }
    return 0;  // incomplete or unsupported
  }

  // Scan a MsgPack fixstr, return bytes consumed.
  static size_t scan_str(const uint8_t * data, size_t len, std::string & out)
  {
    if (len == 0) return 0;
    uint8_t b = data[0];
    if ((b & 0xe0) != 0xa0) return 0;  // not a fixstr
    size_t str_len = b & 0x1f;
    if (len < 1 + str_len) return 0;
    out.assign(reinterpret_cast<const char *>(data + 1), str_len);
    return 1 + str_len;
  }

  // Scan any MsgPack string (fixstr, str8, str16, str32), return bytes consumed.
  static size_t scan_str_full(const uint8_t * data, size_t len, std::string & out)
  {
    if (len == 0) return 0;
    uint8_t b = data[0];
    size_t str_len = 0;
    size_t header = 1;
    if ((b & 0xe0) == 0xa0) {  // fixstr
      str_len = b & 0x1f;
    } else if (b == 0xd9 && len >= 2) {  // str8
      str_len = data[1];
      header = 2;
    } else if (b == 0xda && len >= 3) {  // str16
      str_len = static_cast<size_t>((data[1] << 8) | data[2]);
      header = 3;
    } else if (b == 0xdb && len >= 5) {  // str32
      str_len = static_cast<size_t>(
        (static_cast<uint32_t>(data[1]) << 24) | (static_cast<uint32_t>(data[2]) << 16) |
        (static_cast<uint32_t>(data[3]) << 8) | data[4]);
      header = 5;
    } else {
      return 0;  // not a string
    }
    if (len < header + str_len) return 0;
    out.assign(reinterpret_cast<const char *>(data + header), str_len);
    return header + str_len;
  }

  // Scan a MsgPack number (float64, float32, or int convertible to double).
  static size_t scan_number(const uint8_t * data, size_t len, double & out)
  {
    if (len == 0) return 0;
    uint8_t b = data[0];

    // float64
    if (b == 0xcb && len >= 9) {
      uint64_t bits =
        (static_cast<uint64_t>(data[1]) << 56) | (static_cast<uint64_t>(data[2]) << 48) |
        (static_cast<uint64_t>(data[3]) << 40) | (static_cast<uint64_t>(data[4]) << 32) |
        (static_cast<uint64_t>(data[5]) << 24) | (static_cast<uint64_t>(data[6]) << 16) |
        (static_cast<uint64_t>(data[7]) << 8) | static_cast<uint64_t>(data[8]);
      std::memcpy(&out, &bits, sizeof(out));
      return 9;
    }

    // float32
    if (b == 0xca && len >= 5) {
      uint32_t bits = (static_cast<uint32_t>(data[1]) << 24) |
                      (static_cast<uint32_t>(data[2]) << 16) |
                      (static_cast<uint32_t>(data[3]) << 8) | data[4];
      float f;
      std::memcpy(&f, &bits, sizeof(f));
      out = f;
      return 5;
    }

    // Integer convertible to double
    int64_t ival;
    size_t used = scan_int(data, len, ival);
    if (used > 0) {
      out = static_cast<double>(ival);
      return used;
    }

    return 0;
  }

  // ── Reader thread ───────────────────────────────────────────────────

  void reader_loop()
  {
    std::vector<uint8_t> buf;
    buf.reserve(8192);

    while (running_) {
      uint8_t tmp[4096];
      ssize_t n;

      // Copy socket fd without holding lock during I/O
      int fd_copy = -1;
      {
        std::lock_guard<std::mutex> lk(sock_mutex_);
        fd_copy = sock_fd_;
      }

      // Check if socket is valid
      if (fd_copy < 0) {
        // Reconnect without holding lock (register_method needs it)
        bool reconnected = false;
        {
          std::lock_guard<std::mutex> lk(sock_mutex_);
          if (sock_fd_ < 0) {
            reconnected = connect_socket_locked();
          }
        }
        if (reconnected) {
          std::lock_guard<std::mutex> hlk(handlers_mutex_);
          for (const auto & kv : float_handlers_) register_method(kv.first);
          for (const auto & kv : string_handlers_) register_method(kv.first);
        } else if (running_) {
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        continue;
      }

      // Perform select() and read() WITHOUT holding sock_mutex_
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(fd_copy, &rfds);
      struct timeval tv = {0, 100000};  // 100ms timeout
      if (select(fd_copy + 1, &rfds, nullptr, nullptr, &tv) <= 0) {
        continue;
      }

      n = ::read(fd_copy, tmp, sizeof(tmp));

      if (n <= 0) {
        // Invalidate socket only if it's still the same fd
        {
          std::lock_guard<std::mutex> lk(sock_mutex_);
          if (sock_fd_ == fd_copy) {
            ::close(sock_fd_);
            sock_fd_ = -1;
          }
        }
        continue;  // Reconnect on next iteration
      }

      // Process received data
      buf.insert(buf.end(), tmp, tmp + n);

      // Parse and dispatch as many messages as possible
      size_t off = 0;
      while (off < buf.size()) {
        Message msg;
        size_t consumed = try_parse_message(buf.data() + off, buf.size() - off, msg);
        if (consumed == 0) break;  // incomplete, wait for more data
        off += consumed;

        if (!msg.method.empty()) {
          dispatch(msg);
        }
      }

      if (off > 0) {
        buf.erase(buf.begin(), buf.begin() + off);
      }

      // Prevent unbounded buffer growth
      if (buf.size() > 65536) buf.clear();
    }
  }

  void dispatch(const Message & msg)
  {
    std::lock_guard<std::mutex> lk(handlers_mutex_);
    if (!msg.strings.empty()) {
      auto it = string_handlers_.find(msg.method);
      if (it != string_handlers_.end() && it->second) {
        it->second(msg.strings[0]);
        return;
      }
    }
    auto it = float_handlers_.find(msg.method);
    if (it != float_handlers_.end() && it->second) {
      it->second(msg.numbers);
    }
  }

  // ── State ───────────────────────────────────────────────────────────

  std::vector<std::string> socket_candidates_;
  std::string connected_path_;
  int sock_fd_{-1};
  std::mutex sock_mutex_;

  std::thread reader_thread_;
  std::atomic<bool> running_{false};

  std::map<std::string, FloatHandler> float_handlers_;
  std::map<std::string, StringHandler> string_handlers_;
  std::mutex handlers_mutex_;
};

#endif  // BRIDGE_RPC_CLIENT_HPP_
