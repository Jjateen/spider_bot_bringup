#pragma once

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
// Usage:
//   BridgeRPCClient client("/run/arduino-router/rpc.sock");
//   client.provide("imu", [](auto& p) { /* p = {ax,ay,az,gx,gy,gz,sample,ts} */ });
//   client.notify("set_servo_pwms", {102.0, 512.0, ...});
//   client.start();  // background reader thread

class BridgeRPCClient
{
public:
  using FloatHandler = std::function<void(const std::vector<double> &)>;

  BridgeRPCClient(const std::string & socket_path)
  : socket_path_(socket_path) {}

  ~BridgeRPCClient() { stop(); }

  // Connect and start the background reader thread.
  bool start()
  {
    if (running_) return true;
    if (!connect_socket()) return false;
    running_ = true;
    reader_thread_ = std::thread(&BridgeRPCClient::reader_loop, this);

    // Re-register any methods that were provided before start()
    std::lock_guard<std::mutex> lk(handlers_mutex_);
    for (const auto & kv : handlers_) {
      register_method(kv.first);
    }
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

  // Register a handler for inbound notifications/requests on `method`.
  // Must be called before start(), or provide() will auto-start.
  void provide(const std::string & method, FloatHandler handler)
  {
    bool do_start = false;
    {
      std::lock_guard<std::mutex> lk(handlers_mutex_);
      handlers_[method] = std::move(handler);
    }
    if (!running_) {
      do_start = true;
    } else {
      register_method(method);
    }
    if (do_start) start();
  }

private:
  // ── Socket ──────────────────────────────────────────────────────────

  bool connect_socket()
  {
    std::lock_guard<std::mutex> lk(sock_mutex_);
    sock_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock_fd_ < 0) return false;

    struct sockaddr_un addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

    if (::connect(sock_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;
      return false;
    }
    return true;
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

      // Read response: [1, msgid, nil, true]  (4-element fixarray)
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
        // Check if we have a complete response (first byte = 0x94 = fixarray of 4)
        if (!resp.empty() && resp[0] == 0x94 && resp.size() >= 4) {
          // Minimal check: response is at least 4 elements worth of bytes
          return true;
        }
      }
    }
    return !resp.empty();
  }

  // ── MsgPack packers ─────────────────────────────────────────────────

  static void pack_byte(std::vector<uint8_t> & buf, uint8_t b)
  {
    buf.push_back(b);
  }

  static void pack_array(std::vector<uint8_t> & buf, size_t n)
  {
    if (n <= 15) { pack_byte(buf, 0x90 | n); return; }
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
    if (val >= 0 && val <= 0x7F) { pack_byte(buf, val); return; }
    if (val >= -32 && val <= -1) { pack_byte(buf, 0xe0 | (val & 0x1F)); return; }
    if (val >= -128 && val <= 127) { pack_byte(buf, 0xd0); pack_byte(buf, val & 0xFF); return; }
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
    buf.insert(buf.end(), (uint8_t *)&bits, (uint8_t *)&bits + 8);
  }

  static void pack_nil(std::vector<uint8_t> & buf)
  {
    pack_byte(buf, 0xc0);
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
  // On success, `type`, `method`, `params` are filled for notifications.
  size_t try_parse_message(
    const uint8_t * data, size_t len,
    int64_t & out_type,
    std::string & out_method,
    std::vector<double> & out_params)
  {
    if (len == 0) return 0;
    size_t off = 0;

    // Must start with a fixarray (we skip array16/32 for simplicity).
    // Notifications are [2, "method", [params]] — 3 elements (0x93).
    // Responses are [1, msgid, error, result] — 4 elements (0x94).
    if ((data[off] & 0xf0) != 0x90) return 1;  // skip garbage byte
    size_t arr_len = data[off] & 0x0f;
    off++;

    // If this is a response (4 elements), skip it entirely.
    // We handle responses synchronously in register_method().
    if (arr_len == 4) {
      // Walk past all 4 elements. Quick scan: skip the array.
      return len;  // consume everything (we don't care about async responses)
    }

    // Must be a notification (3 elements)
    if (arr_len != 3) return off;

    // Element 0: message type (must be 2)
    {
      int64_t t;
      size_t used = scan_int(data + off, len - off, t);
      if (used == 0) return 0;
      off += used;
      out_type = t;
    }

    // Element 1: method name (string)
    {
      size_t used = scan_str(data + off, len - off, out_method);
      if (used == 0) return 0;
      off += used;
    }

    // Element 2: params array
    {
      if (off >= len) return 0;
      if ((data[off] & 0xf0) != 0x90) return off + 1;
      size_t param_count = data[off] & 0x0f;
      off++;

      out_params.clear();
      out_params.reserve(param_count);
      for (size_t i = 0; i < param_count; ++i) {
        double val;
        size_t used = scan_number(data + off, len - off, val);
        if (used == 0) return 0;
        off += used;
        out_params.push_back(val);
      }
    }

    return off;
  }

  // Scan a MsgPack integer, return bytes consumed (0 = need more data).
  static size_t scan_int(const uint8_t * data, size_t len, int64_t & out)
  {
    if (len == 0) return 0;
    uint8_t b = data[0];
    if (b <= 0x7f) { out = b; return 1; }
    if (b >= 0xe0) { out = (int8_t)b; return 1; }
    if (b == 0xd0 && len >= 2) { out = (int8_t)data[1]; return 2; }
    if (b == 0xd1 && len >= 3) {
      out = (int16_t)((data[1] << 8) | data[2]);
      return 3;
    }
    if (b == 0xd2 && len >= 5) {
      out = (int32_t)(((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) |
                      ((uint32_t)data[3] << 8) | data[4]);
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
    out.assign((const char *)data + 1, str_len);
    return 1 + str_len;
  }

  // Scan a MsgPack number (float64, float32, or int convertible to double).
  static size_t scan_number(const uint8_t * data, size_t len, double & out)
  {
    if (len == 0) return 0;
    uint8_t b = data[0];

    // float64
    if (b == 0xcb && len >= 9) {
      uint64_t bits = ((uint64_t)data[1] << 56) | ((uint64_t)data[2] << 48) |
                      ((uint64_t)data[3] << 40) | ((uint64_t)data[4] << 32) |
                      ((uint64_t)data[5] << 24) | ((uint64_t)data[6] << 16) |
                      ((uint64_t)data[7] << 8) | (uint64_t)data[8];
      std::memcpy(&out, &bits, sizeof(out));
      return 9;
    }

    // float32
    if (b == 0xca && len >= 5) {
      uint32_t bits = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) |
                      ((uint32_t)data[3] << 8) | data[4];
      float f;
      std::memcpy(&f, &bits, sizeof(f));
      out = f;
      return 5;
    }

    // Integer convertible to double
    int64_t ival;
    size_t used = scan_int(data, len, ival);
    if (used > 0) {
      out = (double)ival;
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
      {
        std::lock_guard<std::mutex> lk(sock_mutex_);
        if (sock_fd_ < 0) {
          if (!reconnect()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
          }
        }

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sock_fd_, &rfds);
        struct timeval tv = {0, 100000};
        if (select(sock_fd_ + 1, &rfds, nullptr, nullptr, &tv) <= 0) continue;

        n = ::read(sock_fd_, tmp, sizeof(tmp));
      }
      if (n <= 0) {
        if (running_) {
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        continue;
      }

      buf.insert(buf.end(), tmp, tmp + n);

      // Parse and dispatch as many messages as possible
      size_t off = 0;
      while (off < buf.size()) {
        int64_t msg_type = 0;
        std::string method;
        std::vector<double> params;
        size_t consumed = try_parse_message(
          buf.data() + off, buf.size() - off, msg_type, method, params);
        if (consumed == 0) break;  // incomplete, wait for more data
        off += consumed;

        if (msg_type == 2 && !method.empty()) {
          std::lock_guard<std::mutex> lk(handlers_mutex_);
          auto it = handlers_.find(method);
          if (it != handlers_.end() && it->second) {
            it->second(params);
          }
        }
      }

      if (off > 0) {
        buf.erase(buf.begin(), buf.begin() + off);
      }

      // Prevent unbounded buffer growth
      if (buf.size() > 65536) buf.clear();
    }
  }

  bool reconnect()
  {
    if (sock_fd_ >= 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;
    }
    return connect_socket();
  }

  // ── State ───────────────────────────────────────────────────────────

  std::string socket_path_;
  int sock_fd_{-1};
  std::mutex sock_mutex_;

  std::thread reader_thread_;
  std::atomic<bool> running_{false};

  std::map<std::string, FloatHandler> handlers_;
  std::mutex handlers_mutex_;
};
