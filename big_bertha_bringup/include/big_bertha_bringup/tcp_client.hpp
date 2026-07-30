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

#ifndef BIG_BERTHA_BRINGUP__TCP_CLIENT_HPP_
#define BIG_BERTHA_BRINGUP__TCP_CLIENT_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace big_bertha_bringup
{

class TcpClient
{
public:
  TcpClient() = default;
  ~TcpClient() { close(); }

  TcpClient(const TcpClient &) = delete;
  TcpClient & operator=(const TcpClient &) = delete;
  TcpClient(TcpClient && other) noexcept : fd_(other.fd_), read_buf_(std::move(other.read_buf_))
  {
    other.fd_ = -1;
  }
  TcpClient & operator=(TcpClient && other) noexcept
  {
    if (this != &other) {
      close();
      fd_ = other.fd_;
      read_buf_ = std::move(other.read_buf_);
      other.fd_ = -1;
    }
    return *this;
  }

  bool connect(const std::string & host, int port)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    close_locked();

    struct sockaddr_in addr
    {
    };
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) <= 0) {
      return false;
    }

    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) return false;

    // Disable Nagle's algorithm — small IMU/servo commands (< 100 bytes) must
    // be sent immediately, not delayed up to 200 ms waiting for a batch.
    int one = 1;
    ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    if (::connect(fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    return true;
  }

  void close()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    close_locked();
  }

  bool is_connected() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return fd_ >= 0;
  }

  bool send(const std::string & data)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (fd_ < 0) return false;

    ssize_t n = ::send(fd_, data.data(), data.size(), MSG_NOSIGNAL);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
      n = ::send(fd_, data.data(), data.size(), MSG_NOSIGNAL);
    }
    if (n <= 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    return true;
  }

  bool read_line(std::string & line)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    line.clear();

    auto pos = read_buf_.find('\n');
    while (pos == std::string::npos) {
      if (!refill_buf_locked()) return false;
      pos = read_buf_.find('\n');
    }

    line = read_buf_.substr(0, pos);
    read_buf_.erase(0, pos + 1);
    return true;
  }

  void drain()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (fd_ < 0) return;

    char buf[4096];
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    struct timeval tv = {0, 0};
    if (select(fd_ + 1, &rfds, nullptr, nullptr, &tv) > 0) {
      ssize_t n;
      do {
        n = ::read(fd_, buf, sizeof(buf));
        if (n > 0) {
          read_buf_.append(buf, n);
        }
      } while (n > 0);
      if (n < 0) {
        ::close(fd_);
        fd_ = -1;
      }
    }
  }

private:
  void close_locked()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    read_buf_.clear();
  }

  bool refill_buf_locked()
  {
    if (fd_ < 0) return false;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    struct timeval tv = {0, 500000};  // 500 ms timeout

    int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (ret <= 0) {
      if (ret < 0) {
        ::close(fd_);
        fd_ = -1;
      }
      return false;
    }

    char buf[4096];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    read_buf_.append(buf, n);
    return true;
  }

  int fd_{-1};
  mutable std::mutex mtx_;
  std::string read_buf_;
};

}  // namespace big_bertha_bringup

#endif  // BIG_BERTHA_BRINGUP__TCP_CLIENT_HPP_
