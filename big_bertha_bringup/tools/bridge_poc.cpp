// bridge_poc — standalone PoC benchmark for the arduino-router Bridge RPC
// protocol.  Measures sustained notify throughput, IMU receive rate, and
// round-trip latency.  No ROS dependencies.
//
// Usage:
//   ./bridge_poc [--duration 30] [--socket /run/arduino-router/rpc.sock]
//
// Build:
//   g++ -std=c++17 -O2 -pthread bridge_poc.cpp -o bridge_poc
//   (or via colcon — see CMakeLists.txt)
//
// Pass criteria (#54):
//   12 servos @ 50 Hz down + IMU @ ≥100 Hz up at 460800 baud
//   with <5 ms P99 jitter.

#include "../src/bridge_rpc_client.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// ── Stats collector (lock-free counters) ──────────────────────────────

struct Stats {
  std::atomic<uint64_t> imu_received{0};
  std::atomic<uint64_t> pong_received{0};
  std::atomic<uint64_t> servo_sent{0};
  std::atomic<uint64_t> ping_sent{0};
  std::atomic<uint64_t> send_errors{0};

  // Latency samples (nanoseconds) — circular buffer for P99
  static constexpr size_t MAX_SAMPLES = 10000;
  std::vector<int64_t> rtt_samples;
  std::mutex rtt_mutex;

  void record_rtt(int64_t ns)
  {
    std::lock_guard<std::mutex> lk(rtt_mutex);
    if (rtt_samples.size() < MAX_SAMPLES) {
      rtt_samples.push_back(ns);
    }
  }

  double p99_rtt_ms()
  {
    std::lock_guard<std::mutex> lk(rtt_mutex);
    if (rtt_samples.empty()) return 0.0;
    auto v = rtt_samples;
    std::sort(v.begin(), v.end());
    size_t idx = static_cast<size_t>(v.size() * 0.99);
    if (idx >= v.size()) idx = v.size() - 1;
    return v[idx] / 1.0e6;
  }

  double avg_rtt_ms()
  {
    std::lock_guard<std::mutex> lk(rtt_mutex);
    if (rtt_samples.empty()) return 0.0;
    int64_t sum = 0;
    for (auto s : rtt_samples) sum += s;
    return (sum / (double)rtt_samples.size()) / 1.0e6;
  }
};

static Stats g_stats;
static std::atomic<bool> g_running{true};

void signal_handler(int) { g_running = false; }

// ── Format helpers ────────────────────────────────────────────────────

std::string timestamp()
{
  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  std::tm tm;
  localtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%H:%M:%S", &tm);
  std::ostringstream ss;
  ss << buf << "." << std::setw(3) << std::setfill('0') << ms.count();
  return ss.str();
}

// ── Main ──────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  std::string socket_path = "/run/arduino-router/rpc.sock";
  int duration_sec = 30;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
      duration_sec = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--socket") == 0 && i + 1 < argc) {
      socket_path = argv[++i];
    } else {
      std::cerr << "Usage: " << argv[0]
                << " [--duration SEC] [--socket PATH]\n";
      return 1;
    }
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::cout << "╔══════════════════════════════════════════════════════╗\n";
  std::cout << "║  Bridge RPC PoC — Router throughput benchmark       ║\n";
  std::cout << "╠══════════════════════════════════════════════════════╣\n";
  std::cout << "║  Socket: " << std::left << std::setw(45) << socket_path << "║\n";
  std::cout << "║  Duration: " << std::setw(43) << (std::to_string(duration_sec) + " s") << "║\n";
  std::cout << "╚══════════════════════════════════════════════════════╝\n\n";

  // ── Connect ──────────────────────────────────────────────────────

  BridgeRPCClient client(socket_path);

  // Register IMU handler
  client.provide("imu", [](const std::vector<double> &) {
    g_stats.imu_received.fetch_add(1, std::memory_order_relaxed);
  });

  // Register pong handler (for RTT measurement)
  client.provide("pong", [](const std::vector<double> & params) {
    auto now = std::chrono::steady_clock::now();
    g_stats.pong_received.fetch_add(1, std::memory_order_relaxed);
    // params = [timestamp_when_sent_ns]
    if (!params.empty()) {
      int64_t sent_ns = static_cast<int64_t>(params[0]);
      auto received_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
      g_stats.record_rtt(received_ns - sent_ns);
    }
  });

  if (!client.start()) {
    std::cerr << "[FATAL] Could not connect to router socket: " << socket_path << "\n";
    std::cerr << "  Is arduino-router running? Check: systemctl status arduino-router\n";
    return 1;
  }
  std::cout << "[" << timestamp() << "] Connected to router\n";

  // ── Test loop ────────────────────────────────────────────────────

  auto test_start = std::chrono::steady_clock::now();
  auto deadline = test_start + std::chrono::seconds(duration_sec);
  auto next_report = test_start + std::chrono::seconds(5);
  auto next_ping = test_start;

  // Pre-build servo command: 12 floats at center position
  std::vector<double> servo_cmd(12, 0.0);

  // Timed send loop
  auto next_servo = test_start;
  constexpr auto SERVO_INTERVAL = std::chrono::milliseconds(20);  // 50 Hz
  constexpr auto PING_INTERVAL = std::chrono::milliseconds(1000); // 1 Hz

  // Wait for some IMU data to arrive before measuring
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::cout << "[" << timestamp() << "] Benchmark started"
            << " — sending servos @ 50 Hz, logging every 5 s\n\n";

  uint64_t last_servo_sent = 0;
  uint64_t last_imu_rcvd = 0;

  while (g_running && std::chrono::steady_clock::now() < deadline) {
    auto now = std::chrono::steady_clock::now();

    // Send servo command at 50 Hz
    if (now >= next_servo) {
      client.notify("set_servo_pwms", servo_cmd);
      g_stats.servo_sent.fetch_add(1, std::memory_order_relaxed);
      next_servo += SERVO_INTERVAL;
      if (next_servo < now) next_servo = now + SERVO_INTERVAL;
    }

    // Send ping for RTT measurement
    if (now >= next_ping) {
      auto ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
      client.notify_int("ping", {ts});
      g_stats.ping_sent.fetch_add(1, std::memory_order_relaxed);
      next_ping = now + PING_INTERVAL;
    }

    // Report every 5 seconds
    if (now >= next_report) {
      auto sent = g_stats.servo_sent.load(std::memory_order_relaxed);
      auto imu = g_stats.imu_received.load(std::memory_order_relaxed);
      auto dt_s = std::chrono::duration<double>(
        now - (next_report - std::chrono::seconds(5))).count();

      double servo_hz = (sent - last_servo_sent) / dt_s;
      double imu_hz = (imu - last_imu_rcvd) / dt_s;

      std::cout << "[" << timestamp() << "] "
                << "servo TX: " << std::fixed << std::setprecision(1) << servo_hz << " Hz"
                << "  |  IMU RX: " << imu_hz << " Hz"
                << "  |  RTT avg: " << g_stats.avg_rtt_ms() << " ms"
                << "  |  P99: " << g_stats.p99_rtt_ms() << " ms"
                << "\n";

      last_servo_sent = sent;
      last_imu_rcvd = imu;
      next_report = now + std::chrono::seconds(5);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }

  // ── Final report ─────────────────────────────────────────────────

  auto test_end = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration<double>(test_end - test_start).count();

  auto total_servo = g_stats.servo_sent.load();
  auto total_imu = g_stats.imu_received.load();
  auto total_ping = g_stats.ping_sent.load();
  auto total_pong = g_stats.pong_received.load();

  std::cout << "\n╔══════════════════════════════════════════════════════╗\n";
  std::cout << "║  Results                                             ║\n";
  std::cout << "╠══════════════════════════════════════════════════════╣\n";
  std::cout << "║  Duration:       " << std::right << std::setw(36)
            << elapsed << " s  ║\n";
  std::cout << "║  Servo TX:       " << std::setw(10) << total_servo
            << " msgs  (" << std::setw(6) << std::fixed << std::setprecision(1)
            << (total_servo / elapsed) << " Hz)  ║\n";
  std::cout << "║  IMU RX:         " << std::setw(10) << total_imu
            << " msgs  (" << std::setw(6) << (total_imu / elapsed)
            << " Hz)  ║\n";
  std::cout << "║  Ping/Pong:      " << std::setw(10) << total_pong
            << "/" << std::left << std::setw(5) << total_ping
            << std::right << " (" << std::setw(6)
            << (total_ping > 0 ? (100.0 * total_pong / total_ping) : 0.0)
            << "%)  ║\n";
  std::cout << "║  RTT avg:        " << std::setw(36)
            << g_stats.avg_rtt_ms() << " ms  ║\n";
  std::cout << "║  RTT P99:        " << std::setw(36)
            << g_stats.p99_rtt_ms() << " ms  ║\n";
  std::cout << "╚══════════════════════════════════════════════════════╝\n";

  // Decision
  double imu_hz = total_imu / elapsed;
  double servo_hz = total_servo / elapsed;
  double p99 = g_stats.p99_rtt_ms();

  bool pass = (imu_hz >= 100.0) && (servo_hz >= 45.0) && (p99 < 5.0);
  std::cout << "\n  ── Verdict: " << (pass ? "PASS ✓" : "FAIL ✗") << " ──\n";
  if (!pass) {
    if (imu_hz < 100.0)
      std::cout << "    IMU rate " << imu_hz << " Hz < 100 Hz target\n";
    if (servo_hz < 45.0)
      std::cout << "    Servo rate " << servo_hz << " Hz < 45 Hz target\n";
    if (p99 >= 5.0)
      std::cout << "    P99 jitter " << p99 << " ms >= 5 ms limit\n";
    std::cout << "  → Consider fallback: bare Zephyr + binary protocol @ 1-2 Mbaud\n";
  }

  client.stop();
  return pass ? 0 : 1;
}
