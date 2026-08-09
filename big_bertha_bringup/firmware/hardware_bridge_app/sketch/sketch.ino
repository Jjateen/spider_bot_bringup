// sketch.ino  —  Big Bertha hardware bridge (STM32U585)
//
// ── Physical role ──────────────────────────────────────────────────────
// STM32U585 M33 on Arduino UNO Q boards acts as real-time I2C controller,
// offloading servo timing and IMU polling from the A35 Linux side.
// Communication via Arduino Bridge RPC (mailbox-based IPC).
//
// ── Data flow ─────────────────────────────────────────────────────────
//   ROS 2 node (C++) → arduino-router (MsgPack-RPC socket) → this sketch
//
// ── Design principles ─────────────────────────────────────────────────
//   - Non-blocking: Bridge RPC handler never waits on I2C. Servo PWM
//     data arrives as a single comma-separated string (fixes 12-arg
//     Bridge RPC limit). The handler stores values and defers the
//     I2C write to loop() via g_pwm_dirty flag.
//   - No verify-on-hot-path: pca9685_verify_init() runs only in the
//     1 Hz status loop, never in the servo write path. Every servo
//     write is one I2C transaction — no re-init delays.
//   - 50 kHz I2C clock: STM32U585 I2C v2 driver has known TIMEOUT
//     issues (Zephyr #83550). 50 kHz is below spec but empirically
//     more reliable on this platform with STOP-START transactions.

#include <Arduino_RouterBridge.h>
#include <Wire.h>

#include <vector>

// ── I2C device addresses ──────────────────────────────────────────────
// g_bno_addr is set at init to whichever of 0x28/0x29 actually answers:
// the BNO055 answers at 0x28 by default and at 0x29 when its ADR pin is
// high, and the two are not interchangeable on a given breakout.
static uint8_t g_bno_addr = 0x28;
static const uint8_t PCA9685_ADDR = 0x40;

// ── PCA9685 register map ──────────────────────────────────────────────
static const uint8_t PCA9685_MODE1 = 0x00;
static const uint8_t PCA9685_PRE_SCALE = 0xFE;
static const uint8_t PCA9685_LED0_ON_L = 0x06;

// Physical PCA9685 channel → logical servo index (Isaac Sim convention)
// [0..2]=FL, [3..5]=FR, [6..8]=HL, [9..11]=HR
static const uint8_t PWM_CHANNEL_MAP[12] = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};

// ── Diagnostic state ──────────────────────────────────────────────────
// bit 0 = PCA9685 missing, bit 1 = BNO055 missing
static int g_i2c_scan = 0;
static bool g_ai_ok = false;
static bool g_imu_present = false;

// Current PWM off-counts for all 16 channels (0 = output low = servo off)
static uint16_t g_pwm[16] = {0};

// Set true by set_servo_pwms when new PWM data arrives; cleared by loop()
// after writing to the PCA9685. Volatile because handler runs in Bridge RPC
// background thread.
static volatile bool g_pwm_dirty = false;

static unsigned long g_last_imu_push = 0;
static unsigned long g_last_status_push = 0;
static unsigned long g_last_imu_diag_push = 0;  // imu_diag throttled (every 5 s)
static unsigned long g_last_blink = 0;
static int g_blink_code = 0;
static uint32_t g_imu_sample = 0;
static int g_servo_calls = 0;
static int g_ping_count = 0;

// ── Diagnostic counters ───────────────────────────────────────────────
static int g_pwm_write_attempts = 0;  // total write cycles attempted
static int g_pwm_write_fails = 0;     // cycles where at least one channel failed
static int g_pwm_write_oks = 0;       // successful channels in last cycle
static int g_pwm_last_fail_ch = -1;   // last physical channel that failed (-1 = none)
static int g_pwm_last_fail_code = 0;  // endTransmission() return: 0=ok, 2=NACK-addr, 3=NACK-data
static int g_set_servo_last_len = 0;  // data.length() from last set_servo_pwms call
static int g_set_servo_last_idx = 0;  // parsed field count (12 = clean)
static int g_pwm_readback_ch0 = -1;   // PCA9685 ch0 OFF register readback (-1 = read failed)
static volatile bool g_diag_pending = false;
static int g_diag_test_pwms[12];
static unsigned long g_diag_settle_start = 0;
static int g_diag_phase = 0;
static unsigned long g_diag_start_ms = 0;  // when phase 1 started, for timeout guard

// I2C error tracking for self-healing
static int g_i2c_consecutive_fails = 0;  // resets on any success
static bool g_i2c_busy = false;          // guard for BG-thread vs loop() race

// Servo command watchdog. Reports a stalled command stream; the legs hold
// their last commanded pose (see the check in loop()).
static unsigned long g_last_servo_cmd = 0;
static const unsigned long SERVO_TIMEOUT_MS = 250;  // 250ms watchdog
// Latch so one stall reports once, not on every loop cycle.
static bool g_servo_timed_out = false;

// 100 Hz IMU (10 ms) — BNO055 fusion modes cap their output at 100 Hz.
static const unsigned long IMU_INTERVAL = 10;
static const unsigned long STATUS_INTERVAL = 1000;

// ── I2C helpers ───────────────────────────────────────────────────────

static bool i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2c_read_bytes(uint8_t dev, uint8_t reg, uint8_t * buf, size_t len)
{
  g_i2c_busy = true;
  bool ok = false;
  for (int attempt = 0; attempt < 3; ++attempt) {
    Wire.beginTransmission(dev);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
      delay(1);
      continue;
    }
    if (Wire.requestFrom(dev, (uint8_t)len) != len) {
      delay(1);
      continue;
    }
    for (size_t i = 0; i < len; ++i) {
      buf[i] = Wire.read();
    }
    ok = true;
    break;
  }
  g_i2c_busy = false;
  if (ok) {
    g_i2c_consecutive_fails = 0;
  } else {
    ++g_i2c_consecutive_fails;
  }
  return ok;
}

// ── PCA9685 servo controller ─────────────────────────────────────────

static bool pca9685_init()
{
  if (!i2c_write_byte(PCA9685_ADDR, PCA9685_MODE1, 0x10)) return false;  // sleep
  delay(2);
  if (!i2c_write_byte(PCA9685_ADDR, PCA9685_PRE_SCALE, 121)) return false;  // 50 Hz
  delay(1);
  if (!i2c_write_byte(PCA9685_ADDR, PCA9685_MODE1, 0x20)) return false;  // wake + AI
  delay(2);
  return true;
}

// Write each active servo channel individually (5 bytes per I2C transaction).
// Takes a snapshot of g_pwm[] at the start so all 12 writes come from the
// same ROS message — a new servo command arriving mid-cycle sets
// g_pwm_dirty again and will be picked up on the next cycle.
static bool pca9685_write_servos()
{
  uint16_t snapshot[12];
  for (int i = 0; i < 12; ++i) snapshot[i] = g_pwm[PWM_CHANNEL_MAP[i]];

  ++g_pwm_write_attempts;
  g_pwm_write_oks = 0;
  g_pwm_last_fail_ch = -1;
  g_pwm_last_fail_code = 0;
  for (int i = 0; i < 12; ++i) {
    uint8_t ch = PWM_CHANNEL_MAP[i];
    uint16_t off = snapshot[i];
    uint8_t reg = PCA9685_LED0_ON_L + 4 * ch;
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(reg);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(off & 0xFF);
    Wire.write((off >> 8) & 0x0F);
    int code = Wire.endTransmission();
    if (code != 0) {
      ++g_pwm_write_fails;
      if (g_pwm_last_fail_ch < 0) {
        g_pwm_last_fail_ch = ch;
        g_pwm_last_fail_code = code;
      }
    } else {
      ++g_pwm_write_oks;
    }
  }
  return g_pwm_write_oks == 12;
}

static bool pca9685_verify_init()
{
  uint8_t mode1 = 0;
  if (!i2c_read_bytes(PCA9685_ADDR, PCA9685_MODE1, &mode1, 1)) return false;
  return (mode1 & 0x20) != 0 && (mode1 & 0x10) == 0;
}

// ── BNO055 IMU ───────────────────────────────────────────────────────

// The BNO055 is a system-in-package 9-axis IMU with an on-board sensor
// fusion MCU. It runs its own absolute-orientation filter (NDOF mode:
// accel + gyro + magnetometer) and exposes the fused quaternion plus raw
// accel/gyro/mag on the data registers, so no host-side AHRS is needed.
// The previous MPU-6500 had no magnetometer die at all, which is why yaw
// free-ran on gyro integration; the BNO055's fused heading is absolute.

// CHIP_ID (0x00) read at init: 0xA0 = BNO055 answering. Reported in
// hw_status so the part can be identified without a scope or a teardown.
static uint8_t g_bno_chip_id = 0x00;

// CALIB_STAT (0x35): four 2-bit fields, one per fused subsystem.
//   bits[1:0] = accel, bits[3:2] = mag, bits[5:4] = gyro, bits[7:6] = sys
// Each is 0 = uncalibrated .. 3 = fully calibrated. NDOF heading is only
// trustworthy once the mag calibration reaches 3 (figure-of-eight while
// level). Status registers are re-read before each imu_diag push.
static uint8_t g_bno_calib_stat = 0x00;
static uint8_t g_bno_sys_status = 0x00;
static uint8_t g_bno_sys_err = 0x00;
static uint8_t g_bno_unit_sel = 0x00;
static uint8_t g_bno_op_mode = 0x00;
static uint8_t g_bno_axis_config = 0x00;
static uint8_t g_bno_axis_sign = 0x00;

// Boot handshake. Pick whichever address answers (0x28 default, 0x29 when
// the ADR pin is high), then wait out the ~650 ms boot until CHIP_ID reads
// 0xA0. A bare write-ACK is NOT enough: the part can ACK address
// transactions while still booting and silently drop configuration writes,
// which would read back as a phantom "IMU missing".
static bool bno055_wait_ready()
{
  static const uint8_t candidates[2] = {0x28, 0x29};
  uint8_t chosen = 0;
  for (int i = 0; i < 2; ++i) {
    if (i2c_write_byte(candidates[i], 0x07, 0x00)) {  // PAGE_ID -> page 0
      chosen = candidates[i];
      break;
    }
  }
  if (chosen == 0) return false;

  g_bno_addr = chosen;
  for (int i = 0; i < 40; ++i) {
    uint8_t chip = 0;
    if (i2c_read_bytes(g_bno_addr, 0x00, &chip, 1) && chip == 0xA0) {
      return true;
    }
    delay(50);
  }
  return false;
}

static bool bno055_init()
{
  if (!bno055_wait_ready()) return false;
  delay(100);  // datasheet: writes right after first ACK can be silently dropped

  if (!i2c_write_byte(g_bno_addr, 0x3D, 0x00)) return false;  // OPR_MODE = CONFIG
  delay(25);
  i2c_write_byte(g_bno_addr, 0x3B, 0x00);  // UNIT_SEL: m/s^2 accel, dps gyro, Windows
  i2c_write_byte(g_bno_addr, 0x3F, 0x00);  // SYS_TRIGGER: reset clear, no self-test
  i2c_write_byte(g_bno_addr, 0x3E, 0x00);  // PWR_MODE: NORMAL
  delay(20);

  i2c_read_bytes(g_bno_addr, 0x00, &g_bno_chip_id, 1);
  if (g_bno_chip_id != 0xA0) return false;

  // NDOF: 9-axis absolute orientation. AXIS_MAP_CONFIG/SIGN are left at
  // their identity power-on defaults; mounting signs are applied on the
  // host (imu_axis_sign) so this board needs no part-specific trans.
  if (!i2c_write_byte(g_bno_addr, 0x3D, 0x0C)) return false;  // OPR_MODE = NDOF
  delay(30);
  return true;
}

static bool bno055_read(
  float & qw, float & qx, float & qy, float & qz,
  float & gx, float & gy, float & gz,
  float & ax, float & ay, float & az,
  float & mx, float & my, float & mz)
{
  uint8_t quat[8] = {0};  // QUA at 0x20..0x27, 2^14 = 1.0 per unit
  uint8_t gyro[6] = {0};  // GYR at 0x14..0x19, 16 LSB = 1 dps
  uint8_t acc[6] = {0};   // ACC at 0x08..0x0D, 100 LSB = 1 m/s^2
  uint8_t mag[6] = {0};   // MAG at 0x0E..0x13, 16 LSB = 1 uT
  if (!i2c_read_bytes(g_bno_addr, 0x20, quat, 8)) {
    qw = qx = qy = qz = gx = gy = gz = ax = ay = az = mx = my = mz = 0.0f;
    return false;
  }
  if (!i2c_read_bytes(g_bno_addr, 0x14, gyro, 6)) {
    qw = qx = qy = qz = gx = gy = gz = ax = ay = az = mx = my = mz = 0.0f;
    return false;
  }
  if (!i2c_read_bytes(g_bno_addr, 0x08, acc, 6)) {
    qw = qx = qy = qz = gx = gy = gz = ax = ay = az = mx = my = mz = 0.0f;
    return false;
  }
  // Magnetometer is optional: NDOF reads it internally for absolute yaw,
  // and whether the host publishes a field is its own call.
  i2c_read_bytes(g_bno_addr, 0x0E, mag, 6);

  // Quaternion: each component is on a 2^14 = 1.0 scale.
  qw = ((int16_t)((quat[1] << 8) | quat[0])) / 16384.0f;
  qx = ((int16_t)((quat[3] << 8) | quat[2])) / 16384.0f;
  qy = ((int16_t)((quat[5] << 8) | quat[4])) / 16384.0f;
  qz = ((int16_t)((quat[7] << 8) | quat[6])) / 16384.0f;

  // Gyro scale 16 LSB = 1 dps; convert to rad/s.
  gx = ((int16_t)((gyro[1] << 8) | gyro[0])) / 16.0f * (PI / 180.0f);
  gy = ((int16_t)((gyro[3] << 8) | gyro[2])) / 16.0f * (PI / 180.0f);
  gz = ((int16_t)((gyro[5] << 8) | gyro[4])) / 16.0f * (PI / 180.0f);

  // Accel, m/s^2 mode: 100 LSB = 1 m/s^2. Includes gravity; the host
  // keeps its own gravity bookkeeping (ZUPT + dead-reckon subtract 9.81).
  ax = ((int16_t)((acc[1] << 8) | acc[0])) / 100.0f;
  ay = ((int16_t)((acc[3] << 8) | acc[2])) / 100.0f;
  az = ((int16_t)((acc[5] << 8) | acc[4])) / 100.0f;

  // Mag scale 16 LSB = 1 uT -> Tesla for sensor_msgs/MagneticField.
  mx = ((int16_t)((mag[1] << 8) | mag[0])) / 16.0f * 1e-6f;
  my = ((int16_t)((mag[3] << 8) | mag[2])) / 16.0f * 1e-6f;
  mz = ((int16_t)((mag[5] << 8) | mag[4])) / 16.0f * 1e-6f;
  return true;
}

// Re-read everything the imu_diag string depends on. Runs every 5 s so
// calibration progress is observable without a scope: NDOF fusion
// converges over time and the CALIB_STAT nibbles are the health readout.
static void read_status_registers()
{
  i2c_read_bytes(g_bno_addr, 0x00, &g_bno_chip_id, 1);
  i2c_read_bytes(g_bno_addr, 0x35, &g_bno_calib_stat, 1);
  i2c_read_bytes(g_bno_addr, 0x39, &g_bno_sys_status, 1);
  i2c_read_bytes(g_bno_addr, 0x3A, &g_bno_sys_err, 1);
  i2c_read_bytes(g_bno_addr, 0x3B, &g_bno_unit_sel, 1);
  i2c_read_bytes(g_bno_addr, 0x3D, &g_bno_op_mode, 1);
  i2c_read_bytes(g_bno_addr, 0x41, &g_bno_axis_config, 1);
  i2c_read_bytes(g_bno_addr, 0x42, &g_bno_axis_sign, 1);
}

// ── Diagnostics ───────────────────────────────────────────────────────

static int i2c_scan_devices()
{
  int missing = 0;
  Wire.beginTransmission(PCA9685_ADDR);
  if (Wire.endTransmission() != 0) missing |= 1;
  Wire.beginTransmission(g_bno_addr);
  if (Wire.endTransmission() != 0) missing |= 2;
  return missing;
}

// LED blink codes (non-blocking)
//   code 0: solid ON              — everything OK
//   code 1: 100ms period          — PCA9685 missing
//   code 2: 400ms period          — PCA9685 init fail
//   code 3: 2000ms period         — BNO055 missing
static void blink_update()
{
  unsigned long now = millis();

  // Determine which code should be active
  int code = 0;
  if (g_i2c_scan & 1)
    code = 1;
  else if (!g_ai_ok)
    code = 2;
  else if (g_i2c_scan & 2)
    code = 3;

  // Changed code → reset blink phase
  if (code != g_blink_code) {
    g_blink_code = code;
    g_last_blink = now;
    if (code == 0) {
      digitalWrite(LED_BUILTIN, HIGH);
      return;
    }
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  if (code == 0) return;  // solid ON

  unsigned long period = (code == 1) ? 100UL : (code == 2) ? 400UL : 2000UL;
  unsigned long half = period / 2;
  bool on = (now - g_last_blink) < half;
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
  if (now - g_last_blink >= period) g_last_blink = now;
}

// ── Bridge RPC handlers ───────────────────────────────────────────────

void set_servo_pwms(String data)
{
  ++g_servo_calls;
  g_last_servo_cmd = millis();  // Timestamp for watchdog

  // Parse comma-separated PWM values (e.g. "307,153,512,...")
  // Single-arg format avoids the Bridge RPC 12-argument limit.
  g_set_servo_last_len = data.length();
  int vals[12];
  int idx = 0;
  int start = 0;
  int len = data.length();
  for (int i = 0; i <= len && idx < 12; ++i) {
    if (i == len || data.charAt(i) == ',') {
      // data.substring().toInt() heap-allocates a String per field, and this
      // runs at 200 Hz (12 fields) -> ~2400 allocations/sec. The M33 heap is
      // small and the sketch was observed to die ~20 s after boot under this
      // churn. Parse the digits in place instead.
      int v = 0;
      for (int j = start; j < i; ++j) {
        const char c = data.charAt(j);
        if (c >= '0' && c <= '9') {
          v = v * 10 + (c - '0');
        }
      }
      vals[idx++] = v;
      start = i + 1;
    }
  }
  g_set_servo_last_idx = idx;
  if (idx != 12) return;

  for (int i = 0; i < 12; ++i) g_pwm[PWM_CHANNEL_MAP[i]] = (uint16_t)vals[i];

  g_pwm_dirty = true;
}

void on_ping()
{
  ++g_ping_count;
  Bridge.notify("pong", g_ping_count);
}

void on_scan_i2c()
{
  if (g_i2c_busy) return;
  std::vector<uint8_t> found;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) found.push_back(addr);
  }
  Bridge.notify("i2c_scan", found);
}

// ── Servo diagnostic: schedules a test for loop() ─────────────────────
// Stores test PWMs, defers all I2C work to loop() so Bridge.update()
// never blocks waiting on I2C.
void on_servo_diag()
{
  // Distribute test values evenly across the configured PWM range.
  // Must match pwm_min/pwm_max in hardware_bridge.yaml.
  const int pwm_min = 102;  // 0.5ms for wide-range clone
  const int pwm_max = 512;  // 2.5ms for wide-range clone
  const int range = pwm_max - pwm_min;

  for (int i = 0; i < 12; ++i) {
    // Evenly distributed: 102, 139, 176, 213, ..., 512
    g_diag_test_pwms[i] = pwm_min + (i * range / 11);
  }
  g_diag_pending = true;
}

// ── Setup & Loop ──────────────────────────────────────────────────────

void setup()
{
  Wire.begin();
  Wire.setClock(50000);  // 50 kHz — STM32U585 I2C v2 driver is unreliable above this

  g_i2c_scan = i2c_scan_devices();
  g_imu_present = bno055_init();
  read_status_registers();
  g_ai_ok = pca9685_init() && pca9685_verify_init();

  // Initialize Bridge RPC.  If begin() fails, started=false and the
  // background thread skips update() — incoming RPCs from the Python
  // relay (set_servo_pwms, ping) would never be dispatched.  We call
  // Bridge.update() directly in loop() as a safety net, so we do NOT
  // retry begin() here (retrying would leak transport/client/server
  // objects).
  bool bridge_ok = Bridge.begin(460800);
  if (!bridge_ok) {
    // Blink code 4 = Bridge RPC negotiation failed (router unreachable
    // or responded false).  The loop-update fallback will still process
    // incoming RPCs on the regular channel.
  }

  // Register RPC handlers on the regular channel (processed by the
  // background thread when started, ALWAYS by our loop() update call).
  Bridge.provide("set_servo_pwms", set_servo_pwms);
  Bridge.provide("scan_i2c", on_scan_i2c);
  Bridge.provide("ping", on_ping);
  Bridge.provide("servo_diag", on_servo_diag);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
  // Process incoming RPC calls from the ROS 2 hardware_bridge_node
  // (set_servo_pwms, ping, scan_i2c) regardless of whether the background
  // thread started successfully. update() is guarded by internal mutexes so
  // redundant calls from the background thread are harmless.
  Bridge.update();

  unsigned long now = millis();

  // ── Servo command watchdog ────────────────────────────────────────
  // Runs every cycle, deliberately outside the g_pwm_dirty gate below. The
  // case this guards is commands having STOPPED, and in that case nothing
  // sets g_pwm_dirty, so a watchdog nested inside the gate can only fire when
  // commands are still arriving — the one situation it is not needed. Gated,
  // it also mistook the 1 Hz health check's g_pwm_dirty re-set (which does
  // not refresh g_last_servo_cmd) for a stale command and parked every
  // channel at neutral once a second.
  //
  // It reports the stall and stops refreshing; it does NOT reposition the
  // legs. Driving all twelve channels to neutral mid-stride slams the frame
  // into a pose the gait never asked for, and because the PCA9685 latches its
  // last value anyway, holding is both safer mechanically and the same amount
  // of code. The web control server had to add a 100 ms heartbeat purely to
  // stop the old neutral-park yanking manually placed joints back to centre.
  //
  // g_last_servo_cmd == 0 means no command has ever arrived, so there is
  // nothing to have stalled yet.
  if (g_last_servo_cmd != 0 && (now - g_last_servo_cmd) > SERVO_TIMEOUT_MS) {
    if (!g_servo_timed_out) {  // edge-triggered: report once per stall
      g_servo_timed_out = true;
      Bridge.notify("servo_timeout", (float)(now - g_last_servo_cmd));
    }
  } else {
    g_servo_timed_out = false;
  }

  // ── Deferred servo write ──────────────────────────────────────────
  // Write new PWM values to the PCA9685 if the Python relay sent them.
  // No verify/init check on the hot path — health checks happen at 1 Hz.
  if (g_pwm_dirty) {
    g_pwm_dirty = false;  // optimistic clear — re-set below on failure
    if (!pca9685_write_servos()) {
      g_pwm_dirty = true;  // transaction failed, retry next cycle
    }
  }

  // ── Deferred servo diagnostic ──────────────────────────────────────
  // Non-blocking state machine so that Bridge.update() stays responsive.
  if (g_diag_pending) {
    if (g_diag_phase == 0) {
      // Phase 0: write test PWMs to all 12 active channels
      for (int i = 0; i < 12; ++i) {
        uint8_t ch = PWM_CHANNEL_MAP[i];
        uint16_t val = (uint16_t)g_diag_test_pwms[i];
        uint8_t reg = PCA9685_LED0_ON_L + 4 * ch;
        Wire.beginTransmission(PCA9685_ADDR);
        Wire.write(reg);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(val & 0xFF);
        Wire.write((val >> 8) & 0x0F);
        Wire.endTransmission();
      }
      g_diag_settle_start = millis();
      g_diag_start_ms = millis();
      g_diag_phase = 1;
    } else if (g_diag_phase == 1) {
      // Timeout guard: abort if diag takes > 300ms total
      if (millis() - g_diag_start_ms > 300) {
        Bridge.notify("servo_diag_result", 0, 0, -1, 0, 0);
        g_diag_pending = false;
        g_diag_phase = 0;
      } else if (millis() - g_diag_settle_start >= 5) {
        // Phase 1: settle elapsed — read back and report
        int readback[12];
        int pass[12];
        for (int i = 0; i < 12; ++i) {
          uint8_t ch = PWM_CHANNEL_MAP[i];
          uint8_t reg = PCA9685_LED0_ON_L + 4 * ch + 2;  // OFF_L
          uint8_t off_l = 0, off_h = 0;
          bool ok = i2c_read_bytes(PCA9685_ADDR, reg, &off_l, 1) &&
                    i2c_read_bytes(PCA9685_ADDR, reg + 1, &off_h, 1);
          if (ok) {
            readback[i] = off_l | ((off_h & 0x0F) << 8);
          } else {
            readback[i] = -1;
          }
          pass[i] = (ok && abs(readback[i] - g_diag_test_pwms[i]) <= 2) ? 1 : 0;
        }

        uint8_t mode1 = 0;
        bool mode1_ok = i2c_read_bytes(PCA9685_ADDR, PCA9685_MODE1, &mode1, 1);
        int mode1_val = mode1_ok ? (int)mode1 : -1;

        bool pca_present = (i2c_scan_devices() & 1) == 0;

        String report = "{\"pca_present\":";
        report += pca_present ? "true" : "false";
        report += ",\"mode1\":";
        report += mode1_val;
        report += ",\"ai_ok\":";
        report += (mode1_ok && (mode1 & 0x20)) ? "true" : "false";
        report += ",\"test_pwms\":[";
        for (int i = 0; i < 12; ++i) {
          if (i > 0) report += ",";
          report += g_diag_test_pwms[i];
        }
        report += "],\"readback\":[";
        for (int i = 0; i < 12; ++i) {
          if (i > 0) report += ",";
          report += readback[i];
        }
        report += "],\"pass\":[";
        for (int i = 0; i < 12; ++i) {
          if (i > 0) report += ",";
          report += pass[i];
        }
        report += "]}";
        Bridge.notify("servo_diag_result", report);

        g_diag_pending = false;
        g_diag_phase = 0;
      }
    }
  }

  // Push IMU at 100 Hz (only if sensor was detected).
  // Encoded as ONE comma-separated string: Bridge RPC drops arguments past
  // its limit (see set_servo_pwms / hw_status), and the old 11-arg notify
  // was seen to misframe the router. Order:
  //   qw,qx,qy,qz, gx,gy,gz, ax,ay,az, mx,my,mz, sample, ts
  if (g_imu_present && now - g_last_imu_push >= IMU_INTERVAL) {
    g_last_imu_push = now;
    float qw, qx, qy, qz, gx, gy, gz, ax, ay, az, mx = 0, my = 0, mz = 0;
    if (bno055_read(qw, qx, qy, qz, gx, gy, gz, ax, ay, az, mx, my, mz)) {
      char imu_buf[192];
      int off = snprintf(
        imu_buf, sizeof(imu_buf), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%lu",
        qw, qx, qy, qz, gx, gy, gz, ax, ay, az, mx, my, mz, g_imu_sample++, (unsigned long)now);
      if (off >= (int)sizeof(imu_buf)) off = sizeof(imu_buf) - 1;
      imu_buf[off] = '\0';
      Bridge.notify("imu", (const char *)imu_buf);
    }
  }

  // Push hardware status at 1 Hz
  if (now - g_last_status_push >= STATUS_INTERVAL) {
    g_last_status_push = now;
    g_i2c_scan = i2c_scan_devices();
    g_imu_present = (g_i2c_scan & 2) == 0;
    bool ai = false;
    if (!(g_i2c_scan & 1)) {
      ai = pca9685_verify_init();
      if (!ai) {
        ai = pca9685_init() && pca9685_verify_init();
        if (ai) g_pwm_dirty = true;  // flush RAM state after sleep/wake reset
      }
    }
    g_ai_ok = ai;

    // Read back PCA9685 channel 0 OFF register (two 1-byte reads for reliability)
    uint8_t off_l = 0, off_h = 0;
    if (
      i2c_read_bytes(PCA9685_ADDR, PCA9685_LED0_ON_L + 2, &off_l, 1) &&
      i2c_read_bytes(PCA9685_ADDR, PCA9685_LED0_ON_L + 3, &off_h, 1)) {
      g_pwm_readback_ch0 = off_l | ((off_h & 0x0F) << 8);
    } else {
      g_pwm_readback_ch0 = -1;
    }

    if (g_i2c_consecutive_fails >= 5) {
      pca9685_init();
      g_pwm_dirty = true;
      g_i2c_consecutive_fails = 0;
    }

    // hw_status as ONE comma-separated string, matching the set_servo_pwms /
    // imu_diag pattern: Bridge RPC drops arguments past a limit (~12, see the
    // servo path at set_servo_pwms), and the 11-arg notify was seen to misframe
    // the router (log: "invalid packet, expected array, got: int8") which
    // dropped THIS notification and swallowed the adjacent imu_diag too.
    // The host-side hardware_bridge_node parses "scan,ai,servo_calls,...".
    char hs[80];
    int hso = 0;
    hso += snprintf(
      hs + hso, sizeof(hs) - hso, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", g_i2c_scan, ai ? 1 : 0,
      g_servo_calls, g_ping_count, g_pwm_write_attempts, g_pwm_write_fails, g_pwm_last_fail_ch,
      g_pwm_last_fail_code, g_set_servo_last_len, g_set_servo_last_idx, g_pwm_readback_ch0);
    if (hso >= (int)sizeof(hs)) hso = sizeof(hs) - 1;
    hs[hso] = '\0';
    Bridge.notify("hw_status", (const char *)hs);

    // IMU diagnostics as a single string: part identity, operation mode,
    // system health and the per-subsystem calibration nibbles. One
    // argument, so nothing can be silently truncated.
    //
    // Built with snprintf into a fixed stack buffer (no heap String churn)
    // and kept under 256 bytes total because the sketch tells the router
    // $/setMaxMsgSize = 256; an oversized packet made the router mis-frame
    // the serial stream (log: "invalid packet, expected array, got: int8")
    // and the heap String churn starved the small M33 heap around ~20 s after
    // boot. imu_diag is debug-only so it is also throttled to 5 s.
    if (now - g_last_imu_diag_push >= 5000) {
      g_last_imu_diag_push = now;
      read_status_registers();
      char d[192];
      int off = 0;
      off +=
        snprintf(d + off, sizeof(d) - off, "chip_id=0x%02x,%d", g_bno_chip_id, (int)g_bno_chip_id);
      off += snprintf(d + off, sizeof(d) - off, ",op_mode=0x%02x", g_bno_op_mode);
      off += snprintf(d + off, sizeof(d) - off, ",sys_status=0x%02x", g_bno_sys_status);
      off += snprintf(d + off, sizeof(d) - off, ",sys_err=0x%02x", g_bno_sys_err);
      // CALIB_STAT nibbles: accel[1:0] mag[3:2] gyro[5:4] sys[7:6], each 0-3.
      // 3 = fully calibrated. sys is the minimum of the three; NDOF heading
      // is only trustworthy once mag reaches 3 (figure-of-eight, level).
      off += snprintf(d + off, sizeof(d) - off, ",calib_acc=%d", g_bno_calib_stat & 0x03);
      off += snprintf(d + off, sizeof(d) - off, ",calib_mag=%d", (g_bno_calib_stat >> 2) & 0x03);
      off += snprintf(d + off, sizeof(d) - off, ",calib_gyr=%d", (g_bno_calib_stat >> 4) & 0x03);
      off += snprintf(d + off, sizeof(d) - off, ",calib_sys=%d", (g_bno_calib_stat >> 6) & 0x03);
      off += snprintf(d + off, sizeof(d) - off, ",unit_sel=0x%02x", g_bno_unit_sel);
      off += snprintf(d + off, sizeof(d) - off, ",axis_cfg=0x%02x", g_bno_axis_config);
      off += snprintf(d + off, sizeof(d) - off, ",axis_sign=0x%02x", g_bno_axis_sign);
      if (off >= (int)sizeof(d)) off = sizeof(d) - 1;
      d[off] = '\0';
      Bridge.notify("imu_diag", (const char *)d);
    }
  }

  // LED blink codes (non-blocking)
  blink_update();
}
