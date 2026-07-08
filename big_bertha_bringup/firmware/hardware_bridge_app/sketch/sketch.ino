// sketch.ino  —  Big Bertha hardware bridge (STM32U585)
//
// ── Physical role ──────────────────────────────────────────────────────
// STM32U585 M33 on Arduino UNO Q acts as real-time I2C controller,
// offloading servo timing and IMU polling from the A35 Linux side.
// Communication via Arduino Bridge RPC (mailbox-based IPC) through the
// arduino-router.  The ROS 2 hw_bridge node talks MsgPack-RPC directly
// to the router socket — no Python relay, no Docker.
//
// ── Data flow ─────────────────────────────────────────────────────────
//   hw_bridge (C++ rclcpp) ──MsgPack-RPC── arduino-router ──UART── this sketch
//     ↑ /imu                              ↑                        ↑
//     ↓ set_servo_pwms(12 rad) ───────────┴────────────────────────┘
//
// ── Key changes over v1 ───────────────────────────────────────────────
//   - Per-joint calibration table (min, max, offset, direction, channel)
//     instead of global pwm_min/max and hardcoded channel map.
//   - set_servo_pwms accepts 12 raw radians; firmware applies calibration.
//   - Watchdog: ramps to safe crouch if no command in ~150 ms.
//   - IMU pushed at 200 Hz (5 ms) with sample counter + micros() timestamp.
//   - No polling calls from host; everything is notify/provide.

#include <Arduino_RouterBridge.h>
#include <Wire.h>

#include <vector>

// ── I2C device addresses ──────────────────────────────────────────────
static const uint8_t MPU6050_ADDR = 0x68;
static const uint8_t PCA9685_ADDR = 0x40;

// ── PCA9685 register map ──────────────────────────────────────────────
static const uint8_t PCA9685_MODE1 = 0x00;
static const uint8_t PCA9685_PRE_SCALE = 0xFE;
static const uint8_t PCA9685_LED0_ON_L = 0x06;

// ── PCA9685 global PWM range ──────────────────────────────────────────
// MG995 servos: 102 (~0.5 ms) to 512 (~2.5 ms) at 50 Hz / 12-bit.
static const uint16_t PWM_MIN = 102;
static const uint16_t PWM_MAX = 512;

// ── Per-joint calibration ─────────────────────────────────────────────
// Each joint has: channel (PCA9685), lower/upper angle (deg), mounting
// offset (deg), direction (+1/-1), and policy center (rad).
struct JointCal {
  uint8_t channel;
  float lower_deg;
  float upper_deg;
  float offset_deg;
  int8_t direction;
  float policy_center_rad;
};

// Values from the hardware_bridge.yaml calibration tables.
// [0..2]=FL hip/upper/lower, [3..5]=FR, [6..8]=HL, [9..11]=HR
static const JointCal CAL[12] = {
  {14,  45.0, 180.0,  0.0,  1, 0.0},      //  0: FL hip
  {10,  30.0, 150.0,  0.0,  1, 0.0},      //  1: FL upper
  { 2, 180.0,  50.0,  0.0,  1, 0.0},      //  2: FL lower
  { 6, 140.0,   0.0,  0.0,  1, 1.57},     //  3: FR hip
  {13, 135.0,   0.0,  0.0,  1, 1.57},     //  4: FR upper
  { 9, 140.0,   0.0,  0.0,  1, 1.57},     //  5: FR lower
  { 1,  50.0, 180.0, 10.0, -1, 0.0},      //  6: HL hip
  { 5,  50.0, 180.0, 10.0, -1, 0.0},      //  7: HL upper
  {12,  40.0, 180.0,  0.0, -1, 1.57},     //  8: HL lower
  { 8, 180.0,  40.0,  8.0, -1, 1.57},     //  9: HR hip
  { 0, 150.0,   0.0,  2.0,  1, 1.57},     // 10: HR upper
  { 4,   0.0, 150.0,  5.0,  1, 1.57},     // 11: HR lower
};

// ── Safe crouch position ─────────────────────────────────────────────
// PWM values the watchdog ramps to when no command arrives in time.
// A low, stable stance that won't tip the robot.
static const uint16_t CROUCH_PWM[12] = {
  307, 307, 307,   // FL
  307, 307, 307,   // FR
  307, 307, 307,   // HL
  307, 307, 307,   // HR
};

// ── Diagnostic state ──────────────────────────────────────────────────
static int g_i2c_scan = 0;
static bool g_ai_ok = false;
static bool g_mpu6050_present = false;

// Current PWM off-counts for all 16 channels
static uint16_t g_pwm[16] = {0};
static volatile bool g_pwm_dirty = false;

// Watchdog
static volatile unsigned long g_last_command_ms = 0;
static const unsigned long WATCHDOG_MS = 150;
static bool g_watchdog_active = false;
static float g_watchdog_t = 1.0f;  // 0..1 ramp progress, 1 = fully crouched

// Timers
static unsigned long g_last_imu_push = 0;
static unsigned long g_last_status_push = 0;
static unsigned long g_last_blink = 0;
static int g_blink_code = 0;
static uint32_t g_imu_sample = 0;
static int g_servo_calls = 0;
static int g_ping_count = 0;

// Diagnostic counters
static int g_pwm_write_attempts = 0;
static int g_pwm_write_fails = 0;
static int g_pwm_write_oks = 0;
static int g_pwm_last_fail_ch = -1;
static int g_pwm_last_fail_code = 0;
static int g_set_servo_last_len = 0;
static int g_set_servo_last_idx = 0;
static int g_pwm_readback_ch0 = -1;
static volatile bool g_diag_pending = false;
static int g_diag_test_pwms[12];
static unsigned long g_diag_settle_start = 0;
static int g_diag_phase = 0;
static unsigned long g_diag_start_ms = 0;
static int g_i2c_consecutive_fails = 0;
static bool g_i2c_busy = false;

// 200 Hz IMU (5 ms), 1 Hz status
static const unsigned long IMU_INTERVAL = 5;
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
    if (Wire.endTransmission() != 0) { delay(1); continue; }
    if (Wire.requestFrom(dev, (uint8_t)len) != len) { delay(1); continue; }
    for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
    ok = true;
    break;
  }
  g_i2c_busy = false;
  if (ok) { g_i2c_consecutive_fails = 0; }
  else { ++g_i2c_consecutive_fails; }
  return ok;
}

// ── PCA9685 ───────────────────────────────────────────────────────────
static bool pca9685_init()
{
  if (!i2c_write_byte(PCA9685_ADDR, PCA9685_MODE1, 0x10)) return false;
  delay(2);
  if (!i2c_write_byte(PCA9685_ADDR, PCA9685_PRE_SCALE, 121)) return false;
  delay(1);
  if (!i2c_write_byte(PCA9685_ADDR, PCA9685_MODE1, 0x20)) return false;
  delay(2);
  return true;
}

static bool pca9685_write_servos()
{
  uint16_t snapshot[12];
  for (int i = 0; i < 12; ++i)
    snapshot[i] = g_pwm[CAL[i].channel];

  ++g_pwm_write_attempts;
  g_pwm_write_oks = 0;
  g_pwm_last_fail_ch = -1;
  g_pwm_last_fail_code = 0;
  for (int i = 0; i < 12; ++i) {
    uint8_t ch = CAL[i].channel;
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

// ── MPU6050 ───────────────────────────────────────────────────────────
static bool mpu6050_init()
{
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) return false;
  i2c_write_byte(MPU6050_ADDR, 0x6B, 0x00);
  delay(100);
  return true;
}

static bool mpu6050_read(
  float & ax, float & ay, float & az,
  float & gx, float & gy, float & gz)
{
  uint8_t raw[14] = {0};
  if (!i2c_read_bytes(MPU6050_ADDR, 0x3B, raw, 14)) {
    ax = ay = az = gx = gy = gz = 0.0f;
    return false;
  }
  int16_t rax = (raw[0] << 8) | raw[1];
  int16_t ray = (raw[2] << 8) | raw[3];
  int16_t raz = (raw[4] << 8) | raw[5];
  int16_t rgx = (raw[8] << 8) | raw[9];
  int16_t rgy = (raw[10] << 8) | raw[11];
  int16_t rgz = (raw[12] << 8) | raw[13];
  ax = rax / 16384.0f * 9.80665f;
  ay = ray / 16384.0f * 9.80665f;
  az = raz / 16384.0f * 9.80665f;
  gx = rgx / 131.0f * (PI / 180.0f);
  gy = rgy / 131.0f * (PI / 180.0f);
  gz = rgz / 131.0f * (PI / 180.0f);
  return true;
}

// ── Diagnostics ───────────────────────────────────────────────────────
static int i2c_scan_devices()
{
  int missing = 0;
  Wire.beginTransmission(PCA9685_ADDR);
  if (Wire.endTransmission() != 0) missing |= 1;
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) missing |= 2;
  return missing;
}

static void blink_update()
{
  unsigned long now = millis();
  int code = 0;
  if (g_i2c_scan & 1)       code = 1;
  else if (!g_ai_ok)         code = 2;
  else if (g_i2c_scan & 2)   code = 3;

  if (code != g_blink_code) {
    g_blink_code = code;
    g_last_blink = now;
    if (code == 0) { digitalWrite(LED_BUILTIN, HIGH); return; }
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }
  if (code == 0) return;
  unsigned long period = (code == 1) ? 100UL : (code == 2) ? 400UL : 2000UL;
  unsigned long half = period / 2;
  bool on = (now - g_last_blink) < half;
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
  if (now - g_last_blink >= period) g_last_blink = now;
}

// ── Radians → PWM (per-joint calibration) ────────────────────────────
// Converts a raw joint angle (radians from the policy) to a 12-bit PWM
// value using the per-joint calibration table.
static uint16_t rad_to_pwm(int joint, float rad)
{
  if (joint < 0 || joint >= 12) return 0;

  const JointCal & jc = CAL[joint];

  // 1. Clamp to joint limit (hardware-safe absolute limit)
  if (rad < -PI) rad = -PI;
  if (rad >  PI) rad =  PI;

  // 2. Radians → degrees
  float deg = rad * 180.0f / PI;

  // 3. Remove policy center, apply direction, add mounting offset
  float center_deg = jc.policy_center_rad * 180.0f / PI;
  deg = (deg - center_deg) * jc.direction;
  deg = deg + jc.offset_deg;

  // 4. Servo frame: 0° is fully CCW, 90° is center, 180° is fully CW.
  //    The policy outputs angle-relative-to-center, so add 90° bias.
  deg = deg + 90.0f;

  // 5. Clamp to mechanical limits
  float lo = (jc.lower_deg < jc.upper_deg) ? jc.lower_deg : jc.upper_deg;
  float hi = (jc.lower_deg > jc.upper_deg) ? jc.lower_deg : jc.upper_deg;
  if (deg < lo) deg = lo;
  if (deg > hi) deg = hi;

  // 6. Map 0-180° → PWM_MIN-PWM_MAX
  float t = deg / 180.0f;
  float pwm = t * (PWM_MAX - PWM_MIN) + PWM_MIN;
  if (pwm < 0.0f) pwm = 0.0f;
  if (pwm > 4095.0f) pwm = 4095.0f;
  return (uint16_t)(pwm + 0.5f);
}

// ── Bridge RPC handlers ───────────────────────────────────────────────

void set_servo_pwms(String data)
{
  ++g_servo_calls;

  // Parse 12 comma-separated float radian values.
  // e.g. "0.00,0.05,-0.02,..."
  g_set_servo_last_len = data.length();
  float radians[12];
  int idx = 0;
  int start = 0;
  int len = data.length();
  for (int i = 0; i <= len && idx < 12; ++i) {
    if (i == len || data.charAt(i) == ',') {
      radians[idx++] = data.substring(start, i).toFloat();
      start = i + 1;
    }
  }
  g_set_servo_last_idx = idx;
  if (idx != 12) return;

  // Apply per-joint calibration
  for (int i = 0; i < 12; ++i) {
    g_pwm[CAL[i].channel] = rad_to_pwm(i, radians[i]);
  }

  g_pwm_dirty = true;
  g_last_command_ms = millis();
  g_watchdog_t = 0.0f;   // reset watchdog ramp
  g_watchdog_active = false;
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

void on_servo_diag()
{
  for (int i = 0; i < 12; ++i)
    g_diag_test_pwms[i] = 100 + i * 200;
  g_diag_pending = true;
}

// ── Watchdog ──────────────────────────────────────────────────────────
// If no set_servo_pwms received for WATCHDOG_MS, ramp all servos to
// the CROUCH_PWM position over ~50 ms (5 steps).
static void watchdog_update()
{
  unsigned long now_ms = millis();
  unsigned long elapsed = now_ms - g_last_command_ms;

  if (elapsed < WATCHDOG_MS) return;

  if (!g_watchdog_active) {
    // First trigger: save current positions and start ramp
    g_watchdog_active = true;
    g_watchdog_t = 0.0f;
  }

  // Advance ramp: reach full crouch in ~50 ms (~10 loop iterations at 5 ms)
  g_watchdog_t += 0.2f;
  if (g_watchdog_t > 1.0f) g_watchdog_t = 1.0f;

  // Interpolate between current PWM and crouch
  for (int i = 0; i < 12; ++i) {
    uint8_t ch = CAL[i].channel;
    float current = g_pwm[ch];
    float target = CROUCH_PWM[i];
    float stepped = current + (target - current) * g_watchdog_t;
    g_pwm[ch] = (uint16_t)(stepped + 0.5f);
  }
  g_pwm_dirty = true;
}

// ── Setup & Loop ──────────────────────────────────────────────────────

void setup()
{
  Wire.begin();
  Wire.setClock(50000);

  g_i2c_scan = i2c_scan_devices();
  g_mpu6050_present = mpu6050_init();
  g_ai_ok = pca9685_init() && pca9685_verify_init();

  Bridge.begin(460800);

  Bridge.provide("set_servo_pwms", set_servo_pwms);
  Bridge.provide("scan_i2c", on_scan_i2c);
  Bridge.provide("ping", on_ping);
  Bridge.provide("servo_diag", on_servo_diag);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
  Bridge.update();
  unsigned long now = millis();

  // ── Deferred servo write ──────────────────────────────────────────
  if (g_pwm_dirty) {
    g_pwm_dirty = false;
    if (!pca9685_write_servos()) {
      g_pwm_dirty = true;
    }
  }

  // ── Watchdog ──────────────────────────────────────────────────────
  watchdog_update();

  // ── Deferred servo diagnostic ──────────────────────────────────────
  if (g_diag_pending) {
    if (g_diag_phase == 0) {
      for (int i = 0; i < 12; ++i) {
        uint8_t ch = CAL[i].channel;
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
      if (millis() - g_diag_start_ms > 300) {
        Bridge.notify("servo_diag_result", 0, 0, -1, 0, 0);
        g_diag_pending = false;
        g_diag_phase = 0;
      } else if (millis() - g_diag_settle_start >= 5) {
        int readback[12];
        int pass[12];
        for (int i = 0; i < 12; ++i) {
          uint8_t ch = CAL[i].channel;
          uint8_t reg = PCA9685_LED0_ON_L + 4 * ch + 2;
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
        report += ",\"mode1\":"; report += mode1_val;
        report += ",\"ai_ok\":"; report += (mode1_ok && (mode1 & 0x20)) ? "true" : "false";
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

  // ── Push IMU at 200 Hz ────────────────────────────────────────────
  if (g_mpu6050_present && now - g_last_imu_push >= IMU_INTERVAL) {
    g_last_imu_push = now;
    float ax, ay, az, gx, gy, gz;
    if (mpu6050_read(ax, ay, az, gx, gy, gz)) {
      Bridge.notify("imu", ax, ay, az, gx, gy, gz,
                    (float)g_imu_sample++, (float)micros());
    }
  }

  // ── Push hardware status at 1 Hz ──────────────────────────────────
  if (now - g_last_status_push >= STATUS_INTERVAL) {
    g_last_status_push = now;
    g_i2c_scan = i2c_scan_devices();
    g_mpu6050_present = (g_i2c_scan & 2) == 0;
    bool ai = false;
    if (!(g_i2c_scan & 1)) {
      ai = pca9685_verify_init();
      if (!ai) {
        ai = pca9685_init() && pca9685_verify_init();
        if (ai) g_pwm_dirty = true;
      }
    }
    g_ai_ok = ai;

    uint8_t off_l = 0, off_h = 0;
    if (i2c_read_bytes(PCA9685_ADDR, PCA9685_LED0_ON_L + 2, &off_l, 1) &&
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

    Bridge.notify("hw_status", g_i2c_scan, ai ? 1 : 0, g_servo_calls, g_ping_count,
                  g_pwm_write_attempts, g_pwm_write_fails, g_pwm_last_fail_ch,
                  g_pwm_last_fail_code, g_set_servo_last_len, g_set_servo_last_idx,
                  g_pwm_readback_ch0);
  }

  blink_update();
}
