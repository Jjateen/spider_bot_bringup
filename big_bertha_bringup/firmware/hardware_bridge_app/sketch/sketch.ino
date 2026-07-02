// sketch.ino  —  Big Bertha hardware bridge (STM32U585)
//
// ── Physical role ──────────────────────────────────────────────────────
// STM32U585 M33 on Arduino UNO Q boards acts as real-time I2C controller,
// offloading servo timing and IMU polling from the A35 Linux side.
// Communication via Arduino Bridge RPC (mailbox-based IPC).
//
// ── Data flow ─────────────────────────────────────────────────────────
//   ROS 2 node (C++) → TCP JSON :50007 → Python relay (main.py)
//     → Bridge RPC → STM32U585 (this sketch) → I2C → PCA9685 + MPU6050
//
// ── Design principles ─────────────────────────────────────────────────
//   - Non-blocking: Bridge RPC handler never waits on I2C. Servo PWM
//     data arrives as a single comma-separated string (fixes 12-arg
//     Bridge RPC limit). The handler stores values and defers the
//     I2C write to loop() via g_pwm_dirty flag.
//   - No verify-on-hot-path: pca9685_verify_init() runs only in the
//     1 Hz status loop, never in the servo write path. Every servo
//     write is one I2C transaction — no re-init delays.
//   - 100 kHz I2C clock: up from 50 kHz (which was below spec).
//     Both PCA9685 and MPU6050 support 400 kHz; 100 kHz is a safe
//     conservative step that halves all blocking times.

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

// Physical PCA9685 channel → logical servo index (Isaac Sim convention)
// [0..2]=FL, [3..5]=FR, [6..8]=HL, [9..11]=HR
static const uint8_t PWM_CHANNEL_MAP[12] = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};

// ── Diagnostic state ──────────────────────────────────────────────────
// bit 0 = PCA9685 missing, bit 1 = MPU6050 missing
static int g_i2c_scan = 0;
static bool g_ai_ok = false;
static bool g_mpu6050_present = false;

// Current PWM off-counts for all 16 channels (0 = output low = servo off)
static uint16_t g_pwm[16] = {0};

// Set true by set_servo_pwms when new PWM data arrives; cleared by loop()
// after writing to the PCA9685. Volatile because handler runs in Bridge RPC
// background thread.
static volatile bool g_pwm_dirty = false;

static unsigned long g_last_imu_push = 0;
static unsigned long g_last_status_push = 0;
static unsigned long g_last_blink = 0;
static int g_blink_code = 0;
static uint32_t g_imu_sample = 0;
static int g_servo_calls = 0;
static int g_ping_count = 0;

// ── Diagnostic counters ───────────────────────────────────────────────
static int g_pwm_write_attempts = 0;   // total write cycles attempted
static int g_pwm_write_fails = 0;      // cycles where at least one channel failed
static int g_pwm_write_oks = 0;        // successful channels in last cycle
static int g_pwm_last_fail_ch = -1;    // last physical channel that failed (-1 = none)
static int g_pwm_last_fail_code = 0;   // endTransmission() return: 0=ok, 2=NACK-addr, 3=NACK-data
static int g_set_servo_last_len = 0;   // data.length() from last set_servo_pwms call
static int g_set_servo_last_idx = 0;   // parsed field count (12 = clean)
static int g_pwm_readback_ch0 = -1;    // PCA9685 ch0 OFF register readback (-1 = read failed)

// 125 Hz IMU (8 ms) — matches expected rate for the 50 Hz policy controller
static const unsigned long IMU_INTERVAL = 8;
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
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(dev, (uint8_t)len) != len) return false;
  for (size_t i = 0; i < len; ++i) {
    buf[i] = Wire.read();
  }
  return true;
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
  for (int i = 0; i < 12; ++i)
    snapshot[i] = g_pwm[PWM_CHANNEL_MAP[i]];

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

// ── MPU6050 IMU ───────────────────────────────────────────────────────

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

// LED blink codes (non-blocking)
//   code 0: solid ON              — everything OK
//   code 1: 100ms period          — PCA9685 missing
//   code 2: 400ms period          — PCA9685 init fail
//   code 3: 2000ms period         — MPU6050 missing
static void blink_update()
{
  unsigned long now = millis();

  // Determine which code should be active
  int code = 0;
  if (g_i2c_scan & 1)       code = 1;
  else if (!g_ai_ok)         code = 2;
  else if (g_i2c_scan & 2)   code = 3;

  // Changed code → reset blink phase
  if (code != g_blink_code) {
    g_blink_code = code;
    g_last_blink = now;
    if (code == 0) { digitalWrite(LED_BUILTIN, HIGH); return; }
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

  // Parse comma-separated PWM values (e.g. "307,153,512,...")
  // Single-arg format avoids the Bridge RPC 12-argument limit.
  g_set_servo_last_len = data.length();
  int vals[12];
  int idx = 0;
  int start = 0;
  int len = data.length();
  for (int i = 0; i <= len && idx < 12; ++i) {
    if (i == len || data.charAt(i) == ',') {
      vals[idx++] = data.substring(start, i).toInt();
      start = i + 1;
    }
  }
  g_set_servo_last_idx = idx;
  if (idx != 12) return;

  for (int i = 0; i < 12; ++i)
    g_pwm[PWM_CHANNEL_MAP[i]] = (uint16_t)vals[i];

  g_pwm_dirty = true;
}

void on_ping()
{
  ++g_ping_count;
  Bridge.notify("pong", g_ping_count);
}

void on_scan_i2c()
{
  std::vector<uint8_t> found;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
      found.push_back(addr);
  }
  Bridge.notify("i2c_scan", found);
}

// ── Servo diagnostic: write test values, read back, report ────────────
// Runs synchronously (diagnostic-only, not on hot path).
void on_servo_diag()
{
  // 1. Write known test PWMs to each of the 12 active channels
  int test_pwms[12];
  for (int i = 0; i < 12; ++i) {
    test_pwms[i] = 100 + i * 200;  // 100, 300, 500, ..., 2300
    uint8_t ch = PWM_CHANNEL_MAP[i];
    g_pwm[ch] = (uint16_t)test_pwms[i];
    uint8_t reg = PCA9685_LED0_ON_L + 4 * ch;
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(reg);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(test_pwms[i] & 0xFF);
    Wire.write((test_pwms[i] >> 8) & 0x0F);
    Wire.endTransmission();
  }

  delay(5);  // settle

  // 2. Read back each channel's OFF register
  int readback[12];
  int pass[12];
  for (int i = 0; i < 12; ++i) {
    uint8_t ch = PWM_CHANNEL_MAP[i];
    uint8_t reg = PCA9685_LED0_ON_L + 4 * ch + 2;  // OFF_L
    uint8_t off_l = 0, off_h = 0;
    bool ok = i2c_read_bytes(PCA9685_ADDR, reg, &off_l, 1)
           && i2c_read_bytes(PCA9685_ADDR, reg + 1, &off_h, 1);
    if (ok) {
      readback[i] = off_l | ((off_h & 0x0F) << 8);
    } else {
      readback[i] = -1;
    }
    pass[i] = (ok && abs(readback[i] - test_pwms[i]) <= 2) ? 1 : 0;
  }

  // 3. Read MODE1 to verify init state
  uint8_t mode1 = 0;
  bool mode1_ok = i2c_read_bytes(PCA9685_ADDR, PCA9685_MODE1, &mode1, 1);
  int mode1_val = mode1_ok ? (int)mode1 : -1;

  // 4. Probe PCA9685 presence
  bool pca_present = (i2c_scan_devices() & 1) == 0;

  // 5. Build report as JSON string packed into one Bridge RPC arg
  String report = "{\"pca_present\":";
  report += pca_present ? "true" : "false";
  report += ",\"mode1\":"; report += mode1_val;
  report += ",\"ai_ok\":"; report += (mode1_ok && (mode1 & 0x20)) ? "true" : "false";
  report += ",\"test_pwms\":[";
  for (int i = 0; i < 12; ++i) {
    if (i > 0) report += ",";
    report += test_pwms[i];
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
}

// ── Setup & Loop ──────────────────────────────────────────────────────

void setup()
{
  Wire.begin();
  Wire.setClock(100000);  // 100 kHz (was 50 kHz — below I2C spec)

  g_i2c_scan = i2c_scan_devices();
  g_mpu6050_present = mpu6050_init();
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
  // Process incoming RPC calls from the Python relay (set_servo_pwms,
  // ping, scan_i2c) regardless of whether the background thread started
  // successfully.  update() is guarded by internal mutexes so redundant
  // calls from the background thread are harmless.
  Bridge.update();

  unsigned long now = millis();

  // ── Deferred servo write ──────────────────────────────────────────
  // Write new PWM values to the PCA9685 if the Python relay sent them.
  // No verify/init check on the hot path — health checks happen at 1 Hz.
  if (g_pwm_dirty) {
    g_pwm_dirty = false;  // optimistic clear — re-set below on failure
    if (!pca9685_write_servos()) {
      g_pwm_dirty = true;  // transaction failed, retry next cycle
    }
  }

  // Push IMU at 125 Hz (only if sensor was detected)
  if (g_mpu6050_present && now - g_last_imu_push >= IMU_INTERVAL) {
    g_last_imu_push = now;
    float ax, ay, az, gx, gy, gz;
    if (mpu6050_read(ax, ay, az, gx, gy, gz)) {
      Bridge.notify("imu", ax, ay, az, gx, gy, gz,
                    (float)g_imu_sample++, (float)now);
    }
  }

  // Push hardware status at 1 Hz
  if (now - g_last_status_push >= STATUS_INTERVAL) {
    g_last_status_push = now;
    g_i2c_scan = i2c_scan_devices();
    g_mpu6050_present = (g_i2c_scan & 2) == 0;
    bool ai = false;
    if (!(g_i2c_scan & 1)) {
      ai = pca9685_verify_init();
      if (!ai) {
        ai = pca9685_init() && pca9685_verify_init();
        if (ai) g_pwm_dirty = true;  // flush RAM state after sleep/wake reset
      }
    }
    g_ai_ok = ai;

    // Read back PCA9685 channel 0 OFF register to verify writes
    uint8_t off_l = 0, off_h = 0;
    if (i2c_read_bytes(PCA9685_ADDR, PCA9685_LED0_ON_L + 2, &off_l, 1) &&
        i2c_read_bytes(PCA9685_ADDR, PCA9685_LED0_ON_L + 3, &off_h, 1)) {
      g_pwm_readback_ch0 = off_l | ((off_h & 0x0F) << 8);
    } else {
      g_pwm_readback_ch0 = -1;
    }

    Bridge.notify("hw_status", g_i2c_scan, ai ? 1 : 0, g_servo_calls, g_ping_count,
                  g_pwm_write_attempts, g_pwm_write_fails, g_pwm_last_fail_ch,
                  g_pwm_last_fail_code, g_set_servo_last_len, g_set_servo_last_idx,
                  g_pwm_readback_ch0);
  }

  // LED blink codes (non-blocking)
  blink_update();
}
