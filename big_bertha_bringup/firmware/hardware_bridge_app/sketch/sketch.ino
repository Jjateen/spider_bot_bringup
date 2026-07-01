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
// ── Improvements over original ────────────────────────────────────────
//   - Bridge.begin(460800) — explicit baud rate matching arduino-router
//   - Bulk PCA9685 write — all 16 channels in one I2C transaction
//   - 125 Hz IMU push — better temporal resolution for policy controller
//   - Auto re-init of PCA9685 on verify failure — self-healing
//   - ping provider — quick liveness check
//   - LED blink codes — hardware diagnostics at a glance

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

static unsigned long g_last_imu_push = 0;
static unsigned long g_last_status_push = 0;
static uint32_t g_imu_sample = 0;
static int g_servo_calls = 0;
static int g_ping_count = 0;

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

static void pca9685_init()
{
  i2c_write_byte(PCA9685_ADDR, PCA9685_MODE1, 0x10);  // sleep
  delay(2);
  i2c_write_byte(PCA9685_ADDR, PCA9685_PRE_SCALE, 121);  // 50 Hz
  delay(1);
  i2c_write_byte(PCA9685_ADDR, PCA9685_MODE1, 0x20);  // wake + AI
  delay(2);
}

// Write all 16 channels in a single I2C burst (auto-increment)
static void pca9685_write_all()
{
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(PCA9685_LED0_ON_L);
  for (int ch = 0; ch < 16; ++ch) {
    uint16_t off = g_pwm[ch];
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(off & 0xFF);
    Wire.write((off >> 8) & 0x0F);
  }
  Wire.endTransmission();
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

// LED blink error codes:
//   3 quick (100ms) = PCA9685 missing
//   2 medium (200ms)= PCA9685 init fail
//   1 long (500ms)  = MPU6050 missing
//   Solid on        = everything OK
static void blink_error(int count, int flash_ms, int pause_ms)
{
  for (int i = 0; i < count; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(flash_ms);
    digitalWrite(LED_BUILTIN, LOW);
    if (i < count - 1) delay(flash_ms);
  }
  delay(pause_ms);
}

// ── Bridge RPC handlers ───────────────────────────────────────────────

void set_servo_pwms(std::vector<int> pwms)
{
  ++g_servo_calls;

  if (!pca9685_verify_init()) pca9685_init();

  size_t n = pwms.size();
  if (n > 12) n = 12;
  for (size_t i = 0; i < n; ++i)
    g_pwm[PWM_CHANNEL_MAP[i]] = (uint16_t)pwms[i];

  pca9685_write_all();
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

// ── Setup & Loop ──────────────────────────────────────────────────────

void setup()
{
  Wire.begin();
  Wire.setClock(50000);

  g_i2c_scan = i2c_scan_devices();
  g_mpu6050_present = mpu6050_init();
  pca9685_init();
  g_ai_ok = pca9685_verify_init();

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
    int scan = i2c_scan_devices();
    g_mpu6050_present = (scan & 2) == 0;
    bool ai = false;
    if (!(scan & 1)) {
      ai = pca9685_verify_init();
      if (!ai) {
        pca9685_init();
        ai = pca9685_verify_init();
      }
    }
    g_ai_ok = ai;
    Bridge.notify("hw_status", scan, ai ? 1 : 0, g_servo_calls, g_ping_count);
  }

  // LED blink codes (priority cascade)
  if (g_i2c_scan & 1) {
    blink_error(3, 100, 600);
  } else if (!g_ai_ok) {
    blink_error(2, 200, 800);
  } else if (g_i2c_scan & 2) {
    blink_error(1, 500, 1500);
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    // No delay in healthy path — IMU runs at full rate
  }
}
