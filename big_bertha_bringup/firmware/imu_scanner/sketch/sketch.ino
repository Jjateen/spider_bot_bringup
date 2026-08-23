// sketch.ino  —  IMU Scanner (standalone diagnostic app for the UNO Q)
//
// Registers a Bridge RPC handler ("read_imu") callable from the Python
// side. On each call it probes the MPU9250 at I2C address 0x68, reads
// the 14-byte register block (ACCEL_XOUT_H..GYRO_ZOUT), converts to SI
// units, and pushes the result back via Bridge.notify("imu_result", ...).
//
// Also performs a full bus scan (addresses 1..126) so the Python side
// can list every I2C device it finds — useful when a sensor is missing
// to distinguish "no power" from "wrong address" from "bus locked".

#include <Arduino_RouterBridge.h>
#include <Wire.h>

#include <vector>

static const uint8_t MPU9250_ADDR = 0x68;
static const uint8_t PCA9685_ADDR = 0x40;

// PCA9685 register map (for diagnostics)
static const uint8_t PCA9685_MODE1 = 0x00;
static const uint8_t PCA9685_LED0_ON_L = 0x06;

// ── I2C helpers ─────────────────────────────────────────────────────────

static bool i2c_probe(uint8_t dev)
{
  Wire.beginTransmission(dev);
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

// ── MPU9250 read ────────────────────────────────────────────────────────

static bool mpu9250_read(float & ax, float & ay, float & az, float & gx, float & gy, float & gz)
{
  if (!i2c_probe(MPU9250_ADDR)) {
    return false;
  }
  uint8_t raw[14] = {0};
  if (!i2c_read_bytes(MPU9250_ADDR, 0x3B, raw, 14)) {
    return false;
  }
  int16_t raw_ax = (raw[0] << 8) | raw[1];
  int16_t raw_ay = (raw[2] << 8) | raw[3];
  int16_t raw_az = (raw[4] << 8) | raw[5];
  int16_t raw_gx = (raw[8] << 8) | raw[9];
  int16_t raw_gy = (raw[10] << 8) | raw[11];
  int16_t raw_gz = (raw[12] << 8) | raw[13];
  ax = raw_ax / 16384.0f * 9.80665f;
  ay = raw_ay / 16384.0f * 9.80665f;
  az = raw_az / 16384.0f * 9.80665f;
  gx = raw_gx / 131.0f * (PI / 180.0f);
  gy = raw_gy / 131.0f * (PI / 180.0f);
  gz = raw_gz / 131.0f * (PI / 180.0f);
  return true;
}

// ── Full I2C scan ───────────────────────────────────────────────────────

static std::vector<uint8_t> i2c_scan_all()
{
  std::vector<uint8_t> found;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found.push_back(addr);
    }
  }
  return found;
}

// ── PCA9685 diagnostics ─────────────────────────────────────────────────

// Check PCA9685 health: probe, read MODE1 and PRE_SCALE, verify AI bit.
// Pushes: (present, mode1, pre_scale, ai_ok)
void on_check_pca9685()
{
  if (!i2c_probe(PCA9685_ADDR)) {
    Bridge.notify("pca9685_result", 0, 0, 0, 0);  // not present
    return;
  }
  uint8_t mode1 = 0, pre_scale = 0;
  i2c_read_bytes(PCA9685_ADDR, PCA9685_MODE1, &mode1, 1);
  i2c_read_bytes(PCA9685_ADDR, 0xFE, &pre_scale, 1);
  bool ai_ok = (mode1 & 0x20) != 0;  // bit 5 = AI
  Bridge.notify("pca9685_result", 1, mode1, pre_scale, ai_ok ? 1 : 0);
}

// Set a single PCA9685 channel to a test PWM value (0..4095).
// Pushes: (ok, ch, pwm)
void on_servo_test(uint8_t ch, uint16_t pwm)
{
  if (!i2c_probe(PCA9685_ADDR)) {
    Bridge.notify("servo_test_result", 0, ch, pwm);
    return;
  }
  pwm = (pwm > 4095) ? 4095 : pwm;
  uint8_t reg = PCA9685_LED0_ON_L + 4 * ch;
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(0);           // ON_L = 0
  Wire.write(0);           // ON_H = 0
  Wire.write(pwm & 0xFF);  // OFF_L
  Wire.write(pwm >> 8);    // OFF_H
  if (Wire.endTransmission() != 0) {
    Bridge.notify("servo_test_result", 0, ch, pwm);
    return;
  }
  Bridge.notify("servo_test_result", 1, ch, pwm);
}

// ── Bridge RPC handlers ─────────────────────────────────────────────────

// Called from Python via Bridge.notify("read_imu").
// Probes the MPU9250, reads it if present, and pushes the result.
void on_read_imu()
{
  float ax, ay, az, gx, gy, gz;
  if (mpu9250_read(ax, ay, az, gx, gy, gz)) {
    Bridge.notify("imu_result", 1, ax, ay, az, gx, gy, gz);
  } else {
    Bridge.notify("imu_result", 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }
}

// Called from Python via Bridge.notify("scan_bus").
// Scans all 126 I2C addresses and pushes the list back.
void on_scan_bus()
{
  auto devices = i2c_scan_all();
  Bridge.notify("bus_scan_result", devices);
}

// ── Setup & Loop ────────────────────────────────────────────────────────

void setup()
{
  Wire.begin();
  Wire.setClock(50000);

  Bridge.begin();

  Bridge.provide("read_imu", on_read_imu);
  Bridge.provide("scan_bus", on_scan_bus);
  Bridge.provide("check_pca9685", on_check_pca9685);
  Bridge.provide("servo_test", on_servo_test);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() { delay(10); }
