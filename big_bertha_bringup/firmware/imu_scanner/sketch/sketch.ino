// sketch.ino  —  IMU Scanner (standalone diagnostic app for the UNO Q)
//
// Registers a Bridge RPC handler ("read_imu") callable from the Python
// side. On each call it probes the BNO055 at I2C address 0x28, reads the
// fused quaternion (0x20) plus raw gyro/accel (0x14/0x08), converts to SI
// units, and pushes the result back via Bridge.notify("imu_result", ...).
//
// Also performs a full bus scan (addresses 1..126) so the Python side
// can list every I2C device it finds — useful when a sensor is missing
// to distinguish "no power" from "wrong address" from "bus locked".

#include <Arduino_RouterBridge.h>
#include <Wire.h>

#include <vector>

static const uint8_t BNO055_ADDR = 0x28;  // BNO055 IMU (0x29 when ADR pin high)
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

static bool i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// ── BNO055 read ─────────────────────────────────────────────────────────

// Puts the chip into NDOF fusion mode (accel+gyro+mag -> absolute
// quaternion) so the diagnostic exercises the same state the production
// hardware_bridge_app uses. Units are set to m/s^2 accel, dps gyro and
// the quaternion is on a 2^14 = 1.0 scale.
static bool bno055_init()
{
  if (!i2c_write_byte(BNO055_ADDR, 0x3D, 0x00)) return false;  // OPR_MODE = CONFIG
  delay(25);
  i2c_write_byte(BNO055_ADDR, 0x3B, 0x00);  // UNIT_SEL: m/s^2, dps, Windows
  i2c_write_byte(BNO055_ADDR, 0x3F, 0x00);  // SYS_TRIGGER: reset clear
  i2c_write_byte(BNO055_ADDR, 0x3E, 0x00);  // PWR_MODE: NORMAL
  delay(20);
  uint8_t chip = 0;
  i2c_read_bytes(BNO055_ADDR, 0x00, &chip, 1);
  if (chip != 0xA0) return false;  // not a BNO055
  i2c_write_byte(BNO055_ADDR, 0x3D, 0x0C);  // OPR_MODE = NDOF
  delay(30);
  return true;
}

static bool bno055_read(
  float & qw, float & qx, float & qy, float & qz,
  float & gx, float & gy, float & gz,
  float & ax, float & ay, float & az)
{
  if (!i2c_probe(BNO055_ADDR)) {
    return false;
  }
  uint8_t quat[8] = {0};  // QUA at 0x20..0x27, 2^14 = 1.0 per unit
  uint8_t gyro[6] = {0};  // GYR at 0x14..0x19, 16 LSB = 1 dps
  uint8_t acc[6] = {0};   // ACC at 0x08..0x0D, 100 LSB = 1 m/s^2
  if (!i2c_read_bytes(BNO055_ADDR, 0x20, quat, 8)) return false;
  if (!i2c_read_bytes(BNO055_ADDR, 0x14, gyro, 6)) return false;
  if (!i2c_read_bytes(BNO055_ADDR, 0x08, acc, 6)) return false;

  qw = ((int16_t)((quat[1] << 8) | quat[0])) / 16384.0f;
  qx = ((int16_t)((quat[3] << 8) | quat[2])) / 16384.0f;
  qy = ((int16_t)((quat[5] << 8) | quat[4])) / 16384.0f;
  qz = ((int16_t)((quat[7] << 8) | quat[6])) / 16384.0f;

  gx = ((int16_t)((gyro[1] << 8) | gyro[0])) / 16.0f * (PI / 180.0f);
  gy = ((int16_t)((gyro[3] << 8) | gyro[2])) / 16.0f * (PI / 180.0f);
  gz = ((int16_t)((gyro[5] << 8) | gyro[4])) / 16.0f * (PI / 180.0f);

  ax = ((int16_t)((acc[1] << 8) | acc[0])) / 100.0f;
  ay = ((int16_t)((acc[3] << 8) | acc[2])) / 100.0f;
  az = ((int16_t)((acc[5] << 8) | acc[4])) / 100.0f;
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
// Probes the BNO055 (initializing fusion on first call), reads it if
// present, and pushes the result.
bool g_imu_init_done = false;

void on_read_imu()
{
  if (!g_imu_init_done) {
    g_imu_init_done = bno055_init();
  }
  float qw, qx, qy, qz, gx, gy, gz, ax, ay, az;
  if (g_imu_init_done && bno055_read(qw, qx, qy, qz, gx, gy, gz, ax, ay, az)) {
    Bridge.notify(
      "imu_result", 1, qw, qx, qy, qz, ax, ay, az, gx, gy, gz);
  } else {
    Bridge.notify("imu_result", 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
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