// sketch.ino  —  Big Bertha hardware bridge (STM32U585)
//
// Uses Arduino RouterBridge RPC so the MPU can call set_servo_pwms and
// get_imu_data over the internal UART without conflicting with the bridge.
//
// Providers (callable from Python via Bridge.call/notify):
//   set_servo_pwms(vector<uint16_t> pwms)  — 12 PWM values
//   get_imu_data() → vector<float>         — [ax, ay, az, gx, gy, gz]

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <vector>

// ── I2C addresses ──────────────────────────────────────────────────────
static const uint8_t MPU6050_ADDR = 0x68;
static const uint8_t PCA9685_ADDR = 0x40;

// PCA9685 registers
static const uint8_t PCA9685_MODE1     = 0x00;
static const uint8_t PCA9685_PRE_SCALE = 0xFE;
static const uint8_t PCA9685_LED0_ON_L = 0x06;

// MPU6050 registers
static const uint8_t MPU6050_PWR_MGMT_1   = 0x6B;
static const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;

// ── I2C helpers ────────────────────────────────────────────────────────
static void i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static void i2c_read_bytes(uint8_t dev, uint8_t reg, uint8_t * buf, size_t len)
{
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(dev, (uint8_t)len);
  for (size_t i = 0; i < len && Wire.available(); ++i) {
    buf[i] = Wire.read();
  }
}

// ── PCA9685 driver ─────────────────────────────────────────────────────
static void pca9685_init()
{
  i2c_write_byte(PCA9685_ADDR, PCA9685_MODE1, 0x10);
  delay(1);
  i2c_write_byte(PCA9685_ADDR, PCA9685_PRE_SCALE, 121);  // 50 Hz
  i2c_write_byte(PCA9685_ADDR, PCA9685_MODE1, 0x80);
  delay(1);
}

static void pca9685_set_pwm(uint8_t ch, uint16_t off)
{
  uint8_t reg = PCA9685_LED0_ON_L + 4 * ch;
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(0);
  Wire.write(0);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  Wire.endTransmission();
}

// ── MPU6050 driver ─────────────────────────────────────────────────────
static void mpu6050_init()
{
  i2c_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
  delay(100);
}

static void mpu6050_read(
  float & ax, float & ay, float & az,
  float & gx, float & gy, float & gz)
{
  uint8_t raw[14];
  i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, raw, 14);

  int16_t raw_ax = (raw[0]  << 8) | raw[1];
  int16_t raw_ay = (raw[2]  << 8) | raw[3];
  int16_t raw_az = (raw[4]  << 8) | raw[5];
  int16_t raw_gx = (raw[8]  << 8) | raw[9];
  int16_t raw_gy = (raw[10] << 8) | raw[11];
  int16_t raw_gz = (raw[12] << 8) | raw[13];

  ax = raw_ax / 16384.0f * 9.80665f;
  ay = raw_ay / 16384.0f * 9.80665f;
  az = raw_az / 16384.0f * 9.80665f;
  gx = raw_gx / 131.0f * (PI / 180.0f);
  gy = raw_gy / 131.0f * (PI / 180.0f);
  gz = raw_gz / 131.0f * (PI / 180.0f);
}

// ── Bridge RPC providers ───────────────────────────────────────────────

// set_servo_pwms(pwms)  —  called via Bridge.notify() from Python
void set_servo_pwms(std::vector<uint16_t> pwms)
{
  size_t n = pwms.size();
  if (n > 12) n = 12;
  for (size_t i = 0; i < n; ++i) {
    pca9685_set_pwm((uint8_t)i, pwms[i]);
  }
}

// get_imu_data() → [ax, ay, az, gx, gy, gz]  —  called via Bridge.call()
std::vector<float> get_imu_data()
{
  float ax, ay, az, gx, gy, gz;
  mpu6050_read(ax, ay, az, gx, gy, gz);
  return {ax, ay, az, gx, gy, gz};
}

// ── Arduino lifecycle ──────────────────────────────────────────────────
void setup()
{
  Wire.begin();
  Wire.setClock(400000);

  mpu6050_init();
  pca9685_init();

  Bridge.begin();
  Bridge.provide("set_servo_pwms", set_servo_pwms);
  Bridge.provide("get_imu_data", get_imu_data);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
  delay(10);
}
