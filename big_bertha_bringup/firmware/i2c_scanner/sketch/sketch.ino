// sketch.ino  —  I2C Scanner (standalone diagnostic app for the UNO Q)
//
// Scans all three I2C buses (Wire, Wire1, Wire2) and reports which devices
// are found on each. Also provides PCA9685 diagnostics and servo test.
//
// Bus mapping on UNO Q:
//   Wire  = I2C2  → D20/SDA(PB11)  D21/SCL(PB10)  — JANALOG header
//   Wire1 = I2C4  → Qwiic connector (PD12/PD13)
//   Wire2 = I2C3  → A4/SDA(PC1)    A5/SCL(PC0)    — traditional SDA/SCL

#include <Arduino_RouterBridge.h>
#include <Wire.h>

static const uint8_t PCA9685_ADDR = 0x40;
static const uint8_t BNO055_ADDR = 0x28;  // BNO055 IMU (0x29 when ADR pin high)

static const uint8_t PCA9685_MODE1 = 0x00;
static const uint8_t PCA9685_LED0_ON_L = 0x06;

static bool i2c_probe(TwoWire & bus, uint8_t dev)
{
  bus.beginTransmission(dev);
  return bus.endTransmission() == 0;
}

static bool i2c_read_bytes(TwoWire & bus, uint8_t dev, uint8_t reg, uint8_t * buf, size_t len)
{
  bus.beginTransmission(dev);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom(dev, (uint8_t)len) != len) return false;
  for (size_t i = 0; i < len; ++i) {
    buf[i] = bus.read();
  }
  return true;
}

// ── Scan a single bus, sending one Bridge.notify per device found ─────

static void scan_single_bus(int bus_id, TwoWire & bus)
{
  for (uint8_t addr = 1; addr < 127; ++addr) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      Bridge.notify("bus_device", bus_id, addr);
    }
  }
  Bridge.notify("bus_done", bus_id);
}

void on_scan_bus()
{
  scan_single_bus(0, Wire);
  scan_single_bus(1, Wire1);
  scan_single_bus(2, Wire2);
}

// ── PCA9685 diagnostics ─────────────────────────────────────────────────

void on_check_pca9685()
{
  if (!i2c_probe(Wire, PCA9685_ADDR)) {
    Bridge.notify("pca9685_result", 0, 0, 0, 0);
    return;
  }
  uint8_t mode1 = 0, pre_scale = 0;
  i2c_read_bytes(Wire, PCA9685_ADDR, PCA9685_MODE1, &mode1, 1);
  i2c_read_bytes(Wire, PCA9685_ADDR, 0xFE, &pre_scale, 1);
  bool ai_ok = (mode1 & 0x20) != 0;
  Bridge.notify("pca9685_result", 1, mode1, pre_scale, ai_ok ? 1 : 0);
}

void on_servo_test(uint8_t ch, uint16_t pwm)
{
  if (!i2c_probe(Wire, PCA9685_ADDR)) {
    Bridge.notify("servo_test_result", 0, ch, pwm);
    return;
  }
  pwm = (pwm > 4095) ? 4095 : pwm;
  uint8_t reg = PCA9685_LED0_ON_L + 4 * ch;
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(0);
  Wire.write(0);
  Wire.write(pwm & 0xFF);
  Wire.write(pwm >> 8);
  if (Wire.endTransmission() != 0) {
    Bridge.notify("servo_test_result", 0, ch, pwm);
    return;
  }
  Bridge.notify("servo_test_result", 1, ch, pwm);
}

void setup()
{
  Wire.begin();
  Wire.setClock(50000);

  Wire1.begin();
  Wire1.setClock(400000);

  Wire2.begin();
  Wire2.setClock(400000);

  Bridge.begin(460800);

  Bridge.provide("scan_bus", on_scan_bus);
  Bridge.provide("check_pca9685", on_check_pca9685);
  Bridge.provide("servo_test", on_servo_test);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() { delay(10); }
