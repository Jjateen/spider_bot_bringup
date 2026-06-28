#include <Arduino.h>
#line 1 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
// sketch.ino  -  Big Bertha hardware bridge (STM32U585)
// IMU_INTERVAL=8 (125Hz), Bridge.begin(460800), no blocking delays in loop().
// PCA9685: bulk write all 16 channels in one I2C transaction (auto-increment).
// verify_init checks AI=1 AND SLEEP=0.

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <vector>

static const uint8_t MPU6050_ADDR      = 0x68;
static const uint8_t PCA9685_ADDR      = 0x40;
static const uint8_t PCA9685_MODE1     = 0x00;
static const uint8_t PCA9685_PRE_SCALE = 0xFE;
static const uint8_t PCA9685_LED0_ON_L = 0x06;
// channel index -> PCA9685 physical channel (skips 3,7,11,15)
static const uint8_t PWM_CHANNEL_MAP[12] = {0,1,2,4,5,6,8,9,10,12,13,14};

static int  g_i2c_scan    = 0;
static bool g_ai_ok       = false;
static int  g_servo_calls = 0;

// current PWM off-counts for all 16 channels (0 = output stays low = servo off)
static uint16_t g_pwm[16] = {0};

static unsigned long g_last_imu_push    = 0;
static unsigned long g_last_status_push = 0;
static uint32_t      g_imu_sample       = 0;
static const unsigned long IMU_INTERVAL    = 8;
static const unsigned long STATUS_INTERVAL = 1000;

// ── I2C helpers ──────────────────────────────────────────────────────────────

#line 33 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
static void i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t val);
#line 41 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
static void i2c_read_bytes(uint8_t dev, uint8_t reg, uint8_t *buf, size_t len);
#line 53 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
static void pca9685_init();
#line 68 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
static void pca9685_write_all();
#line 83 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
static bool pca9685_verify_init();
#line 93 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
static void mpu6050_init();
#line 99 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
static void mpu6050_read(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
#line 120 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
static int i2c_scan_devices();
#line 132 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
void set_servo_pwms(std::vector<int> pwms);
#line 149 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
void on_ping();
#line 155 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
void on_scan_i2c();
#line 168 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
void setup();
#line 187 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
void loop();
#line 33 "/home/arduino/ArduinoApps/hardware_bridge_app/sketch/sketch.ino"
static void i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static void i2c_read_bytes(uint8_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(dev, (uint8_t)len);
    for (size_t i = 0; i < len && Wire.available(); ++i)
        buf[i] = Wire.read();
}

// ── PCA9685 ──────────────────────────────────────────────────────────────────

static void pca9685_init()
{
    // 1. Put to sleep so we can write prescaler
    i2c_write_byte(PCA9685_ADDR, PCA9685_MODE1, 0x10); // SLEEP=1, AI=0
    delay(2);
    // 2. Set prescaler for 50 Hz  (25MHz / (4096 * 50Hz) - 1 = 121)
    i2c_write_byte(PCA9685_ADDR, PCA9685_PRE_SCALE, 121);
    delay(1);
    // 3. Wake up with auto-increment enabled (AI=1, SLEEP=0)
    i2c_write_byte(PCA9685_ADDR, PCA9685_MODE1, 0x20);
    delay(2); // oscillator needs >=500us to stabilise
}

// Write all 16 PCA9685 channels in a single I2C burst (auto-increment).
// Channels not in the servo map keep their last value (g_pwm).
static void pca9685_write_all()
{
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(PCA9685_LED0_ON_L); // start at channel 0, ON_L
    for (int ch = 0; ch < 16; ++ch) {
        uint16_t off = g_pwm[ch];
        Wire.write(0x00);           // ON_L  = 0 (no delay-on)
        Wire.write(0x00);           // ON_H  = 0
        Wire.write(off & 0xFF);     // OFF_L
        Wire.write((off >> 8) & 0x0F); // OFF_H (only lower 4 bits used)
    }
    Wire.endTransmission();
}

// Check MODE1: AI bit must be set AND SLEEP bit must be clear
static bool pca9685_verify_init()
{
    uint8_t mode1 = 0;
    i2c_read_bytes(PCA9685_ADDR, PCA9685_MODE1, &mode1, 1);
    return (mode1 & 0x20) != 0   // AI=1 (auto-increment enabled)
        && (mode1 & 0x10) == 0;  // SLEEP=0 (oscillator running)
}

// ── MPU-6050 ─────────────────────────────────────────────────────────────────

static void mpu6050_init()
{
    i2c_write_byte(MPU6050_ADDR, 0x6B, 0x00); // wake up, internal clock
    delay(100);
}

static void mpu6050_read(float &ax, float &ay, float &az,
                          float &gx, float &gy, float &gz)
{
    uint8_t raw[14] = {};
    i2c_read_bytes(MPU6050_ADDR, 0x3B, raw, 14);
    int16_t rax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ray = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t raz = (int16_t)((raw[4]  << 8) | raw[5]);
    int16_t rgx = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t rgy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t rgz = (int16_t)((raw[12] << 8) | raw[13]);
    ax = rax / 16384.0f * 9.80665f;
    ay = ray / 16384.0f * 9.80665f;
    az = raz / 16384.0f * 9.80665f;
    gx = rgx / 131.0f * (3.14159265f / 180.0f);
    gy = rgy / 131.0f * (3.14159265f / 180.0f);
    gz = rgz / 131.0f * (3.14159265f / 180.0f);
}

// ── I2C scan ─────────────────────────────────────────────────────────────────

static int i2c_scan_devices()
{
    int missing = 0;
    Wire.beginTransmission(PCA9685_ADDR);
    if (Wire.endTransmission() != 0) missing |= 1;
    Wire.beginTransmission(MPU6050_ADDR);
    if (Wire.endTransmission() != 0) missing |= 2;
    return missing;
}

// ── Bridge handlers ──────────────────────────────────────────────────────────

void set_servo_pwms(std::vector<int> pwms)
{
    ++g_servo_calls;

    if (!pca9685_verify_init()) pca9685_init();

    // Update g_pwm for the mapped channels, leave others unchanged
    size_t n = pwms.size();
    if (n > 12) n = 12;
    for (size_t i = 0; i < n; ++i)
        g_pwm[PWM_CHANNEL_MAP[i]] = (uint16_t)pwms[i];

    // Write all 16 channels in one burst
    pca9685_write_all();
}

int g_ping_count = 0;
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

// ── Arduino entry points ─────────────────────────────────────────────────────

void setup()
{
    Wire.begin();
    Wire.setClock(400000);

    g_i2c_scan = i2c_scan_devices();
    mpu6050_init();
    pca9685_init();
    g_ai_ok = pca9685_verify_init();

    Bridge.begin(460800);
    Bridge.provide("set_servo_pwms", set_servo_pwms);
    Bridge.provide("scan_i2c", on_scan_i2c);
    Bridge.provide("ping", on_ping);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
    unsigned long now = millis();

    if (now - g_last_imu_push >= IMU_INTERVAL) {
        g_last_imu_push = now;
        float ax, ay, az, gx, gy, gz;
        mpu6050_read(ax, ay, az, gx, gy, gz);
        Bridge.notify("imu", ax, ay, az, gx, gy, gz,
                      (float)g_imu_sample++, (float)now);
    }

    if (now - g_last_status_push >= STATUS_INTERVAL) {
        g_last_status_push = now;
        int scan = i2c_scan_devices();
        bool ai = (!(scan & 1)) ? pca9685_verify_init() : false;
        Bridge.notify("hw_status", scan, ai ? 1 : 0, g_servo_calls, g_ping_count);
    }
}

