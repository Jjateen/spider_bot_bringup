// sketch.ino  —  Big Bertha hardware bridge (STM32U585)
//
// ── Physical role ──────────────────────────────────────────────────────
// STM32U585 M33 on Arduino UNO Q boards acts as real-time I2C controller,
// offloading servo timing and IMU polling from the A35 Linux side.
// Communication via Arduino Bridge RPC (mailbox-based IPC).
//
// ── Data flow ─────────────────────────────────────────────────────────
//   ROS 2 node (C++) → TCP JSON :50007 → Python relay (main.py)
//     → Bridge RPC → STM32U585 (this sketch) → I2C → PCA9685 + MPU9250
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
static const uint8_t MPU9250_ADDR = 0x68;
static const uint8_t AK8963_ADDR = 0x0C;   // magnetometer inside the MPU9250
static const uint8_t PCA9685_ADDR = 0x40;

// ── PCA9685 register map ──────────────────────────────────────────────
static const uint8_t PCA9685_MODE1 = 0x00;
static const uint8_t PCA9685_PRE_SCALE = 0xFE;
static const uint8_t PCA9685_LED0_ON_L = 0x06;

// Physical PCA9685 channel → logical servo index (Isaac Sim convention)
// [0..2]=FL, [3..5]=FR, [6..8]=HL, [9..11]=HR
static const uint8_t PWM_CHANNEL_MAP[12] = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};

// ── Diagnostic state ──────────────────────────────────────────────────
// bit 0 = PCA9685 missing, bit 1 = MPU9250 missing
static int g_i2c_scan = 0;
static bool g_ai_ok = false;
static bool g_mpu9250_present = false;
static bool g_mag_present = false;

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
static volatile bool g_diag_pending = false;
static int g_diag_test_pwms[12];
static unsigned long g_diag_settle_start = 0;
static int g_diag_phase = 0;
static unsigned long g_diag_start_ms = 0;    // when phase 1 started, for timeout guard

// I2C error tracking for self-healing
static int g_i2c_consecutive_fails = 0;       // resets on any success
static bool g_i2c_busy = false;               // guard for BG-thread vs loop() race

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
  g_i2c_busy = true;
  bool ok = false;
  for (int attempt = 0; attempt < 3; ++attempt) {
    Wire.beginTransmission(dev);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) { delay(1); continue; }
    if (Wire.requestFrom(dev, (uint8_t)len) != len) { delay(1); continue; }
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

// ── MPU9250 IMU ───────────────────────────────────────────────────────

// WHO_AM_I (0x75) read at init: 0x71 = MPU-9250, 0x73 = MPU-9255,
// 0x70 = MPU-6500 (six axis, no magnetometer die at all). Reported in
// hw_status so the part can be identified without a scope or a teardown.
static uint8_t g_mpu_whoami = 0x00;

static bool mpu9250_init()
{
  Wire.beginTransmission(MPU9250_ADDR);
  if (Wire.endTransmission() != 0) return false;

  // Full device reset first. The MPU keeps its register state for as long as
  // it stays powered, and reflashing the MCU does not power-cycle it, so
  // without this the chip can still be carrying configuration from whatever
  // ran before -- including I2C_MST_EN, which hides the magnetometer.
  i2c_write_byte(MPU9250_ADDR, 0x6B, 0x80);
  delay(100);
  i2c_write_byte(MPU9250_ADDR, 0x6B, 0x00);   // wake, internal oscillator
  delay(100);

  i2c_read_bytes(MPU9250_ADDR, 0x75, &g_mpu_whoami, 1);

  // USER_CTRL bit 5 I2C_MST_EN must be 0 or "pins ES_DA and ES_SCL are
  // isolated from pins SDA/SDI and SCL/SCLK" (RM-000008 §4.33) -- the
  // auxiliary bus carrying the magnetometer is cut off from the host, and
  // BYPASS_EN is documented to work only "when the i2c master interface is
  // disabled". Reset clears it; cleared explicitly so the requirement is
  // visible rather than implied.
  uint8_t user_ctrl = 0;
  i2c_read_bytes(MPU9250_ADDR, 0x6A, &user_ctrl, 1);
  i2c_write_byte(MPU9250_ADDR, 0x6A, user_ctrl & ~0x20);
  delay(10);
  return true;
}

static bool mpu9250_read(
  float & ax, float & ay, float & az,
  float & gx, float & gy, float & gz)
{
  uint8_t raw[14] = {0};
  if (!i2c_read_bytes(MPU9250_ADDR, 0x3B, raw, 14)) {
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

// ── AK8963 magnetometer (bundled inside the MPU9250) ───────────────
// The AK8963 is wired to the MPU9250's auxiliary I2C. Enabling bypass
// mode on the MPU9250 exposes the AK8963 at 0x0C on the main bus so we
// can read it directly.

// WIA (0x00) identity byte, read at init. 0x48 = AK8963 answering. Kept in a
// global so the 1 Hz status push can report it: this board is an MPU-9265, and
// some of those ship without a working magnetometer die, so "did the chip
// answer" has to be observable rather than assumed.
static uint8_t g_mag_wia = 0x00;
// Factory sensitivity adjustment from the fuse ROM, per axis. Without it the
// three axes are scaled differently by up to ~15% and any heading derived
// from them is skewed.
static float g_mag_asa[3] = {1.0f, 1.0f, 1.0f};
static bool g_mag_overflow = false;   // last read tripped HOFL

static bool ak8963_init()
{
  // Belt and braces: mpu9250_init() already reset the part and cleared
  // I2C_MST_EN, but BYPASS_EN is a no-op while the master interface is on, so
  // make the precondition explicit here too rather than depend on call order.
  uint8_t user_ctrl = 0;
  i2c_read_bytes(MPU9250_ADDR, 0x6A, &user_ctrl, 1);
  if (user_ctrl & 0x20) {
    i2c_write_byte(MPU9250_ADDR, 0x6A, user_ctrl & ~0x20);
    delay(10);
  }
  // INT_PIN_CFG: set BYPASS_EN (bit 1) so the AK8963 sits on the main bus.
  if (!i2c_write_byte(MPU9250_ADDR, 0x37, 0x02)) return false;
  delay(10);

  // Identity check. Anything other than 0x48 means no magnetometer answered
  // and the caller must not publish a field.
  if (!i2c_read_bytes(AK8963_ADDR, 0x00, &g_mag_wia, 1)) return false;
  if (g_mag_wia != 0x48) return false;

  // CNTL1 must pass through power-down before every mode change.
  i2c_write_byte(AK8963_ADDR, 0x0A, 0x00);
  delay(10);
  // Fuse ROM access mode exposes ASAX/ASAY/ASAZ at 0x10..0x12.
  i2c_write_byte(AK8963_ADDR, 0x0A, 0x0F);
  delay(10);
  uint8_t asa[3] = {128, 128, 128};
  i2c_read_bytes(AK8963_ADDR, 0x10, asa, 3);
  for (int i = 0; i < 3; ++i) {
    // Datasheet 8.3.11: Hadj = H * ((ASA - 128) * 0.5 / 128 + 1)
    g_mag_asa[i] = (asa[i] - 128) * 0.5f / 128.0f + 1.0f;
  }
  i2c_write_byte(AK8963_ADDR, 0x0A, 0x00);
  delay(10);
  // 16-bit output, continuous measurement mode 2 (~100 Hz).
  i2c_write_byte(AK8963_ADDR, 0x0A, 0x16);
  delay(10);
  return true;
}

// Probe the AUXILIARY bus directly, driving it with the MPU's own I2C master
// rather than bypassing it onto the main bus. Bypass and master mode are two
// independent routes to the aux bus, and a device could in principle answer on
// one but not the other (bypass MUX not routed on a given module, missing
// pull-ups on ES_DA/ES_CL, and so on). Reading WIA back through EXT_SENS_DATA
// closes that gap: if nothing answers here either, nothing is out there.
//
// Result byte per candidate address 0x0C..0x0F; 0x48 = AK8963 answering.
static uint8_t g_aux_probe[4] = {0, 0, 0, 0};

// Every aux address that ACKed, and how many. A magnetometer need not be an
// AK8963 at 0x0C: HMC5883L/QMC5883L sit at 0x1E/0x0D, LIS3MDL at 0x1C/0x1E,
// BMM150 at 0x10..0x13. Sweeping only the AK8963 range would miss all of them
// if bypass were also broken, so the whole bus gets swept.
static uint8_t g_aux_found[4] = {0, 0, 0, 0};
static int g_aux_found_count = 0;

static void aux_bus_probe()
{
  // Take the aux bus off bypass and hand it to the internal master.
  i2c_write_byte(MPU9250_ADDR, 0x37, 0x00);          // INT_PIN_CFG: BYPASS_EN off
  delay(5);
  i2c_write_byte(MPU9250_ADDR, 0x6A, 0x20);          // USER_CTRL: I2C_MST_EN on
  delay(5);
  i2c_write_byte(MPU9250_ADDR, 0x24, 0x0D);          // I2C_MST_CTRL: 400 kHz aux
  delay(5);

  g_aux_found_count = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
    i2c_write_byte(MPU9250_ADDR, 0x25, 0x80 | addr); // I2C_SLV0_ADDR, read bit
    i2c_write_byte(MPU9250_ADDR, 0x26, 0x00);        // I2C_SLV0_REG
    i2c_write_byte(MPU9250_ADDR, 0x27, 0x81);        // I2C_SLV0_CTRL: enable, 1 byte
    delay(6);                                         // several master cycles at 1 kHz

    // I2C_MST_STATUS bit 0 I2C_SLV0_NACK: set when the addressed slave did not
    // acknowledge. This is a real presence test — reading EXT_SENS_DATA only
    // shows a stale byte when nothing is there. The register clears on read.
    uint8_t st = 0xFF;
    i2c_read_bytes(MPU9250_ADDR, 0x36, &st, 1);
    if (!(st & 0x01) && g_aux_found_count < 4) {
      g_aux_found[g_aux_found_count] = addr;
    }
    if (!(st & 0x01)) ++g_aux_found_count;

    // Keep the AK8963-range identity bytes for reporting.
    if (addr >= 0x0C && addr <= 0x0F) {
      i2c_read_bytes(MPU9250_ADDR, 0x49, &g_aux_probe[addr - 0x0C], 1);
    }
    i2c_write_byte(MPU9250_ADDR, 0x27, 0x00);        // disable slave 0
  }

  // Hand the bus back: master off, bypass on, so ak8963_read() still works if
  // anything did answer.
  i2c_write_byte(MPU9250_ADDR, 0x6A, 0x00);
  delay(5);
  i2c_write_byte(MPU9250_ADDR, 0x37, 0x02);
  delay(5);
}

static bool ak8963_read(float & mx, float & my, float & mz)
{
  uint8_t st1 = 0;
  if (!i2c_read_bytes(AK8963_ADDR, 0x02, &st1, 1)) return false;
  if (!(st1 & 0x01)) { mx = my = mz = 0.0f; return false; }   // no new data
  uint8_t raw[6] = {0};
  if (!i2c_read_bytes(AK8963_ADDR, 0x03, raw, 6)) return false;
  uint8_t st2 = 0;
  // ST2 MUST be read to complete the measurement cycle, and carries HOFL
  // (bit 3): on overflow the sample is garbage and has to be dropped, not
  // published as if it were a field reading.
  if (!i2c_read_bytes(AK8963_ADDR, 0x09, &st2, 1)) return false;
  g_mag_overflow = (st2 & 0x08) != 0;
  if (g_mag_overflow) { mx = my = mz = 0.0f; return false; }
  int16_t rx = (int16_t)((raw[1] << 8) | raw[0]);
  int16_t ry = (int16_t)((raw[3] << 8) | raw[2]);
  int16_t rz = (int16_t)((raw[5] << 8) | raw[4]);
  const float mRes = 0.15f;   // µT/LSB at 16-bit
  mx = rx * mRes * g_mag_asa[0] * 1e-6f;   // Tesla, sensor_msgs/MagneticField
  my = ry * mRes * g_mag_asa[1] * 1e-6f;
  mz = rz * mRes * g_mag_asa[2] * 1e-6f;
  return true;
}

// ── Diagnostics ───────────────────────────────────────────────────────

static int i2c_scan_devices()
{
  int missing = 0;
  Wire.beginTransmission(PCA9685_ADDR);
  if (Wire.endTransmission() != 0) missing |= 1;
  Wire.beginTransmission(MPU9250_ADDR);
  if (Wire.endTransmission() != 0) missing |= 2;
  return missing;
}

// LED blink codes (non-blocking)
//   code 0: solid ON              — everything OK
//   code 1: 100ms period          — PCA9685 missing
//   code 2: 400ms period          — PCA9685 init fail
//   code 3: 2000ms period         — MPU9250 missing
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
  if (g_i2c_busy) return;
  std::vector<uint8_t> found;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
      found.push_back(addr);
  }
  Bridge.notify("i2c_scan", found);
}

// ── Servo diagnostic: schedules a test for loop() ─────────────────────
// Stores test PWMs, defers all I2C work to loop() so Bridge.update()
// never blocks waiting on I2C.
void on_servo_diag()
{
  for (int i = 0; i < 12; ++i)
    g_diag_test_pwms[i] = 100 + i * 200;  // 100, 300, 500, ..., 2300
  g_diag_pending = true;
}

// ── Setup & Loop ──────────────────────────────────────────────────────

void setup()
{
  Wire.begin();
  Wire.setClock(50000);  // 50 kHz — STM32U585 I2C v2 driver is unreliable above this

  g_i2c_scan = i2c_scan_devices();
  g_mpu9250_present = mpu9250_init();
  // Was pinned false for an I2C isolation test and left that way, which is why
  // the field published as a hard zero. ak8963_init() now verifies WIA before
  // claiming the magnetometer exists, so this is safe to trust.
  g_mag_present = ak8963_init();
  // Bypass found nothing -- confirm on the aux bus itself before
  // concluding the magnetometer is absent rather than unreachable.
  if (!g_mag_present) aux_bus_probe();
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

  // Push IMU at 125 Hz (only if sensor was detected)
  if (g_mpu9250_present && now - g_last_imu_push >= IMU_INTERVAL) {
    g_last_imu_push = now;
    float ax, ay, az, gx, gy, gz, mx = 0, my = 0, mz = 0;
    if (mpu9250_read(ax, ay, az, gx, gy, gz)) {
      if (g_mag_present) ak8963_read(mx, my, mz);
      Bridge.notify("imu", ax, ay, az, gx, gy, gz,
                    mx, my, mz, (float)g_imu_sample++, (float)now);
    }
  }

  // Push hardware status at 1 Hz
  if (now - g_last_status_push >= STATUS_INTERVAL) {
    g_last_status_push = now;
    g_i2c_scan = i2c_scan_devices();
    g_mpu9250_present = (g_i2c_scan & 2) == 0;
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
                  g_pwm_readback_ch0,
                  // Magnetometer health: present flag, the WIA identity byte
                  // (0x48 = AK8963 answered) and whether the last read
                  // overflowed. Reported so a zero field can be told apart
                  // from an absent chip without reflashing.
                  g_mag_present ? 1 : 0, (int)g_mag_wia, g_mag_overflow ? 1 : 0,
                  // 0x71=MPU-9250, 0x73=MPU-9255, 0x70=MPU-6500 (no mag die)
                  (int)g_mpu_whoami,
                  // aux-bus WIA readback for 0x0C..0x0F (0x48 = AK8963)
                  (int)g_aux_probe[0], (int)g_aux_probe[1],
                  (int)g_aux_probe[2], (int)g_aux_probe[3],
                  // how many aux addresses ACKed across the full 0x08..0x77
                  // sweep, and the first few of them
                  g_aux_found_count, (int)g_aux_found[0], (int)g_aux_found[1]);
  }

  // LED blink codes (non-blocking)
  blink_update();
}
