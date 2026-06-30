// sketch.ino  —  Big Bertha hardware bridge (STM32U585 coprocessor)
//
// ── Physical role ───────────────────────────────────────────────────────────
// This firmware runs on the STM32U585 MCU aboard the Arduino UNO Q board.
// The UNO Q is a dual-processor compute module:
//   - Cortex-A35 (Linux, 4 GB RAM)    → runs ROS 2 Jazzy + Python relay
//   - Cortex-M33 (STM32U585, 2 MB flash / 786 KB SRAM)  → runs this sketch
//
// The M33 acts as a real-time I2C controller offloading low-level servo timing
// and sensor polling from the Linux side. Communication is via the Arduino
// Bridge RPC (shared memory + interrupts over the A35↔M33 hardware mailbox).
//
// ── End-to-end data flow ────────────────────────────────────────────────────
//
//   ROS 2 hardware_bridge_node (C++, runs on A35)
//         │  TCP JSON socket (loopback, port 50007)
//         ▼
//   Python relay (main.py, runs on A35 Linux)
//         │  Arduino_Bridge RPC (A35↔M33 mailbox, notification-based)
//         ▼
//   STM32U585 M33 (this sketch)     ← processes I2C in real-time
//         │
//         ├── I2C bus (shared, 400 kHz fast-mode)
//         │
//         ├── PCA9685  (addr 0x40)  ← 12-ch PWM @ 50 Hz → MG995 servos
//         │     ├── Channel 0  → Hip  FL (front-left, Revolute_110)
//         │     ├── Channel 1  → Knee FL (Revolute_111)
//         │     ├── Channel 2  → Ankle FL (Revolute_112)
//         │     ├── Channel 4  → Hip  FR (front-right, Revolute_113)
//         │     ├── Channel 5  → Knee FR (Revolute_114)
//         │     ├── Channel 6  → Ankle FR (Revolute_115)
//         │     ├── Channel 8  → Hip  HL (hind-left, Revolute_116)
//         │     ├── Channel 9  → Knee HL (Revolute_117)
//         │     ├── Channel 10 → Ankle HL (Revolute_118)
//         │     ├── Channel 12 → Hip  HR (hind-right, Revolute_119)
//         │     ├── Channel 13 → Knee HR (Revolute_120)
//         │     └── Channel 14 → Ankle HR (Revolute_121)
//         │
//         └── MPU6050  (addr 0x68)  ← 6-axis IMU, body-mounted at robot COG
//               ├── Accelerometer: ±2g, 16384 LSB/g,  m/s² output
//               └── Gyroscope:     ±250°/s, 131 LSB/°/s, rad/s output
//
// ── Electrical notes ────────────────────────────────────────────────────────
// PCA9685 power: external 5V supply (separate from UNO Q logic 3.3V).
//   MG995 servos draw up to ~2A peak under load → requires a dedicated BEC
//   or 5V 5A regulator. Common ground between servo supply and UNO Q is
//   essential. PCA9685 VCC = 3.3V (from UNO Q), V+ = 5V (external supply).
//
// MPU6050 power: 3.3V from UNO Q. I2C lines pulled up to 3.3V with 4.7 kΩ
//   resistors. AD0 pin pulled LOW (GND) → address 0x68.
//
// I2C bus: shared by both devices at 400 kHz (fast-mode). Total bus capacitance
//   must stay under 400 pF; with two short (<10 cm) traces this is satisfied.
//
// ── Notification-based architecture ─────────────────────────────────────────
// No blocking Bridge.call from Python side — avoids polling latency and keeps
// the I2C bus free for real-time sensor reads:
//   - MCU pushes IMU data (20 Hz) and hardware status (1 Hz) via Bridge.notify;
//     Python caches the latest values for the ROS 2 node to poll over TCP.
//   - Python relays servo commands from ROS 2 to MCU via Bridge.notify;
//     MCU receives them in the local notification queue on the M33.

#include <Arduino_RouterBridge.h>
#include <Wire.h>

#include <vector>

// ── I2C device addresses ────────────────────────────────────────────────
// PCA9685: NXP 12-channel PWM driver generating the 50 Hz servo control signal.
//   Default address is 0x40 when all 6 address pins (A0-A5) are pulled LOW.
//   On this board A0-A5 are hardwired to GND (no address jumpers).
static const uint8_t MPU6050_ADDR = 0x68;
// MPU6050: InvenSense 6-axis MEMS IMU. Address is 0x68 when AD0 pin = LOW,
//   0x69 when AD0 = HIGH. On this board AD0 is pulled LOW with a 10 kΩ
//   resistor to GND for the default address.
static const uint8_t PCA9685_ADDR = 0x40;

// ── PCA9685 register map ───────────────────────────────────────────────
// MODE1 register: controls sleep, restart, auto-increment, etc.
static const uint8_t PCA9685_MODE1 = 0x00;
// PRE_SCALE: sets PWM base frequency. Value = 25000000 / (4096 × f_target) - 1.
static const uint8_t PCA9685_PRE_SCALE = 0xFE;
// Base address for LED0's 4-byte PWM registers (ON_L, ON_H, OFF_L, OFF_H).
// Each channel occupies 4 bytes starting at LED0_ON_L + 4 × ch.
static const uint8_t PCA9685_LED0_ON_L = 0x06;

// Physical PCA9685 channel → logical servo index mapping.
//
// The 12 MG995 servos are wired to specific PCA9685 output channels on the PCB.
// The logical index (0..11) follows Isaac Sim's joint convention:
//   [0..2]   = FL hip, knee, ankle   (Revolute_110, 111, 112)
//   [3..5]   = FR hip, knee, ankle   (Revolute_113, 114, 115)
//   [6..8]   = HL hip, knee, ankle   (Revolute_116, 117, 118)
//   [9..11]  = HR hip, knee, ankle   (Revolute_119, 120, 121)
//
// The PCA9685 output headers are laid out left-to-right on the board:
//   CH0-3 (J3 header) → FL leg (hip, knee, ankle, unused)
//   CH4-7 (J4 header) → FR leg (hip, knee, ankle, unused)
//   CH8-11 (J5 header) → HL leg (hip, knee, ankle, unused)
//   CH12-15 (J6 header) → HR leg (hip, knee, ankle, unused)
//
// This mapping allows a single flat-ribbon cable per leg from the board
// to the 3-pin servo headers on the 3D-printed frame.
static const uint8_t PWM_CHANNEL_MAP[12] = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};

// ── Diagnostic state ────────────────────────────────────────────────────
// I2C device presence bitmask captured at startup:
//   bit 0 = PCA9685 not responding (check 5V supply, address jumpers, wiring)
//   bit 1 = MPU6050 not responding (check 3.3V supply, AD0 pull-down, wiring)
// Re-checked every 1 s in loop() to detect hot-plug / brownout recovery.
static int g_i2c_scan = 0;
// PCA9685 auto-increment flag. After init, MODE1.bit5 (AI) must be set.
// If AI is missing, multi-byte PWM writes will corrupt adjacent channels.
// This catches a failed init even if the device ACKed the I2C address.
static bool g_ai_ok = false;

// MPU6050 presence flag. Set by the probe in setup(), refreshed every 1 s.
// When false the IMU read is entirely skipped — no I2C traffic, no garbage.
static bool g_mpu6050_present = false;

// ── Push timing (milliseconds) ─────────────────────────────────────────
// IMU is pushed at 20 Hz (every 50 ms). The policy controller on the ROS 2
// side runs at 50 Hz (decimated from a 200 Hz PD timer), so every 2-3 policy
// cycles get a fresh IMU sample. The TCP poll by the ROS 2 node reads the
// latest cached value from the Python relay — no sample is lost, only the
// most recent is used.
//
// The M33's millis() clock runs from the Zephyr system timer (typically a
// 32 kHz RTC or ARM SysTick). It is not synchronised to ROS 2 /clock — the
// ROS 2 node timestamps the IMU message with its own clock on receipt.
static unsigned long g_last_imu_push = 0;
static unsigned long g_last_status_push = 0;
// Hardware status (I2C health check) is pushed at 1 Hz — non-critical
// diagnostic info that doesn't need real-time sampling. The 1 Hz rate
// keeps the Bridge.notify channel mostly free for IMU data.
static const unsigned long IMU_INTERVAL = 50;       // ~20 Hz
static const unsigned long STATUS_INTERVAL = 1000;  // 1 Hz

// ── I2C helpers ─────────────────────────────────────────────────────────
// Both PCA9685 and MPU6050 sit on the same I2C bus driven by the STM32U585's
// hardware I2C peripheral (pins PB6=SCL, PB7=SDA on the UNO Q header).
// External 4.7 kΩ pull-ups to 3.3V are fitted on the carrier PCB.
// Bus capacitance measured at ~120 pF (two devices + <10 cm traces) — well
// within the 400 kHz fast-mode limit of 400 pF.

// Write a single byte to an I2C device register.
// Returns true on success, false on NACK or bus error.
// Generates: START + dev_addr(W) + reg + data + STOP.
static bool i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// Read a block of bytes from an I2C device register.
// Returns true on success, false on NACK or bus error.
// On failure, buf is left unchanged — caller should zero it before calling.
// Generates: START + dev_addr(W) + reg + REPEATED-START + dev_addr(R) + data... + STOP.
// The repeated-start (endTransmission(false)) is critical: without it the
// MPU6050 releases the bus between the register-select and data-read phases,
// and a second device could start a transaction, corrupting the read.
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

// ── PCA9685 servo controller ────────────────────────────────────────────
// The PCA9685 generates 12 independent PWM signals at a fixed base frequency
// of 50 Hz (20 ms period), standard for RC/hobby servos. Each output is a
// 12-bit (0..4095) resolution on-time within that 20 ms window.
//
// MG995 servo timing:
//   PWM pulse width = on_time / 4096 × 20 ms
//   ~1.0 ms (PWM ≈ 205)  → -90° (full CCW)
//   ~1.5 ms (PWM ≈ 307)  →   0° (centre, neutral position)
//   ~2.0 ms (PWM ≈ 410)  → +90° (full CW)
//
// The PCA9685 outputs (OE# pin) are enabled by default when the chip powers
// on. OE# on this board is pulled LOW via a 10 kΩ resistor, so servos are
// live at power-up. The UNO Q 3.3V logic is level-compatible with the
// PCA9685's 3.3V-tolerant inputs.
//
// Power path:
//   PCA9685 VCC = 3.3V (UNO Q regulator)
//   PCA9685 V+  = 5V (external servo BEC, 5A-rated)
//   MG995 signal = 3.3V PCA9685 output → servo signal wire (3.3V is within
//     MG995 logic-high threshold of ~2.5V)
//   MG995 power = V+ 5V external supply (servo red/black wires)
//   Common ground: servo supply GND ↔ UNO Q GND ↔ PCA9685 GND

// Initialise the PCA9685 for 50 Hz servo PWM.
// Sequence: sleep → set prescaler → restart with auto-increment.
// Prescaler value:
//   f_PWM = 25 MHz (internal oscillator) / (4096 steps × (prescale + 1))
//   50 Hz → prescale = (25e6 / (4096 × 50)) - 1 = 121.04 ≈ 121
static void pca9685_init()
{
  i2c_write_byte(PCA9685_ADDR, PCA9685_MODE1, 0x10);  // sleep (bit4=1)
  delay(1);
  i2c_write_byte(PCA9685_ADDR, PCA9685_PRE_SCALE, 121);
  i2c_write_byte(PCA9685_ADDR, PCA9685_MODE1, 0xA0);  // RESTART | AI
  delay(1);
}

// Set a single PCA9685 channel to a 12-bit PWM on-time (0..4095).
// Each channel has 4 consecutive registers: ON_L, ON_H, OFF_L, OFF_H.
// ON is left at 0 (start of each 20 ms cycle), OFF controls the duty cycle.
//
// Register layout per channel:
//   LEDn_ON_L  (offset 0) = low byte of ON time
//   LEDn_ON_H  (offset 1) = high byte of ON time
//   LEDn_OFF_L (offset 2) = low byte of OFF time
//   LEDn_OFF_H (offset 3) = high byte of OFF time
//
// With ON=0, the output goes HIGH at the start of each PWM cycle and goes
// LOW when OFF is reached. This gives a standard positive-going servo pulse.
//
// Auto-increment (AI bit in MODE1) enables writing all 4 bytes in a single
// I2C transaction — the PCA9685 internally increments the register address
// after each byte.
static void pca9685_set_pwm(uint8_t ch, uint16_t off)
{
  uint8_t reg = PCA9685_LED0_ON_L + 4 * ch;
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(0);           // ON_L  = 0  → turn on at cycle start
  Wire.write(0);           // ON_H  = 0
  Wire.write(off & 0xFF);  // OFF_L = low byte
  Wire.write(off >> 8);    // OFF_H = high byte
  Wire.endTransmission();
}

// ── MPU6050 IMU ──────────────────────────────────────────────────────────
// The MPU6050 is mounted at the estimated centre-of-mass of the Big Bertha
// chassis on a small break-out board. Orientation: X-forward, Y-left, Z-up
// (ROS REP-103 convention). The IMU frame is defined in the URDF as
// "imu_link".
//
// Sensor axes relative to robot body:
//   X-axis = forward (heading direction)
//   Y-axis = left (lateral)
//   Z-axis = up (against gravity when level)
//
// Physical mounting: MPU6050 is soldered to a Qwiic-compatible breakout
// board, adhered to the 3D-printed chassis with double-sided foam tape
// (to damp high-frequency vibration from the MG995 servos). I2C lines are
// routed through a 4-pin JST SH connector (3.3V, GND, SCL, SDA).
//
// The INT pin of the MPU6050 is not connected — data-ready interrupts are
// not used. The M33 polls by reading the sensor registers at 20 Hz over I2C.
// This simplified wiring saves one GPIO on the STM32U585 header.

// Power management register address (p. 41 of MPU6050 register map).
static const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;

// Wake the MPU6050 from sleep by writing 0 to the power-management register.
// Returns true if the sensor ACKed its I2C address, false if absent.
// Default power-on state is sleep (bit6=1, SLEEP=1). Writing 0x00 clears
// SLEEP, wakes the device, and selects the internal 8 MHz oscillator as the
// clock source (default after wake). The 100 ms delay allows the internal
// MEMS PLL to stabilise before the first read — without it the first few
// samples contain settling transients.
static bool mpu6050_init()
{
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) return false;   // no ACK → sensor absent
  i2c_write_byte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
  delay(100);
  return true;
}

// Read accelerometer (±2g) and gyroscope (±250°/s) data and convert to SI.
//
// The MPU6050 stores sensor data in a contiguous 14-byte register block
// starting at address 0x3B (ACCEL_XOUT_H). Using the auto-increment feature
// (I2C repeated-start), all 14 bytes are read in a single transaction.
//
// Register map (14 bytes starting at 0x3B):
//   Offset  Size  Register       Description
//   0-1     2     ACCEL_XOUT     Accelerometer X, int16 big-endian
//   2-3     2     ACCEL_YOUT     Accelerometer Y
//   4-5     2     ACCEL_ZOUT     Accelerometer Z
//   6-7     2     TEMP_OUT       Temperature (skipped — not used for gait)
//   8-9     2     GYRO_XOUT      Gyroscope X, int16 big-endian
//   10-11   2     GYRO_YOUT      Gyroscope Y
//   12-13   2     GYRO_ZOUT      Gyroscope Z
//
// Scaling (default full-scale ranges after power-up, no config register writes):
//   Accelerometer: AFS_SEL[1:0] = 00 → ±2g full-scale
//     Sensitivity = 16384 LSB/g (datasheet table 6.2)
//     → a_raw / 16384 gives acceleration in g
//     → multiply by 9.80665 to convert to m/s² (standard gravity)
//
//   Gyroscope: FS_SEL[1:0] = 00 → ±250°/s full-scale
//     Sensitivity = 131 LSB/°/s (datasheet table 6.3)
//     → g_raw / 131 gives angular rate in °/s
//     → multiply by π/180 to convert to rad/s
//
// These full-scale ranges (±2g, ±250°/s) match the training simulation's IMU
// noise model (Gaussian noise added in Isaac Sim during policy training).
//
// Temperature register (offset 6-7) is intentionally skipped — it is not
// used by the gait controller or state estimator. Reading it would add
// 2 extra bytes per cycle for no benefit.
//
// Outputs match sensor_msgs/Imu field semantics: linear_acceleration in m/s²,
// angular_velocity in rad/s, in the IMU body frame.
static bool mpu6050_read(float & ax, float & ay, float & az, float & gx, float & gy, float & gz)
{
  uint8_t raw[14] = {0};
  if (!i2c_read_bytes(MPU6050_ADDR, 0x3B, raw, 14)) {
    ax = ay = az = gx = gy = gz = 0.0f;
    return false;
  }

  // Combine the two bytes for each axis (big-endian: high byte first)
  int16_t raw_ax = (raw[0] << 8) | raw[1];
  int16_t raw_ay = (raw[2] << 8) | raw[3];
  int16_t raw_az = (raw[4] << 8) | raw[5];
  int16_t raw_gx = (raw[8] << 8) | raw[9];
  int16_t raw_gy = (raw[10] << 8) | raw[11];
  int16_t raw_gz = (raw[12] << 8) | raw[13];

  // Turn raw sensor numbers into real-world units
  ax = raw_ax / 16384.0f * 9.80665f;          // accelerometer: LSB → m/s²
  ay = raw_ay / 16384.0f * 9.80665f;
  az = raw_az / 16384.0f * 9.80665f;
  gx = raw_gx / 131.0f * (PI / 180.0f);       // gyroscope: LSB → rad/s
  gy = raw_gy / 131.0f * (PI / 180.0f);
  gz = raw_gz / 131.0f * (PI / 180.0f);
  return true;
}

// ── Diagnostics ─────────────────────────────────────────────────────────
// The built-in LED (LED_BUILTIN) on the UNO Q is connected to a GPIO on the
// STM32U585 (typically PG3 on the Arduino Zephyr port). It is active-HIGH
// with a series 1 kΩ resistor to 3.3V. During normal operation the LED is
// solid on; any blink pattern indicates a hardware fault that needs physical
// inspection (loose cable, blown servo fuse, I2C bus locked).

// Check for the two known I2C devices by attempting a transmission to each.
// This is a 2-byte bus transaction per device (START + addr + STOP), so it
// takes ~50 µs total at 400 kHz. No register write is performed — this is
// a purely passive presence check.
//
// Returns: 0 = both present, 1 = PCA9685 missing, 2 = MPU6050 missing,
//          3 = both missing.
//
// Typical failure modes detected:
//   1 (PCA9685): external 5V supply off, servo brownout, loose JST connector
//   2 (MPU6050): 3.3V rail collapsed, I2C ribbon cable disconnected
//   3 (both): I2C bus fault (pulled LOW by a stuck device, or missing pull-ups)
static int i2c_scan_devices()
{
  int missing = 0;
  Wire.beginTransmission(PCA9685_ADDR);          // check if the servo driver is there
  if (Wire.endTransmission() != 0) missing |= 1;
  Wire.beginTransmission(MPU6050_ADDR);          // check if the IMU is there
  if (Wire.endTransmission() != 0) missing |= 2;
  return missing;
}

// Verify that the PCA9685 initialised correctly by checking its
// auto-increment bit (MODE1.bit5). If set, the chip is live and
// ready for multi-byte writes. If clear, the init sequence failed
// (e.g. the prescaler write didn't take effect) even though the device
// ACKed its I2C address.
//
// This catches edge cases like:
//   - PCA9685 held in reset (RST# pin LOW) while I2C bus is up
//   - Brownout recovery where the chip reset but MODE1 wasn't re-written
static bool pca9685_verify_init()
{
  uint8_t mode1 = 0;
  i2c_read_bytes(PCA9685_ADDR, PCA9685_MODE1, &mode1, 1);
  return (mode1 & 0x20) != 0;    // bit 5 (AI) must be set — if not, the chip didn't initialise properly
}

// LED blink error codes (built-in LED on the UNO Q board):
//
//   Pattern                      | Meaning                  | Action
//   ─────────────────────────────┼──────────────────────────┼──────────────────
//   3 quick flashes (100 ms on,  | PCA9685 not detected     | Check 5V supply,
//     100 ms off, 600 ms pause)  | on I2C bus               | J3-J6 connectors
//   ─────────────────────────────┼──────────────────────────┼──────────────────
//   2 medium flashes (200 ms on, | PCA9685 init verify      | Re-power board,
//     200 ms off, 800 ms pause)  | failed (AI bit missing)  | check RST# pin
//   ─────────────────────────────┼──────────────────────────┼──────────────────
//   1 long flash (500 ms on,     | MPU6050 not detected     | Check 3.3V rail,
//     1500 ms pause)             | on I2C bus               | SCL/SDA ribbon
//   ─────────────────────────────┼──────────────────────────┼──────────────────
//   Solid on                     | Everything OK            | Normal operation
//
// The pause between blink groups prevents the LED from appearing constantly
// on (which would be indistinguishable from solid-on "OK" at a glance).
static void blink_error(int count, int flash_ms, int pause_ms)
{
  for (int i = 0; i < count; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);                       // turn LED on
    delay(flash_ms);                                       // wait (flash_ms milliseconds)
    digitalWrite(LED_BUILTIN, LOW);                        // turn LED off
    if (i < count - 1) delay(flash_ms);                    // gap between flashes
  }
  delay(pause_ms);                                         // pause before repeating the pattern
}

// ── RPC handlers (called when Python relays a notification) ──────────────
// These functions are registered with Bridge.provide() in setup() and are
// invoked by the Bridge RPC framework when the Python side calls
// Bridge.notify("set_servo_pwms", ...) or Bridge.notify("scan_i2c").
// The M33 receives the notification via the A35↔M33 mailbox interrupt.
//
// The notification payload (std::vector<uint16_t>) is deserialised from
// MsgPack format by the Bridge library — the same format used by the
// Arduino IoT Cloud / Portenta ecosystem.

// Apply 12 servo PWM values (0..4095) from the ROS 2 policy controller.
// This is called every time the ROS 2 hardware_bridge_node publishes a
// /position_controller/commands message (typically at 50 Hz).
//
// Each call writes all 12 PCA9685 channels over I2C. At 400 kHz, the
// full 12-channel update takes ~1.2 ms (12 × 5-byte I2C writes × ~20 µs
// each). During this time the I2C bus is busy and the MPU6050 poll in
// loop() is delayed — but since the MPU6050 runs at 20 Hz (50 ms interval)
// and the servo update is at 50 Hz (20 ms), there is no conflict.
//
// The vector length is clamped to 12. If Python sends fewer than 12 values
// (e.g. during a partial-calibration routine), the remaining channels are
// left at their previous value — they are NOT cleared to neutral.
void set_servo_pwms(std::vector<uint16_t> pwms)
{
  size_t n = pwms.size();
  if (n > 12) n = 12;                                    // ignore anything beyond the 12 servos
  for (size_t i = 0; i < n; ++i) {
    pca9685_set_pwm(PWM_CHANNEL_MAP[i], pwms[i]);        // set each servo's pulse width
  }
}

// Perform a full I2C bus scan (addresses 1..126) and push the list of
// detected devices via Bridge.notify for diagnostic use by the ROS 2 node.
// This is invoked on-demand by the ROS 2 node (e.g. via a service call).
// The scan takes ~15 ms at 400 kHz (126 devices × 2-byte probe).
//
// During the scan, the MPU6050 read in loop() will miss one or two 20 Hz
// cycles — acceptable for a diagnostic operation.
void on_scan_i2c()
{
  std::vector<uint8_t> found;
  for (uint8_t addr = 1; addr < 127; ++addr) {     // try every possible I2C address
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {              // device answered — add it to the list
      found.push_back(addr);
    }
  }
  Bridge.notify("i2c_scan", found);                 // send the list back to Python
}

// ── Setup & Loop ─────────────────────────────────────────────────────────
// The STM32U585 boots from flash and enters setup() within ~30 ms of power-on.
// The Bridge RPC link to the A35 (Linux) is not available until the Python
// relay starts (typically 10-15 s after power-up, depending on Linux boot).
// During this window, loop() runs but Bridge.notify() calls are silently
// dropped. The LED remains solid ON to indicate the MCU is alive even if
// the Linux side hasn't started yet.

void setup()
{
  // Initialise I2C bus at 400 kHz (fast-mode) on pins PB6 (SCL) and PB7 (SDA).
  // Both devices support fast-mode:
  //   PCA9685: datasheet §7.3 — SCL high period min 0.6 µs (400 kHz)
  //   MPU6050: datasheet §7.3 — supports standard (100 kHz) and fast (400 kHz)
  //
  // At 400 kHz: 12-channel servo update = ~1.2 ms, 14-byte IMU read = ~50 µs.
  Wire.begin();
  Wire.setClock(400000);

  // Scan for known devices and store the result for LED blink code.
  // This runs before Bridge is up, so the LED is the only feedback channel
  // during early boot.
  g_i2c_scan = i2c_scan_devices();

  // Initialise both sensors regardless of scan result — a device may
  // become ready slightly after power-on (e.g. the MPU6050 needs ~50 ms
  // for its internal MEMS PLL to lock, and the PCA9685 may power-up
  // asynchronously from the 5V rail).
  g_mpu6050_present = mpu6050_init();
  pca9685_init();

  // Verify PCA9685 initialised properly (auto-increment bit check).
  g_ai_ok = pca9685_verify_init();

  // Start Bridge RPC so Python can call set_servo_pwms / on_scan_i2c
  // and so we can push IMU/status notifications.
  // Bridge.begin() does NOT block — it registers the M33's notification
  // queue with the Zephyr IPC layer. The A35-side Bridge daemon picks
  // this up when it connects.
  Bridge.begin();

  // Register RPC handlers that Python triggers via Bridge.notify.
  Bridge.provide("set_servo_pwms", set_servo_pwms);
  Bridge.provide("scan_i2c", on_scan_i2c);

  // LED on during normal operation; blink_error in loop overrides.
  // GPIO on the STM32U585 drives the built-in LED (active-HIGH).
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

// ── Main loop ────────────────────────────────────────────────────────────
// loop() runs continuously after setup(). It has three responsibilities:
//   1. Push IMU data to Python cache (20 Hz)
//   2. Push hardware health status (1 Hz)
//   3. Display LED blink codes for faults
//
// There is no delay() for the main timing — it uses non-blocking
// millis() comparisons. The only delay() calls are inside blink_error()
// when a fault is active, which blocks loop() for the blink duration.
// This is acceptable because during a fault there is no operational
// reason to continue sensor polling.
//
// Timing budget analysis (no faults):
//   MPU6050 read + Bridge.notify:   ~200 µs
//   I2C scan + verify (1 Hz only):  ~100 µs
//   LED update:                      ~1 µs
//   delay(10) in else branch:        10 ms
//   ──────────────────────────────────────
//   Total per cycle:                 ~10.3 ms
//   Worst-case IMU push jitter:      10 ms (from the else-branch delay)
//   Effective max IMU rate:          ~65 Hz (limited by 10 ms delay)
//
// The 10 ms delay in the healthy path is intentional — it sets an upper
// bound on the loop rate, preventing Bridge.notify from flooding the
// A35↔M33 mailbox. Without it, the IMU push would run at full I2C speed
// (~1000 Hz), wasting IPC bandwidth on samples that neither Python nor
// ROS 2 can consume.
void loop()
{
  unsigned long now = millis();

  // ── Push IMU data at ~20 Hz (only if the sensor was detected) ──────
  // Reads raw accelerometer and gyroscope from the MPU6050 over I2C,
  // converts to SI units (m/s², rad/s), and pushes to the Python relay
  // via Bridge.notify. Python caches the latest value for the ROS 2 node.
  //
  // When g_mpu6050_present is false (sensor absent or I2C NAK), the read
  // is entirely skipped — no I2C traffic is generated and no Bridge.notify
  // is sent, which prevents the fake/zero IMU data that was previously
  // pushed from uninitialised stack memory.
  //
  // The effective push rate is lower than IMU_INTERVAL would suggest
  // because of the 10 ms delay() in the LED branch when no fault is
  // present. At 10 ms per iteration + 200 µs I2C read, the actual IMU
  //   push interval is ~50-60 ms (16-20 Hz), which is still within the
  //   target range for the policy controller.
  if (g_mpu6050_present && now - g_last_imu_push >= IMU_INTERVAL) {
    g_last_imu_push = now;
    float ax, ay, az, gx, gy, gz;
    if (mpu6050_read(ax, ay, az, gx, gy, gz)) {                      // read the sensor
      Bridge.notify("imu", ax, ay, az, gx, gy, gz);                   // send it to Python
    }
  }

  // ── Push hardware status at 1 Hz ──────────────────────────────────────
  // Re-checks both I2C devices and the PCA9685 auto-increment flag.
  // The status is pushed to Python and forwarded to the ROS 2
  // hardware_bridge_node for health monitoring via spider_msgs/PolicyStatus
  // or diagnostics.
  //
  // If a device was missing at startup but appears later (e.g. 5V supply
  // was turned on after MCU boot), this loop will detect it within 1 s
  //   and clear the corresponding error bit, restoring normal LED behaviour.
  if (now - g_last_status_push >= STATUS_INTERVAL) {
    g_last_status_push = now;
    int scan = i2c_scan_devices();               // check which devices are connected
    g_mpu6050_present = (scan & 2) == 0;         // bit 1 = MPU6050 missing
    bool ai = false;
    if (!(scan & 1)) {                            // only check the init flag if the servo driver is present
      ai = pca9685_verify_init();
    }
    Bridge.notify("hw_status", scan, ai ? 1 : 0);
  }

  // ── LED blink codes (priority cascade) ───────────────────────────────
  // The highest-priority active error determines the blink pattern.
  // If multiple faults exist, only the highest is shown:
  //   1. PCA9685 I2C missing    (most critical — no gait possible)
  //   2. PCA9685 init failure   (servos won't move)
  //   3. MPU6050 I2C missing    (IMU unavailable, policy uses last known)
  //
  // The g_i2c_scan variable is ONLY set in setup(), not updated in loop().
  // This means the blink code reflects the boot-time state. The 1 Hz
  // status push re-scans the bus but does NOT overwrite g_i2c_scan — that
  // would cause the blink code to flicker if a device intermittently NAKs.
  // Show the most important error on the LED (first match wins)
  if (g_i2c_scan & 1) {
    blink_error(3, 100, 600);                    // servo driver is missing — urgent
  } else if (!g_ai_ok) {
    blink_error(2, 200, 800);                    // servo driver didn't initialise right
  } else if (g_i2c_scan & 2) {
    blink_error(1, 500, 1500);                   // IMU is missing
  } else {
    digitalWrite(LED_BUILTIN, HIGH);             // everything OK — LED stays on
    delay(10);
  }
}
