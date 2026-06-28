#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <vector>

static std::vector<uint8_t> scan_i2c()
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

static unsigned long last_scan = 0;

void setup()
{
  Wire.begin();
  Wire.setClock(400000);
  Bridge.begin();
}

void loop()
{
  Bridge.task();
  unsigned long now = millis();
  if (now - last_scan >= 5000) {
    last_scan = now;
    auto addrs = scan_i2c();
    Bridge.notify("i2c_scan_result", addrs);
  }
}
