#include <Wire.h>

// Standalone MAX30102 raw PPG sanity test. No sensor library required.
// Arduino Uno/Nano I2C: SDA=A4, SCL=A5. Use your board's SDA/SCL pins otherwise.

static const uint8_t MAX30102_ADDR = 0x57;
static const uint8_t REG_INT_STATUS_1 = 0x00;
static const uint8_t REG_FIFO_WR_PTR = 0x04;
static const uint8_t REG_FIFO_OVF = 0x05;
static const uint8_t REG_FIFO_RD_PTR = 0x06;
static const uint8_t REG_FIFO_DATA = 0x07;
static const uint8_t REG_FIFO_CONFIG = 0x08;
static const uint8_t REG_MODE_CONFIG = 0x09;
static const uint8_t REG_SPO2_CONFIG = 0x0A;
static const uint8_t REG_LED1_RED = 0x0C;
static const uint8_t REG_LED2_IR = 0x0D;
static const uint8_t REG_PART_ID = 0xFF;

static bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool readRegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MAX30102_ADDR, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

static bool readFifoSample(uint32_t &red, uint32_t &ir) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(REG_FIFO_DATA);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MAX30102_ADDR, (uint8_t)6) != 6) return false;

  uint8_t b[6];
  for (uint8_t i = 0; i < 6; ++i) b[i] = Wire.read();

  red = (((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2]) & 0x3FFFF;
  ir  = (((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5]) & 0x3FFFF;
  return true;
}

static bool configureSensor() {
  // Reset, then wait for the reset bit to clear.
  if (!writeRegister(REG_MODE_CONFIG, 0x40)) return false;
  delay(100);

  // Clear FIFO pointers/overflow; average 4 samples and enable rollover.
  if (!writeRegister(REG_FIFO_WR_PTR, 0x00)) return false;
  if (!writeRegister(REG_FIFO_OVF, 0x00)) return false;
  if (!writeRegister(REG_FIFO_RD_PTR, 0x00)) return false;
  if (!writeRegister(REG_FIFO_CONFIG, 0x50)) return false;

  // SpO2 mode (red + IR), 4096 nA range, 100 samples/s, 411 us / 18-bit.
  if (!writeRegister(REG_SPO2_CONFIG, 0x27)) return false;

  // Set red and IR LED currents to a moderate starting level.
  if (!writeRegister(REG_LED1_RED, 0x24)) return false;
  if (!writeRegister(REG_LED2_IR, 0x24)) return false;

  // Start continuous red+IR acquisition.
  if (!writeRegister(REG_MODE_CONFIG, 0x03)) return false;
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  uint8_t partId = 0;
  if (!readRegister(REG_PART_ID, partId)) {
    Serial.println("ERROR: MAX30102 not responding at I2C address 0x57.");
    Serial.println("Check VCC, GND, SDA, and SCL.");
    while (true) delay(1000);
  }

  if (!configureSensor()) {
    Serial.println("ERROR: MAX30102 register setup failed.");
    while (true) delay(1000);
  }

  Serial.print("MAX30102 found; PART_ID=0x");
  Serial.println(partId, HEX);
  Serial.println("time_ms,red,ir");
}

void loop() {
  uint8_t writePtr = 0;
  uint8_t readPtr = 0;
  if (!readRegister(REG_FIFO_WR_PTR, writePtr) ||
      !readRegister(REG_FIFO_RD_PTR, readPtr)) {
    Serial.println("ERROR: FIFO pointer read failed.");
    delay(250);
    return;
  }

  uint8_t samples = (writePtr - readPtr) & 0x1F;
  while (samples--) {
    uint32_t red = 0;
    uint32_t ir = 0;
    if (!readFifoSample(red, ir)) {
      Serial.println("ERROR: FIFO sample read failed.");
      break;
    }
    Serial.print(millis());
    Serial.print(',');
    Serial.print(red);
    Serial.print(',');
    Serial.println(ir);
  }
}
