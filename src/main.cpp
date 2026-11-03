#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Arduino.h>
 
// ---------- Pin configuration ----------
const int SD_CS = D10;
 
// AS5600 I2C address and raw angle register
const uint8_t AS5600_ADDR = 0x36;
const uint8_t RAW_ANGLE_REG = 0x0E;
 
// ---------- Logging configuration ----------
const uint32_t SAMPLE_INTERVAL_US = 10000; // 100 Hz
const int FLUSH_SAMPLES = 50;
 
File logFile;
 
char logBuffer[4096];
size_t bufferUsed = 0;
int samplesBuffered = 0;
 
uint32_t startTimeUs = 0;
uint32_t nextSampleUs = 0;
uint32_t sampleCount = 0;
 
// Read the 12-bit raw angle from the AS5600
bool readAS5600(uint16_t &rawAngle) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_REG);
 
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
 
  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) {
    return false;
  }
 
  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();
 
  rawAngle = ((uint16_t)highByte << 8) | lowByte;
  rawAngle &= 0x0FFF; // 12-bit measurement
 
  return true;
}
 
// Write accumulated samples to the SD card
bool flushBuffer() {
  if (bufferUsed == 0) {
    return true;
  }
 
  size_t written =
      logFile.write((uint8_t *)logBuffer, bufferUsed);
 
  logFile.flush();
 
  if (written != bufferUsed) {
    Serial.println("SD write error!");
    return false;
  }
 
  bufferUsed = 0;
  samplesBuffered = 0;
 
  return true;
}
 void halt(const char *msg) {
  while (true) {
    Serial.println(msg);
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10);
  Serial.println("boot");

  Wire.begin();
  Wire.setClock(400000);
  Serial.println("i2c ok");

  if (!SD.begin(SD_CS)) halt("SD card initialization failed!");
  Serial.println("sd ok");

  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() != 0) halt("AS5600 not detected!");
  Serial.println("as5600 ok");

  SD.remove("/angle.csv");
  Serial.println("removed");

  logFile = SD.open("/angle.csv", FILE_WRITE);
  Serial.println("opened");
  if (!logFile) halt("Could not create angle.csv!");

  logFile.println("time_us,raw_angle,angle_deg");
  Serial.println("header written");
  logFile.flush();
  Serial.println("flushed");

  startTimeUs = micros();
  nextSampleUs = startTimeUs;
  Serial.println("Logging started.");
}
 
void loop() {
  uint32_t now = micros();
 
  // Wait until the next scheduled sample
  if ((int32_t)(now - nextSampleUs) < 0) {
    return;
  }
 
  // Schedule relative to actual time to avoid
  // burst sampling after a slow SD write.
  nextSampleUs = now + SAMPLE_INTERVAL_US;
 
  uint16_t rawAngle;
 
  if (!readAS5600(rawAngle)) {
    Serial.println("AS5600 read error!");
    return;
  }
 
  // Timestamp in microseconds since logging began
  uint32_t timestamp = micros() - startTimeUs;
 
  // Convert 12-bit angle (0-4095) to degrees
  float angleDeg = rawAngle * (360.0f / 4096.0f);
 
  // Format one CSV row
  char line[64];
 
  int len = snprintf(
    line, sizeof(line),
    "%lu,%u,%.3f\n",
    (unsigned long)timestamp,
    rawAngle,
    angleDeg
  );
 
  if (len <= 0 || len >= (int)sizeof(line)) {
    return;
  }
 
  // Avoid overflowing the RAM buffer
  if (bufferUsed + len >= sizeof(logBuffer)) {
    if (!flushBuffer()) return;
  }
 
  memcpy(logBuffer + bufferUsed, line, len);
  bufferUsed += len;
  samplesBuffered++;
  sampleCount++;
 
  // Flush about once every 0.5 seconds
  if (samplesBuffered >= FLUSH_SAMPLES) {
    if (!flushBuffer()) {
      Serial.println("Logging stopped due to SD error.");
      while (true) delay(1000);
    }
  }
}