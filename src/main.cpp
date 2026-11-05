#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// ---------- Pin configuration ----------
const int SD_CS = D10;

// AS5600 I2C address and angle register
const uint8_t AS5600_ADDR = 0x36;
const uint8_t RAW_ANGLE_REG = 0x0E;

// ---------- Timing configuration ----------
const uint32_t SAMPLE_INTERVAL_US = 10000;  // 100 Hz
const uint32_t PRINT_INTERVAL_US = 500000;  // print every 0.5 s
const int FLUSH_SAMPLES = 50;

File logFile;
char logName[24];

char logBuffer[4096];
size_t bufferUsed = 0;
int samplesBuffered = 0;

bool logging = false;
uint16_t zeroOffset = 0;
uint16_t lastRaw = 0;
bool haveRaw = false;

uint32_t logStartUs = 0;
uint32_t nextSampleUs = 0;
uint32_t lastPrintUs = 0;

// Read the 12-bit angle from the AS5600
bool readAS5600(uint16_t &rawAngle) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_REG);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return false;

  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();
  rawAngle = (((uint16_t)highByte << 8) | lowByte) & 0x0FFF;
  return true;
}

// Write accumulated samples to the SD card
bool flushBuffer() {
  if (bufferUsed == 0) return true;

  size_t written = logFile.write((uint8_t *)logBuffer, bufferUsed);
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

bool startLogging() {
  // Pick the first unused filename: angle_001.csv, angle_002.csv, ...
  for (int i = 1; i < 1000; i++) {
    snprintf(logName, sizeof(logName), "/angle_%03d.csv", i);
    if (!SD.exists(logName)) break;
  }

  logFile = SD.open(logName, FILE_WRITE);
  if (!logFile) {
    Serial.println("Could not create log file!");
    return false;
  }

  logFile.println("time_us,raw_angle,angle_deg");
  logFile.flush();

  bufferUsed = 0;
  samplesBuffered = 0;
  logStartUs = micros();
  logging = true;
  Serial.printf("Logging started: %s\n", logName);
  return true;
}

void stopLogging() {
  flushBuffer();
  logFile.close();
  logging = false;
  Serial.printf("Logging stopped: %s\n", logName);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'z':
      case 'Z':
        if (logging) {
          Serial.println("Press x to stop logging before zeroing.");
        } else if (haveRaw) {
          zeroOffset = lastRaw;
          Serial.println("Zeroed.");
        }
        break;
      case 's':
      case 'S':
        if (logging) Serial.println("Already logging.");
        else startLogging();
        break;
      case 'x':
      case 'X':
        if (logging) stopLogging();
        else Serial.println("Not logging.");
        break;
      default:
        break;  // ignore newlines and anything else
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  if (!SD.begin(SD_CS)) halt("SD card initialization failed!");

  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() != 0) halt("AS5600 not detected!");

  Serial.println("Ready.");
  Serial.println("  z = zero angle");
  Serial.println("  s = start logging");
  Serial.println("  x = stop logging");

  nextSampleUs = micros();
}

void loop() {
  handleSerial();

  uint32_t now = micros();
  if ((int32_t)(now - nextSampleUs) < 0) return;
  nextSampleUs = now + SAMPLE_INTERVAL_US;

  uint16_t rawAngle;
  if (!readAS5600(rawAngle)) {
    Serial.println("AS5600 read error!");
    return;
  }
  lastRaw = rawAngle;
  haveRaw = true;

  // Angle relative to the zero point, wrapped to 0-4095
  uint16_t adjusted = (rawAngle - zeroOffset) & 0x0FFF;
  float angleDeg = adjusted * (360.0f / 4096.0f);

  // Print to the serial monitor every 0.5 s
  if (now - lastPrintUs >= PRINT_INTERVAL_US) {
    lastPrintUs = now;
    Serial.printf("%s angle=%.2f deg (raw=%u)\n",
                  logging ? "[LOG] " : "[idle]", angleDeg, rawAngle);
  }

  if (!logging) return;

  // Timestamp in microseconds since logging started
  uint32_t timestamp = micros() - logStartUs;

  char line[64];
  int len = snprintf(line, sizeof(line), "%lu,%u,%.3f\n",
                     (unsigned long)timestamp, rawAngle, angleDeg);
  if (len <= 0 || len >= (int)sizeof(line)) return;

  if (bufferUsed + len >= sizeof(logBuffer)) {
    if (!flushBuffer()) {
      logFile.close();
      logging = false;
      Serial.println("Logging stopped due to SD error.");
      return;
    }
  }

  memcpy(logBuffer + bufferUsed, line, len);
  bufferUsed += len;
  samplesBuffered++;

  if (samplesBuffered >= FLUSH_SAMPLES) {
    if (!flushBuffer()) {
      logFile.close();
      logging = false;
      Serial.println("Logging stopped due to SD error.");
    }
  }
}