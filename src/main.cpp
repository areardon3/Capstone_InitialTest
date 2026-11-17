#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// ---------- Pin configuration ----------
const int SD_CS = D10;
const int FP_PIN = A1; 
const int BTN_ZERO_PIN = A6;   // zero both angles
const int BTN_LOG_PIN  = A7;   // start/stop logging
const int SENSOR2_SDA = D4;
const int SENSOR2_SCL = D5;
const int relayPin = D6; // FES control 

// AS5600 I2C address and angle register
const uint8_t AS5600_ADDR = 0x36;
const uint8_t RAW_ANGLE_REG = 0x0E;

// ---------- Timing configuration ----------
const uint32_t SAMPLE_INTERVAL_US = 10000;  // 100 Hz
const uint32_t PRINT_INTERVAL_US = 500000;  // print every 0.5 s
const int FLUSH_SAMPLES = 50;

// ---------- Force plate (use your calibrated values) ----------
const int FP_ON_MV  = 2000;
const int FP_OFF_MV = 1800;
const bool FP_SHOW_MV = true;

File logFile;
char logName[24];

char logBuffer[4096];
size_t bufferUsed = 0;
int samplesBuffered = 0;

bool logging = false;
bool haveRaw = false;

uint16_t zeroOffset1 = 0, zeroOffset2 = 0;
uint16_t lastRaw1 = 0, lastRaw2 = 0;

bool fpTrigger = false;
int fpMv = 0;

uint32_t logStartUs = 0;
uint32_t nextSampleUs = 0;
uint32_t lastPrintUs = 0;

// Read the 12-bit angle from an AS5600 on the given I2C bus
bool readAS5600(TwoWire &bus, uint16_t &rawAngle) {
  bus.beginTransmission(AS5600_ADDR);
  bus.write(RAW_ANGLE_REG);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return false;

  uint8_t highByte = bus.read();
  uint8_t lowByte = bus.read();
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

  logFile.println("time_us,raw1,angle1_deg,raw2,angle2_deg,fp_mv,fp_trigger");
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

void zeroAngles() {
  if (logging) {
    Serial.println("Stop logging before zeroing.");
  } else if (haveRaw) {
    zeroOffset1 = lastRaw1;
    zeroOffset2 = lastRaw2;
    Serial.println("Zeroed both sensors.");
  }
}

void handleButtons() {
  // Read the real pin state on first call so a button already low at boot
  // doesn't count as a press
  static bool lastZero = digitalRead(BTN_ZERO_PIN);
  static bool lastLog  = digitalRead(BTN_LOG_PIN);
  static uint32_t lastPressMs = 0;

  bool zeroNow = digitalRead(BTN_ZERO_PIN);
  bool logNow  = digitalRead(BTN_LOG_PIN);

  if (millis() - lastPressMs > 300) {  // lockout between presses
    if (lastZero == HIGH && zeroNow == LOW) {
      lastPressMs = millis();
      zeroAngles();
    }
    if (lastLog == HIGH && logNow == LOW) {
      lastPressMs = millis();
      if (logging) stopLogging();
      else startLogging();
    }
  }

  lastZero = zeroNow;
  lastLog = logNow;
}

void handleSerial() {
  // Keyboard commands from the serial monitor
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'z':
      case 'Z':
        zeroAngles();
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
        break;
    }
  }

  handleButtons();
}

void setup() {
  Serial.begin(115200);

  while (!Serial && millis() < 5000) delay(10);

  pinMode(BTN_ZERO_PIN, INPUT_PULLUP);
  pinMode(BTN_LOG_PIN, INPUT_PULLUP);
  pinMode(FP_PIN, INPUT);

  // Sensor 1: default I2C bus (A4 = SDA, A5 = SCL)
  Wire.begin();
  Wire.setClock(400000);

  // Sensor 2: second I2C bus on D4 (SDA) and D5 (SCL)
  Wire1.begin(SENSOR2_SDA, SENSOR2_SCL, 400000);

  if (!SD.begin(SD_CS)) halt("SD card initialization failed!");

  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() != 0) halt("AS5600 #1 (A4/A5) not detected!");

  Wire1.beginTransmission(AS5600_ADDR);
  if (Wire1.endTransmission() != 0) halt("AS5600 #2 (D4/D5) not detected!");

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
 
  Serial.begin(115200);
 
  Serial.println("Relay controller initialized.");

  Serial.println("Ready.");
  Serial.println("  z / A6 = zero angles");
  Serial.println("  s / A7 = start logging, x / A7 = stop logging");

  nextSampleUs = micros();
}

void loop() {
  handleSerial();

  uint32_t now = micros();
  if ((int32_t)(now - nextSampleUs) < 0) return;
  nextSampleUs = now + SAMPLE_INTERVAL_US;

  uint16_t raw1, raw2;
  if (!readAS5600(Wire, raw1)) {
    Serial.println("Sensor 1 read error!");
    return;
  }
  if (!readAS5600(Wire1, raw2)) {
    Serial.println("Sensor 2 read error!");
    return;
  }
  lastRaw1 = raw1;
  lastRaw2 = raw2;
  haveRaw = true;

  // Angles relative to each sensor's zero point, wrapped to 0-4095
  float angle1 = ((raw1 - zeroOffset1) & 0x0FFF) * (360.0f / 4096.0f);
  float angle2 = ((raw2 - zeroOffset2) & 0x0FFF) * (360.0f / 4096.0f);

  // Force plate trigger with hysteresis
  fpMv = analogReadMilliVolts(FP_PIN);
  if (!fpTrigger && fpMv >= FP_ON_MV) {
    fpTrigger = true;
  } else if (fpTrigger && fpMv <= FP_OFF_MV) {
    fpTrigger = false;
  }

  // Print to the serial monitor every 0.5 s
  if (now - lastPrintUs >= PRINT_INTERVAL_US) {
    lastPrintUs = now;
    Serial.printf("%s angle1=%.2f  angle2=%.2f deg  FP Trigger: %s",
                  logging ? "[LOG] " : "[idle]", angle1, angle2,
                  fpTrigger ? "True" : "False");
    if (FP_SHOW_MV) Serial.printf("  (FP=%d mV)", fpMv);
    Serial.println();
  }

  if (!logging) return;

  // Timestamp in microseconds since logging started
  uint32_t timestamp = micros() - logStartUs;

  char line[96];
  int len = snprintf(line, sizeof(line), "%lu,%u,%.3f,%u,%.3f,%d,%d\n",
                     (unsigned long)timestamp,
                     (unsigned)raw1, angle1,
                     (unsigned)raw2, angle2,
                     fpMv, fpTrigger ? 1 : 0);
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