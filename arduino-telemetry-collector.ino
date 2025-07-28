#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <Wire.h>

typedef bool bool8_t;
typedef byte byte8_t;

typedef int int16_t;
typedef long int32_t;
typedef float float32_t;

typedef byte uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;

static const char HEX_CHARS[] = "0123456789ABCDEF";

static const uint8_t ERROR_LED = 13; // Error indication only - uses built-in LED
static const uint32_t ERROR_LED_ON_MS = 250;
static const uint32_t ERROR_LED_OFF_MS = 500;

#define GPS_SERIAL Serial1
static const uint16_t GPS_FIX_WAIT_MS = 120000;
static const uint16_t GPS_TIME_WAIT_MS = 30000;

static const int16_t SD_CS_PIN = 53;
static const int16_t MPU_ADDR = 0x68; // MPU-6500 I2C address (same as MPU-6050)

static const uint16_t BUFFER_SIZE_MAX = 4096;
static const uint32_t FILE_SIZE_MAX = 1024 * 1024;
static const uint16_t OBS_SIZE_MAX = 1 + 4 + 6 + 6 + 6 + 10;

static const char MAGIC_BYTES[4] = {'A', 'T', 'C', '\0'};
static const uint16_t VERSION = 0;

static const uint16_t LOOP_MS = 10;
static const uint16_t ACCEL_INTERVAL = 1;
static const uint16_t GYRO_INTERVAL = 1;
static const uint16_t MAGN_INTERVAL = 0;
static const uint16_t GPS_INTERVAL = 100;
static const uint16_t RESET_INTERVAL = 100;

// Sensor configuration. See docs for meaning of values.
static const byte8_t ATC_CFG_ACCL = 1;
static const byte8_t ATC_CFG_GYRO = 1;
static const byte8_t ATC_CFG_MAGN = 0;
static const byte8_t ATC_CFG_GPS = 1;

// Bitmap values to mark observation presence.
static const byte8_t ATC_ROW_ACCL = 1;
static const byte8_t ATC_ROW_GYRO = 2;
static const byte8_t ATC_ROW_MAGN = 4;
static const byte8_t ATC_ROW_GPS = 8;

// Bitmap values to mark observation missing values.
static const byte8_t ATC_NAN_ACCL = 16;
static const byte8_t ATC_NAN_GYRO = 32;
static const byte8_t ATC_NAN_MAGN = 64;
static const byte8_t ATC_NAN_GPS = 128;

TinyGPSPlus gps;
char *fileName;
uint32_t referenceEpochSeconds;
uint16_t referenceEpochMillis;
uint32_t referenceMillis;
byte8_t *bufferRootPtr;
byte8_t *bufferWritePtr;
uint32_t fileSize = 0;
uint32_t loopIteration = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Arduino Telemetry Collector Starting...");

  initializeErrorLed();

  while (!initializeMemory()) {
    blinkError(6);
  }

  while (!initializeMPU6050()) {
    blinkError(5);
  }

  while (!initializeSD()) {
    blinkError(4);
  }

  if (!initializeGPS()) {
    blinkError(3);
  }

  while (!initializeTime()) {
    blinkError(2);
  }

  Serial.println("Setup completed - entering main loop");
}

void loop() {
  uint32_t loopStart = millis();

  if (!fileSize && bufferWritePtr == bufferRootPtr) { // Start of a new file;
    // Get new reference timing
    while (!updateReferenceTime()) {
      blinkError(2);
      delay(LOOP_MS);
    }

    // Set filename
    setFileName(referenceEpochSeconds);

    // Set up header
    char *magicBytesPtr = (char *)bufferWritePtr;
    for (uint8_t i = 0; i < sizeof(MAGIC_BYTES); i++) {
      magicBytesPtr[i] = MAGIC_BYTES[i];
      bufferWritePtr++;
    }

    uint16_t *versionPtr = (uint16_t *)bufferWritePtr;
    versionPtr[0] = VERSION;
    bufferWritePtr += 2;

    *bufferWritePtr++ = ATC_CFG_ACCL;
    *bufferWritePtr++ = ATC_CFG_GYRO;
    *bufferWritePtr++ = ATC_CFG_MAGN;
    *bufferWritePtr++ = ATC_CFG_GPS;

    uint32_t *referenceEpochSecondsPtr = (uint32_t *)bufferWritePtr;
    referenceEpochSecondsPtr[0] = referenceEpochSeconds;
    bufferWritePtr += 4;

    uint16_t *referenceEpochMillisPtr = (uint16_t *)bufferWritePtr;
    referenceEpochMillisPtr[0] = referenceEpochMillis;
    bufferWritePtr += 2;
  }

  // Infer which fields are required;
  if (loopIteration == RESET_INTERVAL) {
    loopIteration = 1;
  } else {
    loopIteration++;
  }

  byte8_t *rowConfigPtr = bufferWritePtr++;    // We set it later when we know any missing data.
  bufferWritePtr += writeTime(bufferWritePtr); // writeTime always succeeds, advance pointer

  byte8_t rowSensors = 0;
  if (ACCEL_INTERVAL && loopIteration % ACCEL_INTERVAL == 0) {
    rowSensors |= ATC_ROW_ACCL;
    uint8_t bytesWritten = writeAccelerometer(bufferWritePtr);
    if (bytesWritten) {
      bufferWritePtr += bytesWritten;
    } else {
      rowSensors |= ATC_NAN_ACCL;
    }
  }
  if (GYRO_INTERVAL && loopIteration % GYRO_INTERVAL == 0) {
    rowSensors |= ATC_ROW_GYRO;
    uint8_t bytesWritten = writeGyroscope(bufferWritePtr);
    if (bytesWritten) {
      bufferWritePtr += bytesWritten;
    } else {
      rowSensors |= ATC_NAN_GYRO;
    }
  }
  if (MAGN_INTERVAL && loopIteration % MAGN_INTERVAL == 0) {
    rowSensors |= ATC_ROW_MAGN;
    uint8_t bytesWritten = writeMagnetometer(bufferWritePtr);
    if (bytesWritten) {
      bufferWritePtr += bytesWritten;
    } else {
      rowSensors |= ATC_NAN_MAGN;
    }
  }
  if (GPS_INTERVAL && loopIteration % GPS_INTERVAL == 0) {
    rowSensors |= ATC_ROW_GPS;
    uint8_t bytesWritten = writeGPS(bufferWritePtr);
    if (bytesWritten) {
      bufferWritePtr += bytesWritten;
    } else {
      rowSensors |= ATC_NAN_GPS;
    }
  }
  *rowConfigPtr = rowSensors; // Writing the row config now it is fully known

  // If buffer is nearly full or close to too big for the file, flush it
  uint32_t bufferSize = bufferWritePtr - bufferRootPtr;
  bool8_t bufferNearlyFull = bufferSize > BUFFER_SIZE_MAX - OBS_SIZE_MAX;
  bool8_t fileNearlyFull = fileSize + bufferSize > FILE_SIZE_MAX - OBS_SIZE_MAX;
  if (bufferNearlyFull || fileNearlyFull) {
    while (!writeToFile(bufferRootPtr, bufferSize, fileName)) {
      blinkError(4);
    }
    bufferWritePtr = bufferRootPtr;

    if (fileNearlyFull) {
      fileSize = 0; // Start new file
    } else {
      fileSize += bufferSize;
    }
  }

  // Process any new GPS data
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }

  // Wait to align with timing grid
  uint32_t loopTime = millis() - loopStart;
  if (LOOP_MS > loopTime) {
    // Under budget: wait for normal interval
    delay(LOOP_MS - loopTime);
    digitalWrite(ERROR_LED, LOW);
  } else if (LOOP_MS < loopTime) {
    // Over budget: wait to next timing grid alignment
    digitalWrite(ERROR_LED, HIGH);
    uint32_t overrun = loopTime - LOOP_MS;
    uint32_t nextSlot = ((overrun / LOOP_MS) + 1) * LOOP_MS;
    uint32_t delayNeeded = nextSlot - overrun;
    delay(delayNeeded);
  } else {
    // Exactly on time
    digitalWrite(ERROR_LED, LOW);
  }
}

void blinkError(int16_t blink_count) {
  /// Blink the error LED a specified number of times.
  ///
  /// \param blink_count The number of times to blink the LED.
  Serial.print("Error - blinking ");
  Serial.print(blink_count);
  Serial.println(" times");

  for (int16_t i = 0; i < blink_count; i++) {
    digitalWrite(ERROR_LED, HIGH);
    delay(ERROR_LED_ON_MS);
    digitalWrite(ERROR_LED, LOW);
    delay(ERROR_LED_OFF_MS);
  }
  delay(ERROR_LED_ON_MS + ERROR_LED_OFF_MS);
}

bool8_t initializeMemory() {
  fileName = (char *)malloc(13 * sizeof(char));
  if (fileName == NULL) {
    Serial.println("Failed to allocate memory for filename");
    return false;
  }
  fileName[8] = '.';
  fileName[9] = 'A';
  fileName[10] = 'T';
  fileName[11] = 'C';
  fileName[12] = '\0';

  bufferRootPtr = (byte8_t *)malloc(BUFFER_SIZE_MAX);
  if (bufferRootPtr == NULL) {
    Serial.println("Failed to allocate memory for byte buffer");
    return false;
  }
  bufferWritePtr = bufferRootPtr;
  return true;
}

bool8_t initializeErrorLed() {
  pinMode(ERROR_LED, OUTPUT);
  digitalWrite(ERROR_LED, LOW);

  return true;
}

bool8_t initializeGPS() {
  Serial.println("Initializing GPS...");

  // Initialize GPS module
  GPS_SERIAL.begin(9600);

  uint32_t initialTime = millis();
  while (millis() - initialTime <= GPS_FIX_WAIT_MS) {
    if (GPS_SERIAL.available()) {
      gps.encode(GPS_SERIAL.read());
      if (gps.location.isUpdated() && gps.location.isValid()) {
        return true;
      }
    }
  }

  Serial.println("GPS sensor initialization failed");
  return false;
}

bool8_t initializeTime() {
  Serial.println("Initializing Time...");

  uint32_t initialTime = millis();
  while (millis() - initialTime <= GPS_TIME_WAIT_MS) {
    if (GPS_SERIAL.available()) {
      gps.encode(GPS_SERIAL.read());
      if (gps.time.isUpdated() && gps.time.isValid() && updateReferenceTime()) {
        return true;
      }
    }
  }

  Serial.println("Time initialization failed");
  return false;
}

bool8_t initializeSD() {
  Serial.println("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed");
    return false;
  }

  Serial.println("SD card initialized successfully");
  return true;
}

bool8_t initializeMPU6050() {
  Serial.println("Initializing MPU sensor...");
  Wire.begin();
  Wire.setClock(100000); // Set I2C clock to 100kHz (standard speed)

  // Wake up MPU sensor
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up
  Wire.endTransmission(true);
  delay(100);

  // Configure sample rate to match 100Hz polling
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19); // SMPLRT_DIV register
  Wire.write(0x09); // Sample rate = 1kHz / (1 + 9) = 100Hz (matches loop frequency)
  Wire.endTransmission(true);

  // Configure DLPF for stable readings
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); // CONFIG register
  Wire.write(0x03); // DLPF_CFG = 3 (44Hz cutoff, good for automotive)
  Wire.endTransmission(true);

  // Configure accelerometer range (optional - ±2g default)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x00); // ±2g range
  Wire.endTransmission(true);

  // Configure gyroscope range (optional - ±250°/s default)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x00); // ±250°/s range
  Wire.endTransmission(true);

  // Allow time for configuration to settle
  delay(100);

  // Test read
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);

  if (Wire.available()) {
    uint8_t whoAmI = Wire.read();
    if (whoAmI == 0x68 || whoAmI == 0x70 || whoAmI == 0x71) {
      Serial.println("MPU sensor initialized successfully");
      return true;
    }
  }

  Serial.println("MPU sensor initialization failed");
  return false;
}

inline uint8_t writeTime(byte8_t *writePtr) {
  // No overrun check, assuming we are never going to get close
  uint32_t *timePtr = (uint32_t *)writePtr;
  timePtr[0] = millis() - referenceMillis;

  return 4; // Always succeeds, returns bytes written
}

inline uint8_t writeAccelerometer(byte8_t *writePtr) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  uint8_t received = Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t *castPtr = (int16_t *)writePtr;
  if (received == 6) {
    for (uint8_t i = 0; i < 3; i++) {
      castPtr[i] = Wire.read() << 8 | Wire.read();
    }
    return 6; // Success: return bytes written
  }
  return 0; // Failure: return 0 bytes written
}

inline uint8_t writeGyroscope(byte8_t *writePtr) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // GYRO_XOUT_H
  Wire.endTransmission(false);
  uint8_t received = Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t *castPtr = (int16_t *)writePtr;
  if (received == 6) {
    for (uint8_t i = 0; i < 3; i++) {
      castPtr[i] = Wire.read() << 8 | Wire.read();
    }
    return 6; // Success: return bytes written
  }
  return 0; // Failure: return 0 bytes written
}

inline uint8_t writeMagnetometer(byte8_t *writePtr) {
  return 0; // Always fail: return 0 bytes written
}

inline uint8_t writeGPS(byte8_t *writePtr) {
  if (!gps.location.isValid()) {
    return 0; // Failure: return 0 bytes written
  }

  float32_t *castPtr = (float32_t *)writePtr;
  castPtr[0] = gps.location.lng();
  castPtr[1] = gps.location.lat();

  // Optimize HDOP: single library call + safe integer math instead of ceil()
  if (gps.hdop.isValid()) {
    uint32_t hdopValue = gps.hdop.value(); // Single call, store result
    if (hdopValue >= 25500) {
      writePtr[8] = 255;
    } else {
      writePtr[8] = (uint8_t)ceil(hdopValue / 100.0);
    }
  } else {
    writePtr[8] = 0;
  }

  // Optimize satellites: single library call
  if (gps.satellites.isValid()) {
    uint8_t satValue = gps.satellites.value(); // Single call, store result
    writePtr[9] = (satValue >= 255) ? 255 : satValue;
  } else {
    writePtr[9] = 0;
  }

  return 10; // Success: return bytes written
}

bool8_t updateReferenceTime() {
  if (gps.time.isValid()) {
    tmElements_t tm;
    tm.Year = gps.date.year() - 1970; // Time library counts from 1970
    tm.Month = gps.date.month();
    tm.Day = gps.date.day();
    tm.Hour = gps.time.hour();
    tm.Minute = gps.time.minute();
    tm.Second = gps.time.second();

    referenceEpochSeconds = makeTime(tm);
    referenceEpochMillis = gps.time.centisecond() * 10;
    referenceMillis = millis();
    return referenceEpochSeconds > 1735689600; // Timestamps lower than this suggest date/time is incorrect.
  }
  return false;
}

inline bool8_t writeToFile(byte8_t *buffer, uint16_t size, char *fName) {
  File fHandle = SD.open(fName, FILE_WRITE);
  if (!fHandle) {
    Serial.println("Could not open SD card file.");
    blinkError(1);
    return false; // Fail silently during runtime
  }

  fHandle.write(buffer, size);
  fHandle.close();

  return true;
}

inline void *setFileName(uint32_t epoch) {
  for (int16_t i = 7; i >= 0; i--) {
    fileName[7 - i] = HEX_CHARS[(epoch >> (i * 4)) & 0x0F];
  }
}
