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

const char HEX_CHARS[] = "0123456789ABCDEF";

static const uint8_t ERROR_LED = 13; // Error indication only - uses built-in LED
static const uint32_t ERROR_LED_ON_MS = 250;
static const uint32_t ERROR_LED_OFF_MS = 500;

static const uint16_t BUFFER_MARGIN_BYTES = 32; // Must be at least size of SeriesHeader + largest possible Observation
static const uint16_t BUNDLE_MARGIN_BYTES =
    BUFFER_MARGIN_BYTES + 11; // Must also cover the BundleHeader, SensorConfig and ReferenceTimstamp.
static const uint16_t BUFFER_SIZE_BYTES = 4096 + BUFFER_MARGIN_BYTES;

#define GPS_SERIAL Serial1
static const uint16_t GPS_FIX_WAIT_MS = 120000;

static const uint16_t GPS_TIME_WAIT_MS = 30000;
struct MicroTimestamp {
  uint32_t epoch;
  uint32_t micros;
};
MicroTimestamp *referenceTimestampPtr;

static const int16_t SD_CS_PIN = 53;
static const int16_t MPU_ADDR = 0x68; // MPU-6500 I2C address (same as MPU-6050)

#define OBS_SENSOR_ACCELEROMETER 8 // Bit 3 in 4-bit field
#define OBS_SENSOR_GYROSCOPE 4     // Bit 2 in 4-bit field
// Magnetometer currently not supported. Reserved for future extension.
#define OBS_SENSOR_MAGNETOMETER 2 // Bit 1 in 4-bit field
#define OBS_SENSOR_GPS 1          // Bit 0 in 4-bit field

static const uint32_t LOOP_DURATION_US = 10000; // 10ms = 100Hz
static const uint16_t GPS_INTERVAL = 100;       // Every 100 iterations = 1Hz GPS updates
static const uint32_t BUNDLE_SIZE_BYTES = 1024 * 1024;

TinyGPSPlus gps;
byte8_t *bundlePtr;       // Fixed start position.
byte8_t *writePtr;        // Current position.
byte8_t *seriesHeaderPtr; // Used to update seriesObsCount in SeriesHeader.
uint32_t bundleSize = 0;
uint16_t gpsCountdown = 0; // Start with a GPS observation.
uint8_t obsSensors = OBS_SENSOR_ACCELEROMETER | OBS_SENSOR_GYROSCOPE | OBS_SENSOR_GPS;
uint16_t seriesObsCount = 0;
char *fileName;
uint32_t loopStart;

void setup() {
  Serial.begin(9600);
  Serial.println("Arduino Telemetry Collector Starting...");

  while (!initializeConfiguration()) {
    blinkError(5);
  }

  if (!initializeGPS()) {
    blinkError(4);
  }

  while (!initializeTime()) {
    blinkError(3);
  }

  while (!initializeMPU6050()) {
    blinkError(2);
  }

  while (!initializeSD()) {
    blinkError(1);
  }

  Serial.println("Setup completed - entering main loop");
}

void loop() {
  // Debug: Confirm loop is running
  static uint32_t loop_count = 0;
  loop_count++;

  Serial.print("Loop start: ");
  Serial.println(loop_count);

  if (writePtr == bundlePtr) {
    Serial.println("First bundle - initializing...");
    Serial.println("Calling updateFileName...");
    updateFileName();
    Serial.println("Calling writeBundleHeader...");
    writePtr = writeBundleHeader(bundlePtr);
    Serial.println("Calling writeSensorConfig...");
    writePtr = writeSensorConfig(writePtr);
    Serial.println("Calling writeReferenceTimestamp...");
    writePtr = writeReferenceTimestamp(writePtr);
    seriesHeaderPtr = writePtr;
    Serial.println("Bundle initialization complete");
  }

  Serial.println("Starting GPS interval logic...");
  Serial.print("gpsCountdown=");
  Serial.print(gpsCountdown);
  Serial.print(", seriesObsCount before=");
  Serial.println(seriesObsCount);
  
  // TODO(michiel): bit clunky, might be simpler to build the bitmask every iteration and compare it with the last
  if (GPS_INTERVAL > 1) { // Logic only relevant if not every iteration contains GPS.
    if (!gpsCountdown) {  // Countdown over, need a GPS measurement.
      gpsCountdown = GPS_INTERVAL;
      writeSeriesHeader(seriesHeaderPtr, seriesObsCount, obsSensors); // Finishing the previous series.
      obsSensors |= OBS_SENSOR_GPS;
      seriesObsCount = 0;
    } else if (gpsCountdown == GPS_INTERVAL - 1) { // Last obs had GPS, this one not.
      writeSeriesHeader(seriesHeaderPtr, seriesObsCount, obsSensors);
      obsSensors &= (~OBS_SENSOR_GPS);
      seriesObsCount = 0;
    }
    gpsCountdown--;
  }

  Serial.println("Starting series header logic...");
  if (!seriesObsCount) {
    seriesHeaderPtr = writePtr;
    writePtr = writeSeriesHeader(writePtr, seriesObsCount, obsSensors);
  }

  Serial.println("Starting timing logic...");

  // TODO(michiel): at this point in the loop we need to delay to get regular interval
  uint32_t timePassed = micros() - loopStart;
  if (timePassed < LOOP_DURATION_US) {
    delayUs(LOOP_DURATION_US - timePassed);
  }
  loopStart = micros();

  Serial.println("Starting telemetry collection...");
  seriesObsCount++;
  writePtr = writeTimestampOffset(writePtr);

  // Debug output every 100 observations to avoid serial overload
  if (seriesObsCount % 100 == 0) {
    Serial.print("Loop status: obs=");
    Serial.print(seriesObsCount);
    Serial.print(", bufSize=");
    Serial.print(writePtr - bundlePtr);
    Serial.print(", sensors=");
    Serial.println(obsSensors);
  }
  if (obsSensors & OBS_SENSOR_ACCELEROMETER) {
    writePtr = writeAccelerometerReading(writePtr);
  }
  if (obsSensors & OBS_SENSOR_GYROSCOPE) {
    writePtr = writeGyroscopeReading(writePtr);
  }
  if (obsSensors & OBS_SENSOR_MAGNETOMETER) {
    writePtr = writeMagnetometerReading(writePtr);
  }
  if (obsSensors & OBS_SENSOR_GPS) {
    writePtr = writeGPSReading(writePtr);
  }

  uint32_t bufferSize = writePtr - bundlePtr;
  bool8_t seriesFull = seriesObsCount == 4095;
  bool8_t bufferNearlyFull = bufferSize > BUFFER_SIZE_BYTES - BUFFER_MARGIN_BYTES;
  bool8_t bundleNearlyFull = bundleSize + bufferSize > BUNDLE_SIZE_BYTES - BUNDLE_MARGIN_BYTES;
  bool8_t timerNearlyOverrun = 4294967295 - (micros() - referenceTimestampPtr->micros) < 2 * LOOP_DURATION_US;
  if (seriesFull || bufferNearlyFull || bundleNearlyFull || timerNearlyOverrun) {
    Serial.print("Flushing buffer: seriesObsCount=");
    Serial.print(seriesObsCount);
    Serial.print(", bufferSize=");
    Serial.println(bufferSize);

    writeSeriesHeader(seriesHeaderPtr, seriesObsCount, obsSensors); // Update observation count in SeriesHeader
    if (bundleSize) {
      bundleSize += dumpBuffer(seriesHeaderPtr);
    } else {
      bundleSize = dumpBuffer(bundlePtr);
    }
    seriesObsCount = 0;
  }

  if (bundleNearlyFull || timerNearlyOverrun) { // Bundle nearing its limit(s), start new bundle.
    bundleSize = 0;
    writePtr = bundlePtr;
  }

  // Process any new GPS data
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
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

void delayUs(uint32_t us) {
  if (us >= 16384) { // delayMicroseconds not reliable beyond this range
    uint32_t start = micros();
    delay((uint32_t)(us / 1000));
    uint32_t remainder = us % 1000;
    uint32_t usToWait = remainder - (micros() - start);
    if (remainder > usToWait) { // Else overrun happened: we are running behind.
      delayMicroseconds(usToWait);
    }
  } else {
    delayMicroseconds(us);
  }
}

bool8_t initializeConfiguration() {
  loopStart = micros();

  fileName = (char *)malloc(13 * sizeof(char));
  fileName[8] = '.';
  fileName[9] = 'A';
  fileName[10] = 'T';
  fileName[11] = 'C';
  fileName[12] = '\0';

  referenceTimestampPtr = (MicroTimestamp *)malloc(sizeof(MicroTimestamp));

  // Allocate bundle buffer space
  bundlePtr = (byte8_t *)malloc(BUFFER_SIZE_BYTES * sizeof(byte8_t));
  writePtr = bundlePtr;
  if (bundlePtr == NULL) {
    Serial.println("Failed to allocate memory for byte buffer");
    return false;
  }
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
      if (gps.time.isUpdated() && gps.time.isValid() && updateReferenceTimestamp()) {
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

inline byte8_t *writeBundleHeader(byte8_t *buffer) {
  /// Write the BundleHeader into a byte buffer.
  ///
  /// \param buffer The byte buffer location to write into.
  /// \return Pointer to the next byte after the BundleHeader.
  buffer[0] = 0;
  buffer[1] = 0;
  return buffer + 2;
}

inline byte8_t *writeSensorConfig(byte8_t *buffer) {
  // TODO(michiel): this is correct for the platform but means we have logic here and logic in the initialization
  buffer[0] = 0;
  buffer[1] = 0;
  return buffer + 2;
}

inline byte8_t *writeReferenceTimestamp(byte8_t *buffer) {
  updateReferenceTimestamp();
  uint32_t *castPtr = (uint32_t *)buffer;
  castPtr[0] = referenceTimestampPtr->epoch;
  return buffer + 4;
}

inline byte8_t *writeSeriesHeader(byte8_t *buffer, uint16_t seriesObsCount, uint8_t sensors) {
  // Assumes seriesObsCount value is valid.
  uint16_t *castPtr = (uint16_t *)buffer;
  castPtr[0] = (sensors << 12) + seriesObsCount;
  return buffer + 2;
}

inline byte8_t *writeTimestampOffset(byte8_t *buffer) {
  // No overrun check, assuming we are never going to get close
  uint32_t *castPtr = (uint32_t *)buffer;
  castPtr[0] = micros() - referenceTimestampPtr->micros;
  return buffer + 4;
}

inline byte8_t *writeAccelerometerReading(byte8_t *buffer) {
  /// Write the Accelerometer measurements into a byte buffer.
  ///
  /// \param buffer The byte buffer location to write into.
  /// \return Pointer to the next byte after the Accelerometermeasurements.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  uint8_t received = Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t *castPtr = (int16_t *)buffer;
  if (received == 6) {
    for (uint8_t i = 0; i < 3; i++) {
      castPtr[i] = Wire.read() << 8 | Wire.read();
    }
  } else {
    // Fill with zeros if I2C failed - this shouldn't happen often
    Serial.println("Accel I2C failed!");
    for (uint8_t i = 0; i < 3; i++) {
      castPtr[i] = 0;
    }
  }
  return buffer + 6;
}

inline byte8_t *writeGyroscopeReading(byte8_t *buffer) {
  /// Write the Gyroscope measurements into a byte buffer.
  ///
  /// \param buffer The byte buffer location to write into.
  /// \return Pointer to the next byte after the Gyroscope measurements.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // GYRO_XOUT_H
  Wire.endTransmission(false);
  uint8_t received = Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t *castPtr = (int16_t *)buffer;
  if (received == 6) {
    for (uint8_t i = 0; i < 3; i++) {
      castPtr[i] = Wire.read() << 8 | Wire.read();
    }
  } else {
    // Fill with zeros if I2C failed
    for (uint8_t i = 0; i < 3; i++) {
      castPtr[i] = 0;
    }
  }
  return buffer + 6;
}

inline byte8_t *writeMagnetometerReading(byte8_t *buffer) {
  /// Write the Magnetometer measurements into a byte buffer.
  ///
  /// \param buffer The byte buffer location to write into.
  /// \return Pointer to the next byte after the Magnetometermeasurements.

  int16_t *castPtr = (int16_t *)buffer;
  for (uint8_t i = 0; i < 3; i++) {
    castPtr[i] = 0; // Hardware does not contain Magnetometer.
  }
  return buffer + 6;
}

inline byte8_t *writeGPSReading(byte8_t *buffer) {
  if (gps.location.isValid()) {
    float32_t *castPtr = (float32_t *)buffer;
    castPtr[0] = gps.location.lng();
    castPtr[1] = gps.location.lat();
  } else {
    for (uint8_t i = 0; i < 8; i++) {
      buffer[i] = 0;
    }
  }

  if (gps.hdop.isValid()) {
    if (gps.hdop.value() >= 25500) {
      buffer[8] = 255;
    } else {
      buffer[8] = (uint8_t)ceil(gps.hdop.value() / 100.0);
    }
  } else {
    buffer[8] = 0;
  }

  if (gps.satellites.isValid()) {
    if (gps.satellites.value() >= 255) {
      buffer[9] = 255;
    } else {
      buffer[9] = (uint8_t)gps.satellites.value();
    }
  } else {
    buffer[9] = 0;
  }

  return buffer + 10;
}

bool8_t updateReferenceTimestamp() {
  if (gps.time.isValid()) {
    tmElements_t tm;
    tm.Year = gps.date.year() - 1970; // Time library counts from 1970
    tm.Month = gps.date.month();
    tm.Day = gps.date.day();
    tm.Hour = gps.time.hour();
    tm.Minute = gps.time.minute();
    tm.Second = gps.time.second();

    referenceTimestampPtr->epoch = makeTime(tm);
    referenceTimestampPtr->micros = micros() - ((uint32_t)gps.time.centisecond() * 10000);
    return referenceTimestampPtr->epoch > 1735689600; // Timestamps lower than this suggest date/time is incorrect.
  }
  return false;
}

uint32_t dumpBuffer(byte8_t *firstBytePtr) {
  File dataFile = SD.open(fileName, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Could not open SD card file.");
    blinkError(1);
    return 0; // Fail silently during runtime
  }

  uint32_t writeSize = (writePtr - firstBytePtr) * sizeof(byte8_t);
  dataFile.write(firstBytePtr, writeSize);
  dataFile.close();

  return writeSize;
}

inline void *updateFileName() {
  for (int16_t i = 7; i >= 0; i--) {
    fileName[i] = HEX_CHARS[(referenceTimestampPtr->epoch >> (i * 4)) & 0x0F];
  }
}
