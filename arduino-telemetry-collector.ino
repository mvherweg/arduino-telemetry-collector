#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <TimeLib.h>

// Pin definitions  
const int RED_LED = 13;  // Error indication only - uses built-in LED
const int SD_CS_PIN = 53;

// MPU-6500 I2C address (same as MPU-6050)
const int MPU_ADDR = 0x68;

// GPS Serial (using Serial1 on Mega)
#define GPS_SERIAL Serial1

// Configuration parameters
const unsigned int blink_on_duration = 500;
const unsigned int blink_off_duration = 500;
const unsigned int blink_pause_duration = 2000;
const unsigned int loop_duration = 50;
const unsigned int telemetry_interval = 1;   // Every 50ms = 20Hz data collection  
const unsigned int gps_interval = 20;        // Every 1 second (20 × 50ms)  
const unsigned int write_interval = 146;     // Keep same buffer size: 146 records × 28 bytes = 4,088 bytes ≈ 4KB block
const unsigned int gps_fix_duration = 30000;
const unsigned int gps_fix_tries = 4;


// State variables
unsigned long iteration_counter = 0;
unsigned long last_loop_time = 0;
unsigned long gps_epoch_time = 0;  // Unix timestamp from GPS
unsigned long gps_board_time = 0;  // millis() when GPS time was captured

// Loop timing monitoring
uint8_t last_loop_skipped = 0;  // Iterations skipped in last loop timing violation

// GPS variables
TinyGPSPlus gps;
float latitude = 0.0;
float longitude = 0.0;
bool gps_fix = false;
uint8_t gps_satellites = 0;
float gps_hdop = 99.9; // Horizontal dilution of precision

// MPU-6500 variables
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

// Compressed data buffer structure (28 bytes vs 45 bytes)
struct TelemetryData {
  uint16_t days_since_1970;          // 2 bytes (days since Unix epoch)
  uint32_t milliseconds_in_day;      // 4 bytes (1ms precision within day)
  int16_t accel_x, accel_y, accel_z;  // 6 bytes (16-bit compressed)
  int16_t gyro_x, gyro_y, gyro_z;     // 6 bytes (16-bit compressed)
  float latitude, longitude;         // 8 bytes (keep float for GPS precision)
  uint16_t gps_status_packed;        // 2 bytes (bit-packed GPS/timing data)
  // Bit layout: [15:11]=iterations_skipped, [10:6]=satellites, [5:1]=accuracy, [0]=has_gps
};

TelemetryData* data_buffer;
unsigned int buffer_index = 0;

void setup() {
  Serial.begin(9600);
  GPS_SERIAL.begin(9600);

  // Initialize pins
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);

  Serial.println("Arduino Telemetry Collector Starting...");

  // Run setup routine
  performSetup();
}

void loop() {
  unsigned long loop_start = millis();

  // Iteration start
  iteration_counter++;

  // Location and time acquisition
  if (iteration_counter % gps_interval == 0) {
    unsigned long gps_start = millis();
    unsigned long time_budget = loop_duration - (gps_start - loop_start);

    acquireGPSDataWithBudget(time_budget);
  }

  // Telemetry collection
  if (iteration_counter % telemetry_interval == 0) {
    collectTelemetry();
  }

  // Data writing
  if (iteration_counter % write_interval == 0) {
    writeDataToSD();
  }

  // Maintain timing
  maintainLoopTiming(loop_start);
}

void performSetup() {
  // Step 1: Configuration validation
  while (!validateConfiguration()) {
    blinkError(5);
  }

  // Allocate data buffer (only once)
  if (data_buffer != NULL) {
    free(data_buffer); // Free existing buffer if any
  }
  data_buffer = (TelemetryData*)malloc(write_interval * sizeof(TelemetryData));
  while (data_buffer == NULL) {
    Serial.println("Failed to allocate data buffer");
    blinkError(5);
    data_buffer = (TelemetryData*)malloc(write_interval * sizeof(TelemetryData));
  }
  buffer_index = 0;

  // Step 2: Initialize I2C and MPU sensor (only once)
  Wire.begin();
  while (!initializeMPU6050()) {
    blinkError(2);
  }

  // Step 3: Initialize SD card (only once)
  while (!initializeSD()) {
    blinkError(1);
  }

  // Step 4: Acquire GPS fix (with timeout and retry limits)
  bool gps_fix_acquired = false;
  for (unsigned int attempt = 0; attempt < gps_fix_tries || gps_fix_tries == 0; attempt++) {
    if (acquireGPSFix()) {
      gps_fix_acquired = true;
      break;
    }
    if (gps_fix_tries > 0) {
      Serial.print("GPS fix attempt ");
      Serial.print(attempt + 1);
      Serial.print(" of ");
      Serial.print(gps_fix_tries);
      Serial.println(" failed");
      
      // Wait 30 seconds before next attempt (except after last attempt)
      if (attempt + 1 < gps_fix_tries) {
        Serial.println("Waiting 30 seconds before next GPS attempt...");
        delay(30000);
      }
    }
    blinkError(4);
  }

  if (!gps_fix_acquired) {
    Serial.println("GPS fix acquisition failed after all attempts - continuing without location");
    gps_fix = false;
  }

  // Step 5: Acquire time (retry until success)
  while (!acquireTime()) {
    blinkError(3);
  }

  // Test SD card write (retry until success)
  while (!testSDWrite()) {
    blinkError(1);
  }

  iteration_counter = 0;
  last_loop_time = millis();
  Serial.println("Setup completed successfully. Starting main loop.");
}

bool validateConfiguration() {
  if (blink_on_duration < 100) return false;
  if (blink_off_duration < 100) return false;
  if (blink_pause_duration < 100) return false;
  if (loop_duration < 1) return false;
  if (telemetry_interval < 1) return false;
  if (gps_interval < 1) return false;
  if (gps_interval < telemetry_interval) return false;
  if ((gps_interval * loop_duration) < 200) return false;
  if (write_interval < 1) return false;
  if (write_interval < gps_interval) return false;
  // Check for filename collision risk: ensure we don't write more than once per second
  // Each file uses timestamp in seconds, so write_interval * loop_duration must be ≥ 1000ms
  if ((write_interval * loop_duration) < 1000) {
    Serial.println("Configuration error: risk of filename collisions");
    Serial.print("Write interval * loop duration = ");
    Serial.print(write_interval * loop_duration);
    Serial.println("ms (must be ≥ 1000ms)");
    return false;
  }
  if (gps_fix_duration < 5000) return false; // Minimum 5 seconds
  // gps_fix_tries can be any value (0 = infinite)

  Serial.println("Configuration validation passed");
  return true;
}

bool initializeMPU6050() {
  Serial.println("Initializing MPU sensor...");

  // Wake up MPU sensor
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up
  Wire.endTransmission(true);
  delay(100);

  // Configure sample rate (optional - default is fine for now)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19); // SMPLRT_DIV register
  Wire.write(0x07); // Sample rate = 1kHz / (1 + 7) = 125Hz
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

  // Test read
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);

  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    if (who_am_i == 0x68 || who_am_i == 0x70 || who_am_i == 0x71) {
      Serial.println("MPU sensor initialized successfully");
      return true;
    }
  }

  Serial.println("MPU sensor initialization failed");
  return false;
}

bool initializeSD() {
  Serial.println("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed");
    return false;
  }

  Serial.println("SD card initialized successfully");
  return true;
}

bool acquireGPSFix() {
  Serial.print("Acquiring GPS fix (timeout: ");
  Serial.print(gps_fix_duration / 1000);
  Serial.println(" seconds)...");
  unsigned long start_time = millis();
  int sentences_processed = 0;

  while (millis() - start_time < gps_fix_duration) {
    if (GPS_SERIAL.available()) {
      if (gps.encode(GPS_SERIAL.read())) {
        sentences_processed++;
        updateGPSData();
        
        if (gps_fix) {
          Serial.println("GPS fix acquired");
          return true;
        }
      }
    }
    delay(10); // Shorter delay for more responsive parsing
  }

  Serial.print("GPS fix acquisition failed (processed ");
  Serial.print(sentences_processed);
  Serial.println(" NMEA sentences)");
  return false;
}

bool acquireTime() {
  Serial.println("Acquiring time from GPS...");
  unsigned long start_time = millis();
  int sentences_processed = 0;

  while (millis() - start_time < 30000) { // 30 second timeout
    if (GPS_SERIAL.available()) {
      if (gps.encode(GPS_SERIAL.read())) {
        sentences_processed++;
        updateGPSData();
        
        // Debug output every 10 sentences
        if (sentences_processed % 10 == 0) {
          Serial.print("Processed ");
          Serial.print(sentences_processed);
          Serial.print(" sentences. Date valid: ");
          Serial.print(gps.date.isValid() ? "YES" : "NO");
          Serial.print(", Time valid: ");
          Serial.println(gps.time.isValid() ? "YES" : "NO");
        }
        
        if (gps_epoch_time > 0) {
          Serial.println("Time acquired from GPS");
          return true;
        }
      }
    }
    delay(10); // Shorter delay for more responsive parsing
  }

  Serial.print("Time acquisition failed after processing ");
  Serial.print(sentences_processed);
  Serial.println(" NMEA sentences");
  return false;
}

bool testSDWrite() {

  // Remove existing file and create new one
  if (SD.exists("setup.txt")) {
    SD.remove("setup.txt");
  }
  File test_file = SD.open("setup.txt", FILE_WRITE);
  if (!test_file) {
    Serial.println("Failed to open setup.txt for writing");
    Serial.println("Possible causes: SD card not inserted, bad connections, incompatible card");
    return false;
  }

  test_file.println(gps_epoch_time);
  test_file.close();

  // Read back to verify
  test_file = SD.open("setup.txt", FILE_READ);
  if (!test_file) {
    Serial.println("Failed to open setup.txt for reading");
    return false;
  }

  String read_data = test_file.readString();
  test_file.close();


  // Trim whitespace for comparison
  read_data.trim();
  if (read_data.toInt() == gps_epoch_time) {
    return true;
  }

  Serial.println("SD card write test failed - data mismatch");
  return false;
}

void collectTelemetry() {
  // Read accelerometer data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();


  // Read gyroscope data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // GYRO_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  // Convert to physical units
  accel_x = ax / 16384.0; // ±2g range
  accel_y = ay / 16384.0;
  accel_z = az / 16384.0;

  gyro_x = gx / 131.0; // ±250°/s range
  gyro_y = gy / 131.0;
  gyro_z = gz / 131.0;

  // Store in buffer with 16-bit compression
  if (buffer_index < write_interval) {
    // Use skipped count from last loop timing check
    uint8_t skipped_delta = last_loop_skipped;
    last_loop_skipped = 0;  // Reset for next iteration

    // Get current timestamp with 1ms precision
    getCurrentTimestamp(&data_buffer[buffer_index].days_since_1970, &data_buffer[buffer_index].milliseconds_in_day);
    
    data_buffer[buffer_index].accel_x = encodeAccel(accel_x);
    data_buffer[buffer_index].accel_y = encodeAccel(accel_y);
    data_buffer[buffer_index].accel_z = encodeAccel(accel_z);
    data_buffer[buffer_index].gyro_x = encodeGyro(gyro_x);
    data_buffer[buffer_index].gyro_y = encodeGyro(gyro_y);
    data_buffer[buffer_index].gyro_z = encodeGyro(gyro_z);
    data_buffer[buffer_index].latitude = latitude;
    data_buffer[buffer_index].longitude = longitude;
    // Pack GPS status with bit manipulation
    bool has_gps = (iteration_counter % gps_interval == 0);
    data_buffer[buffer_index].gps_status_packed = packGPSStatus(has_gps, gps_hdop, gps_satellites, skipped_delta);

    buffer_index++;
  }
}

void acquireGPSData() {
  while (GPS_SERIAL.available()) {
    if (gps.encode(GPS_SERIAL.read())) {
      updateGPSData();
    }
  }
}

void acquireGPSDataWithBudget(unsigned long time_budget_ms) {
  unsigned long start_time = millis();

  // Try to acquire GPS data within time budget
  while (millis() - start_time < time_budget_ms) {
    if (GPS_SERIAL.available()) {
      if (gps.encode(GPS_SERIAL.read())) {
        updateGPSData();
        if (gps_fix) {
          break; // Got valid fix, stop trying
        }
      }
    } else {
      delay(1); // Small delay when no data available
    }
  }
}

void updateGPSData() {
  // Update location data using TinyGPS++
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    gps_fix = true;
  } else {
    gps_fix = false;
  }
  
  // Update satellite count
  if (gps.satellites.isValid()) {
    uint32_t sat_count = gps.satellites.value();
    gps_satellites = (sat_count > 255) ? 255 : (uint8_t)sat_count;
  }
  
  // Update HDOP
  if (gps.hdop.isValid()) {
    gps_hdop = gps.hdop.hdop();
  }
  
  // Update time using TinyGPS++ and Time library
  if (gps.date.isValid() && gps.time.isValid()) {
    uint16_t year = gps.date.year();
    uint8_t month = gps.date.month();
    uint8_t day = gps.date.day();
    uint8_t hour = gps.time.hour();
    uint8_t minute = gps.time.minute();
    uint8_t second = gps.time.second();
    
    // Additional validation - TinyGPS++ isValid() is not reliable enough
    if (year >= 2020 && year <= 2030 && month >= 1 && month <= 12 && 
        day >= 1 && day <= 31 && hour <= 23 && minute <= 59 && second <= 59) {
      
      // Use Time library for clean epoch conversion
      tmElements_t tm;
      tm.Year = year - 1970;  // Time library counts from 1970
      tm.Month = month;
      tm.Day = day;
      tm.Hour = hour;
      tm.Minute = minute;
      tm.Second = second;
      
      gps_epoch_time = makeTime(tm);
      gps_board_time = millis();
    }
  }
}



void getCurrentTimestamp(uint16_t* days_since_1970, uint32_t* milliseconds_in_day) {
  unsigned long current_board_time = millis();
  unsigned long elapsed_since_gps = current_board_time - gps_board_time;
  
  // Calculate current Unix timestamp with millisecond precision
  unsigned long current_unix_seconds = gps_epoch_time + (elapsed_since_gps / 1000);
  unsigned long current_ms_in_second = elapsed_since_gps % 1000;
  
  // Calculate days since Unix epoch (Jan 1, 1970)
  *days_since_1970 = (uint16_t)(current_unix_seconds / 86400UL);  // 86400 seconds per day
  
  // Calculate milliseconds within the current day
  unsigned long seconds_in_day = current_unix_seconds % 86400UL;
  *milliseconds_in_day = (seconds_in_day * 1000UL) + current_ms_in_second;
}

void writeDataToSD() {
  // Generate filename once
  String filename = generateFilename();
  File data_file = SD.open(filename, FILE_WRITE);
  if (!data_file) return;  // Fail silently during runtime

  // Write entire buffer in one operation (much faster than loop)
  data_file.write((uint8_t*)data_buffer, buffer_index * sizeof(TelemetryData));
  
  data_file.close();
  buffer_index = 0; // Reset buffer
}

String generateFilename() {
  // Reconstruct Unix timestamp from split format
  unsigned long unix_seconds = (data_buffer[0].days_since_1970 * 86400UL) + (data_buffer[0].milliseconds_in_day / 1000UL);

  // Pre-allocate string to avoid reallocation during concatenation
  String filename;
  filename.reserve(12);  // 8 hex chars + ".ATC" = 12 characters
  
  // Convert to uppercase hex with leading zeros (optimized)
  const char hex_chars[] = "0123456789ABCDEF";
  for (int i = 7; i >= 0; i--) {
    filename += hex_chars[(unix_seconds >> (i * 4)) & 0x0F];
  }
  
  filename += ".ATC";
  return filename;
}




void maintainLoopTiming(unsigned long loop_start) {
  unsigned long loop_end = millis();
  unsigned long loop_time = loop_end - loop_start;

  if (loop_time < loop_duration) {
    delay(loop_duration - loop_time);
  } else if (loop_time > loop_duration) {
    // Count timing violations
    // (tracking iterations skipped instead of violations)

    // Skip iterations to catch up
    unsigned long iterations_to_skip = loop_time / loop_duration;
    iteration_counter += iterations_to_skip;
    
    // Store for next telemetry collection (clamp to uint8_t range)
    last_loop_skipped = (iterations_to_skip > 255) ? 255 : (uint8_t)iterations_to_skip;
    
  }

  last_loop_time = millis();
}


// 16-bit encoding functions for sensor data compression
int16_t encodeAccel(float accel_g) {
  // Scale: ±3.2768g range with 0.0001g precision
  float scaled = accel_g * 10000.0;
  if (scaled > 32767.0) scaled = 32767.0;
  if (scaled < -32768.0) scaled = -32768.0;
  return (int16_t)scaled;
}

int16_t encodeGyro(float gyro_deg_s) {
  // Scale: ±250°/s range with 0.0076°/s precision
  float scaled = gyro_deg_s * 131.0;
  if (scaled > 32767.0) scaled = 32767.0;
  if (scaled < -32768.0) scaled = -32768.0;
  return (int16_t)scaled;
}


// Bit packing function for GPS status
uint16_t packGPSStatus(bool has_gps, float hdop, uint8_t satellites, unsigned long iterations_skipped) {
  uint16_t packed = 0;
  
  // Bit 0: has_gps_data
  if (has_gps) {
    packed |= 0x0001;
  }
  
  // Bits 1-5: GPS accuracy (HDOP as integer, clamped to 0-31)
  uint8_t accuracy = (uint8_t)hdop;  // Integer precision only
  if (accuracy > 31) accuracy = 31;
  packed |= (accuracy & 0x1F) << 1;
  
  // Bits 6-10: GPS satellites (clamped to 0-31)
  uint8_t sats = satellites;
  if (sats > 31) sats = 31;
  packed |= (sats & 0x1F) << 6;
  
  // Bits 11-15: iterations skipped (clamped to 0-31)
  uint8_t skipped = (uint8_t)iterations_skipped;
  if (skipped > 31) skipped = 31;
  packed |= (skipped & 0x1F) << 11;
  
  return packed;
}

void blinkError(int blink_count) {
  Serial.print("Error - blinking ");
  Serial.print(blink_count);
  Serial.println(" times");

  for (int i = 0; i < blink_count; i++) {
    digitalWrite(RED_LED, HIGH);
    delay(blink_on_duration);
    digitalWrite(RED_LED, LOW);

    if (i < blink_count - 1) {
      delay(blink_off_duration);
    }
  }

  delay(blink_pause_duration);
}
