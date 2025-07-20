#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>

// Pin definitions
const int YELLOW_LED = 2;
const int WHITE_LED = 3;
const int BLUE_LED = 4;
const int GREEN_LED = 5;
const int BUTTON_PIN = 6;
const int RED_LED = 7;
const int SD_CS_PIN = 53;

// MPU-6500 I2C address (same as MPU-6050)
const int MPU_ADDR = 0x68;

// GPS Serial (using Serial1 on Mega)
#define GPS_SERIAL Serial1

// Configuration parameters
unsigned long blink_on_duration = 500;
unsigned long blink_off_duration = 500;
unsigned long blink_pause_duration = 2000;
unsigned long loop_duration = 100;
unsigned int telemetry_interval = 1;
unsigned int gps_interval = 10;
unsigned int write_interval = 10;
String write_prefix = "ATC_BMW";
unsigned long gps_fix_duration = 30000;
unsigned int gps_fix_tries = 2;

// State variables
bool running = false;
unsigned long iteration_counter = 0;
unsigned long last_loop_time = 0;
unsigned long gps_epoch_time = 0;
unsigned long gps_board_time = 0;
bool button_pressed = false;
bool last_button_state = HIGH;

// Loop timing monitoring  
unsigned long total_iterations_skipped = 0;
unsigned long last_skipped_count = 0;

// GPS variables
float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;
bool gps_fix = false;
uint8_t gps_satellites = 0;
float gps_hdop = 99.9; // Horizontal dilution of precision

// MPU-6500 variables
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float temperature;

// Compressed data buffer structure (32 bytes vs 45 bytes)
struct TelemetryData {
  unsigned long timestamp;           // 4 bytes
  int16_t accel_x, accel_y, accel_z;  // 6 bytes (16-bit compressed)
  int16_t gyro_x, gyro_y, gyro_z;     // 6 bytes (16-bit compressed)
  int16_t temperature;               // 2 bytes (16-bit compressed)
  float latitude, longitude, altitude; // 12 bytes (keep float for GPS precision)
  uint16_t gps_status_packed;        // 2 bytes (bit-packed GPS/timing data)
  // Bit layout: [15:11]=iterations_skipped, [10:6]=satellites, [5:1]=accuracy, [0]=has_gps
};

TelemetryData* data_buffer;
unsigned int buffer_index = 0;

void setup() {
  Serial.begin(9600);
  GPS_SERIAL.begin(9600);

  // Initialize pins
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(WHITE_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(RED_LED, OUTPUT);

  // Turn off all LEDs initially
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(WHITE_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);

  Serial.println("Arduino Telemetry Collector Starting...");

  // Run setup routine
  performSetup();
}

void loop() {
  if (running) {
    unsigned long loop_start = millis();

    // Check button press
    checkButton();
    if (!running) return; // Exit if stopped

    // Iteration start
    iteration_counter++;
    turnOffLEDs();

    // Location and time acquisition
    if (iteration_counter % gps_interval == 0) {
      unsigned long gps_start = millis();
      unsigned long time_budget = loop_duration - (gps_start - loop_start);

      acquireGPSDataWithBudget(time_budget);
      digitalWrite(BLUE_LED, HIGH);
    }

    // Time tracking and LED updates
    updateTimeAndLEDs();

    // Telemetry collection
    if (iteration_counter % telemetry_interval == 0) {
      collectTelemetry();
      digitalWrite(GREEN_LED, HIGH);
    }

    // Data writing
    if (iteration_counter % write_interval == 0) {
      writeDataToSD();
    }

    // Maintain timing
    maintainLoopTiming(loop_start);
  } else {
    // Not running, just check button
    checkButton();
    delay(100); // Avoid busy waiting
  }
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

  running = true;
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
  if (write_prefix.length() < 1 || write_prefix.length() > 8) return false;
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
      Serial.print("WHO_AM_I: 0x");
      Serial.println(who_am_i, HEX);
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
  int sentences_received = 0;

  while (millis() - start_time < gps_fix_duration) {
    if (GPS_SERIAL.available()) {
      String gps_data = GPS_SERIAL.readStringUntil('\n');
      sentences_received++;

      // Only show first few sentences for debugging
      if (sentences_received <= 3) {
        Serial.print("GPS data: ");
        Serial.println(gps_data);
      }

      if (parseGPSData(gps_data)) {
        if (gps_fix) {
          Serial.println("GPS fix acquired");
          return true;
        }
      }
    }
    delay(100);
  }

  Serial.print("GPS fix acquisition failed (received ");
  Serial.print(sentences_received);
  Serial.println(" NMEA sentences)");
  return false;
}

bool acquireTime() {
  Serial.println("Acquiring time from GPS...");
  unsigned long start_time = millis();

  while (millis() - start_time < 30000) { // 30 second timeout
    if (GPS_SERIAL.available()) {
      String gps_data = GPS_SERIAL.readStringUntil('\n');
      if (parseGPSTime(gps_data)) {
        gps_board_time = millis();
        Serial.print("Time acquired: ");
        Serial.println(gps_epoch_time);
        return true;
      }
    }
    delay(100);
  }

  Serial.println("Time acquisition failed");
  return false;
}

bool testSDWrite() {
  Serial.println("Testing SD card write...");
  Serial.print("Writing timestamp: ");
  Serial.println(gps_epoch_time);

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
  Serial.println("Write operation completed");

  // Read back to verify
  test_file = SD.open("setup.txt", FILE_READ);
  if (!test_file) {
    Serial.println("Failed to open setup.txt for reading");
    return false;
  }

  String read_data = test_file.readString();
  test_file.close();

  Serial.print("Read back data: '");
  Serial.print(read_data);
  Serial.println("'");
  Serial.print("Expected: ");
  Serial.println(gps_epoch_time);

  // Trim whitespace for comparison
  read_data.trim();
  if (read_data.toInt() == gps_epoch_time) {
    Serial.println("SD card write test passed");
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

  // Read temperature data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x41); // TEMP_OUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  int16_t temp_raw = Wire.read() << 8 | Wire.read();

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

  temperature = (temp_raw / 340.0) + 36.53; // Convert to Celsius

  gyro_x = gx / 131.0; // ±250°/s range
  gyro_y = gy / 131.0;
  gyro_z = gz / 131.0;

  // Store in buffer with 16-bit compression
  if (buffer_index < write_interval) {
    // Calculate iterations skipped delta since last record
    unsigned long skipped_delta = total_iterations_skipped - last_skipped_count;
    last_skipped_count = total_iterations_skipped;

    data_buffer[buffer_index].timestamp = getCurrentEpochTime();
    data_buffer[buffer_index].accel_x = encodeAccel(accel_x);
    data_buffer[buffer_index].accel_y = encodeAccel(accel_y);
    data_buffer[buffer_index].accel_z = encodeAccel(accel_z);
    data_buffer[buffer_index].gyro_x = encodeGyro(gyro_x);
    data_buffer[buffer_index].gyro_y = encodeGyro(gyro_y);
    data_buffer[buffer_index].gyro_z = encodeGyro(gyro_z);
    data_buffer[buffer_index].temperature = encodeTemperature(temperature);
    data_buffer[buffer_index].latitude = latitude;
    data_buffer[buffer_index].longitude = longitude;
    data_buffer[buffer_index].altitude = altitude;
    // Pack GPS status with bit manipulation
    bool has_gps = (iteration_counter % gps_interval == 0);
    data_buffer[buffer_index].gps_status_packed = packGPSStatus(has_gps, gps_hdop, gps_satellites, skipped_delta);

    buffer_index++;
  }
}

void acquireGPSData() {
  if (GPS_SERIAL.available()) {
    String gps_data = GPS_SERIAL.readStringUntil('\n');
    parseGPSData(gps_data);
    parseGPSTime(gps_data);
  }
}

void acquireGPSDataWithBudget(unsigned long time_budget_ms) {
  unsigned long start_time = millis();

  // If we don't have a GPS fix, try to acquire one within time budget
  if (!gps_fix && time_budget_ms > 50) { // Need at least 50ms to try
    while (millis() - start_time < time_budget_ms) {
      if (GPS_SERIAL.available()) {
        String gps_data = GPS_SERIAL.readStringUntil('\n');
        parseGPSData(gps_data);
        parseGPSTime(gps_data);

        if (gps_fix) {
          break; // Got fix, stop trying
        }
      }
      delay(10); // Small delay to avoid busy waiting
    }
  } else {
    // Already have fix or no time budget, just update normally
    acquireGPSData();
  }
}

bool parseGPSData(String data) {
  if (data.startsWith("$GPGGA") || data.startsWith("$GNGGA")) {
    // Parse NMEA GGA sentence for position and satellite count
    int comma_positions[14];
    int comma_count = 0;

    for (int i = 0; i < data.length() && comma_count < 14; i++) {
      if (data.charAt(i) == ',') {
        comma_positions[comma_count++] = i;
      }
    }

    if (comma_count >= 8) {
      String lat_str = data.substring(comma_positions[1] + 1, comma_positions[2]);
      String lon_str = data.substring(comma_positions[3] + 1, comma_positions[4]);
      String fix_str = data.substring(comma_positions[5] + 1, comma_positions[6]);
      String sat_str = data.substring(comma_positions[6] + 1, comma_positions[7]);
      String hdop_str = data.substring(comma_positions[7] + 1, comma_positions[8]);
      String alt_str = data.substring(comma_positions[8] + 1, comma_positions[9]);

      // Update satellite count (field 7 in GGA sentence)
      if (sat_str.length() > 0) {
        int sat_count = sat_str.toInt();
        gps_satellites = (sat_count > 255) ? 255 : (uint8_t)sat_count;
      }
      
      // Update HDOP (field 8 in GGA sentence)
      if (hdop_str.length() > 0) {
        gps_hdop = hdop_str.toFloat();
      }

      if (fix_str.toInt() > 0 && lat_str.length() > 0 && lon_str.length() > 0) {
        latitude = convertNMEACoordinate(lat_str);
        longitude = convertNMEACoordinate(lon_str);
        altitude = alt_str.toFloat();
        gps_fix = true;
        return true;
      }
    }
  }
  return false;
}

bool parseGPSTime(String data) {
  if (data.startsWith("$GPRMC") || data.startsWith("$GNRMC")) {
    // Parse NMEA RMC sentence for time
    int comma_positions[12];
    int comma_count = 0;

    for (int i = 0; i < data.length() && comma_count < 12; i++) {
      if (data.charAt(i) == ',') {
        comma_positions[comma_count++] = i;
      }
    }

    if (comma_count >= 9) {
      String time_str = data.substring(comma_positions[0] + 1, comma_positions[1]);
      String date_str = data.substring(comma_positions[8] + 1, comma_positions[9]);

      if (time_str.length() >= 6 && date_str.length() >= 6) {
        // Convert to epoch time (simplified)
        gps_epoch_time = convertGPSTimeToEpoch(time_str, date_str);
        return true;
      }
    }
  }
  return false;
}

float convertNMEACoordinate(String coord) {
  if (coord.length() < 7) return 0.0; // Minimum: DDMM.MM

  // NMEA format: DDMM.MMMM or DDDMM.MMMM
  // Find decimal point
  int decimal_pos = coord.indexOf('.');
  if (decimal_pos < 3) return 0.0; // Invalid format

  // Extract degrees (everything except last 2 digits before decimal)
  float degrees = coord.substring(0, decimal_pos - 2).toFloat();

  // Extract minutes (last 2 digits before decimal + decimal part)
  float minutes = coord.substring(decimal_pos - 2).toFloat();

  return degrees + minutes / 60.0;
}

unsigned long convertGPSTimeToEpoch(String time_str, String date_str) {
  // Parse HHMMSS.SS format (e.g., "172741.00")
  if (time_str.length() < 6) return 0;

  int hours = time_str.substring(0, 2).toInt();
  int minutes = time_str.substring(2, 4).toInt();
  int seconds = time_str.substring(4, 6).toInt();

  // Parse DDMMYY format (e.g., "200725" = July 25, 2020)
  if (date_str.length() < 6) return 0;

  int day = date_str.substring(0, 2).toInt();
  int month = date_str.substring(2, 4).toInt();
  int year = 2000 + date_str.substring(4, 6).toInt(); // Y2K handling for 2000-2099

  // Calculate days since Unix epoch (Jan 1, 1970)
  unsigned long days = 0;

  // Add days for complete years
  for (int y = 1970; y < year; y++) {
    if ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0)) {
      days += 366; // Leap year
    } else {
      days += 365;
    }
  }

  // Days in each month (non-leap year)
  int monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  // Adjust February for leap year
  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
    monthDays[1] = 29;
  }

  // Add days for complete months in current year
  for (int m = 1; m < month; m++) {
    days += monthDays[m - 1];
  }

  // Add remaining days
  days += day - 1; // Subtract 1 because day 1 = 0 days elapsed

  // Convert to seconds and add time
  unsigned long epoch = days * 86400UL; // 24 * 60 * 60
  epoch += hours * 3600UL;
  epoch += minutes * 60UL;
  epoch += seconds;

  return epoch;
}

unsigned long getCurrentEpochTime() {
  unsigned long current_board_time = millis();
  unsigned long elapsed_since_gps = current_board_time - gps_board_time;
  return gps_epoch_time + (elapsed_since_gps / 1000);
}

void writeDataToSD() {
  if (buffer_index == 0) return;

  String filename = generateFilename();
  File data_file = SD.open(filename, FILE_WRITE);
  if (!data_file) {
    Serial.print("Failed to open file: ");
    Serial.println(filename);
    Serial.println("Possible causes: filename too long, SD card full, or corrupted");
    return;
  }

  // Write binary data
  for (int i = 0; i < buffer_index; i++) {
    data_file.write((uint8_t*)&data_buffer[i], sizeof(TelemetryData));
  }

  data_file.close();

  buffer_index = 0; // Reset buffer
}

String generateFilename() {
  unsigned long oldest_time = data_buffer[0].timestamp;

  // Use last 8 digits of epoch for strict 8.3 format
  // Format: XXXXXXXX.ATC (exactly 8 chars + 3 char extension)
  String timestamp_str = String(oldest_time);
  String filename;

  if (timestamp_str.length() >= 8) {
    // Take last 8 digits
    filename = timestamp_str.substring(timestamp_str.length() - 8) + ".ATC";
  } else {
    // Pad with zeros if needed
    filename = timestamp_str;
    while (filename.length() < 8) {
      filename = "0" + filename;
    }
    filename += ".ATC";
  }

  return filename;
}

void checkButton() {
  bool current_button_state = digitalRead(BUTTON_PIN);

  if (current_button_state == LOW && last_button_state == HIGH) {
    button_pressed = true;
    delay(50); // Debounce
  }

  last_button_state = current_button_state;

  if (button_pressed) {
    button_pressed = false;

    if (running) {
      // Stop operation
      if (iteration_counter % write_interval != 0) {
        writeDataToSD(); // Write remaining data
      }
      running = false;
      Serial.println("Stopped by button press");
    } else {
      // Start operation
      Serial.println("Restarting by button press");
      performSetup();
    }
  }
}

void updateTimeAndLEDs() {
  unsigned long current_time = getCurrentEpochTime();
  static unsigned long last_minute = 0;
  static unsigned long last_second = 0;

  // Check for new minute
  if (current_time / 60 != last_minute / 60) {
    digitalWrite(WHITE_LED, HIGH);
    last_minute = current_time;
  }

  // Check for new second
  if (current_time != last_second) {
    digitalWrite(YELLOW_LED, HIGH);
    last_second = current_time;
  }

  // Check loop timing
  unsigned long current_loop_time = millis();
  if (current_loop_time - last_loop_time > loop_duration) {
    digitalWrite(RED_LED, HIGH);
  } else {
    digitalWrite(RED_LED, LOW);
  }
}

void turnOffLEDs() {
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(WHITE_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  // Note: Red LED is controlled by timing logic
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
    Serial.print("Loop timing violation! Skipped ");
    Serial.print(iterations_to_skip);
    Serial.print(" iterations (total violations: ");
    Serial.print(total_iterations_skipped);
    Serial.println(")");
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

int16_t encodeTemperature(float temp_c) {
  // Scale: -40°C to +85°C range with 0.002°C precision
  float scaled = (temp_c + 50.0) * 512.0;
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
