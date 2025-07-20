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

// MPU-6050 I2C address
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

// State variables
bool running = false;
unsigned long iteration_counter = 0;
unsigned long last_loop_time = 0;
unsigned long gps_epoch_time = 0;
unsigned long gps_board_time = 0;
bool button_pressed = false;
bool last_button_state = HIGH;

// GPS variables
float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;
bool gps_fix = false;

// MPU-6050 variables
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

// Data buffer structure
struct TelemetryData {
  unsigned long timestamp;
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float latitude, longitude, altitude;
  bool has_gps_data;
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
  pinMode(BUTTON_PIN, INPUT_PULLUP);
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
      acquireGPSData();
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
  bool setup_success = false;

  while (!setup_success) {
    // Step 1: Configuration validation
    if (!validateConfiguration()) {
      blinkError(5);
      continue;
    }

    // Allocate data buffer
    data_buffer = (TelemetryData*)malloc(write_interval * sizeof(TelemetryData));
    if (data_buffer == NULL) {
      Serial.println("Failed to allocate data buffer");
      blinkError(5);
      continue;
    }
    buffer_index = 0;

    // Step 2: Initialize I2C and MPU-6050
    Wire.begin();
    if (!initializeMPU6050()) {
      blinkError(2);
      continue;
    }

    // Step 3: Initialize SD card
    if (!initializeSD()) {
      blinkError(1);
      continue;
    }

    // Step 4: Acquire GPS fix
    if (!acquireGPSFix()) {
      blinkError(4);
      continue;
    }

    // Step 5: Acquire time
    if (!acquireTime()) {
      blinkError(3);
      continue;
    }

    // Test SD card write
    if (!testSDWrite()) {
      blinkError(1);
      continue;
    }

    setup_success = true;
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

  Serial.println("Configuration validation passed");
  return true;
}

bool initializeMPU6050() {
  Serial.println("Initializing MPU-6050...");

  // Wake up MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up
  Wire.endTransmission(true);
  delay(100);

  // Test read
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);

  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    if (who_am_i == 0x68) {
      Serial.println("MPU-6050 initialized successfully");
      return true;
    }
  }

  Serial.println("MPU-6050 initialization failed");
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
  Serial.println("Acquiring GPS fix...");
  unsigned long start_time = millis();

  while (millis() - start_time < 60000) { // 60 second timeout
    if (GPS_SERIAL.available()) {
      String gps_data = GPS_SERIAL.readStringUntil('\n');
      if (parseGPSData(gps_data)) {
        if (gps_fix) {
          Serial.println("GPS fix acquired");
          return true;
        }
      }
    }
    delay(100);
  }

  Serial.println("GPS fix acquisition failed");
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

  File test_file = SD.open("setup.txt", FILE_WRITE);
  if (!test_file) {
    Serial.println("Failed to open setup.txt for writing");
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

  if (read_data.toInt() == gps_epoch_time) {
    Serial.println("SD card write test passed");
    return true;
  }

  Serial.println("SD card write test failed");
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

  // Store in buffer
  if (buffer_index < write_interval) {
    data_buffer[buffer_index].timestamp = getCurrentEpochTime();
    data_buffer[buffer_index].accel_x = accel_x;
    data_buffer[buffer_index].accel_y = accel_y;
    data_buffer[buffer_index].accel_z = accel_z;
    data_buffer[buffer_index].gyro_x = gyro_x;
    data_buffer[buffer_index].gyro_y = gyro_y;
    data_buffer[buffer_index].gyro_z = gyro_z;
    data_buffer[buffer_index].latitude = latitude;
    data_buffer[buffer_index].longitude = longitude;
    data_buffer[buffer_index].altitude = altitude;
    data_buffer[buffer_index].has_gps_data = (iteration_counter % gps_interval == 0);

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

bool parseGPSData(String data) {
  if (data.startsWith("$GPGGA") || data.startsWith("$GNGGA")) {
    // Parse NMEA GGA sentence for position
    int comma_positions[14];
    int comma_count = 0;

    for (int i = 0; i < data.length() && comma_count < 14; i++) {
      if (data.charAt(i) == ',') {
        comma_positions[comma_count++] = i;
      }
    }

    if (comma_count >= 6) {
      String lat_str = data.substring(comma_positions[1] + 1, comma_positions[2]);
      String lon_str = data.substring(comma_positions[3] + 1, comma_positions[4]);
      String alt_str = data.substring(comma_positions[8] + 1, comma_positions[9]);
      String fix_str = data.substring(comma_positions[5] + 1, comma_positions[6]);

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
  if (coord.length() < 4) return 0.0;

  float degrees = coord.substring(0, coord.indexOf('.') - 2).toFloat();
  float minutes = coord.substring(coord.indexOf('.') - 2).toFloat();

  return degrees + minutes / 60.0;
}

unsigned long convertGPSTimeToEpoch(String time_str, String date_str) {
  // Simplified conversion - would need proper date/time library for accuracy
  // For now, return a placeholder
  return 1600000000; // Placeholder epoch time
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
    return;
  }

  // Write binary data
  for (int i = 0; i < buffer_index; i++) {
    data_file.write((uint8_t*)&data_buffer[i], sizeof(TelemetryData));
  }

  data_file.close();

  Serial.print("Wrote ");
  Serial.print(buffer_index);
  Serial.print(" records to ");
  Serial.println(filename);

  buffer_index = 0; // Reset buffer
}

String generateFilename() {
  unsigned long oldest_time = data_buffer[0].timestamp;

  // Convert to date/time format (simplified)
  // Would need proper date/time library for accurate conversion
  String filename = write_prefix + "_2024-01-01_120000000.atc";
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
    // Skip iterations to catch up
    unsigned long iterations_to_skip = loop_time / loop_duration;
    iteration_counter += iterations_to_skip;
    Serial.print("Skipped ");
    Serial.print(iterations_to_skip);
    Serial.println(" iterations");
  }

  last_loop_time = millis();
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
