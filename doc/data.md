# Arduino Telemetry Collector (ATC) Data Format

This document describes the simplified binary data format used by the Arduino Telemetry Collector for storing sensor telemetry data.

## Overview

The ATC format is designed for efficient storage of mixed-frequency sensor data from automotive telemetry systems. Each `.ATC` file contains a header followed by a stream of variable-length observations.

## File Structure

```
ATC File
├── File Header (16 bytes)
│   ├── Magic Bytes (4 bytes)
│   ├── Version (2 bytes)
│   ├── Sensor Configuration (4 bytes)
│   ├── Reference Timestamp Seconds (4 bytes)
│   └── Reference Timestamp Milliseconds (2 bytes)
└── Observations (variable length)
    ├── Observation 1
    ├── Observation 2
    └── ...
```

## File Header

The file header is exactly 16 bytes and appears once at the beginning of each file.

### Magic Bytes (4 bytes)

- **Offset**: 0
- **Size**: 4 bytes
- **Format**: ASCII string "ATC\0"
- **Purpose**: File type identification

### Version (2 bytes)

- **Offset**: 4
- **Size**: 2 bytes
- **Format**: Little-endian uint16
- **Purpose**: Format version number (currently 0)

### Sensor Configuration (4 bytes)

- **Offset**: 6
- **Size**: 4 bytes
- **Format**: 4 consecutive uint8 values
- **Order**: [Accelerometer, Gyroscope, Magnetometer, GPS]

#### Accelerometer Configuration Byte

The Accelerometer byte of the Sensor Configuration can be interpreted as specified below:

- 0: No accelerometer sensor present/configured.
- 1: Sensor measures in a -2g to +2g range.
- 2: Sensor measures in a -4g to +4g range.
- 3: Sensor measures in a -8g to +8g range.
- 4: Sensor measures in a -16g to +16g range.

Higher values are currently not in use and should not occur.
When they do, the configuration should be considered invalid.

#### Gyroscope Configuration Byte

The Gyroscope byte of the Sensor Configuration can be interpreted as specified below:

- 0: No gyroscope sensor present/configured.
- 1: Sensor measures in a -250 deg/s to +250 deg/s range.
- 2: Sensor measures in a -500 deg/s to +500 deg/s range.
- 3: Sensor measures in a -1000 deg/s to +1000 deg/s range.
- 4: Sensor measures in a -2000 deg/s to +2000 deg/s range.

Higher values are currently not in use and should not occur.
When they do, the configuration should be considered invalid.

#### Magnetometer Configuration Byte

The Magnetometer byte of the Sensor Configuration can be interpreted as specified below:

- 0: No magnetometer sensor present/configured.

As magnetometer reading is currently not supported, 0 is the only valid value.

#### GPS Configuration Byte

The GPS byte of the Sensor Configuration can be interpreted as specified below:

- 0: No GPS sensor present/configured.
- 1: Sensor configured to measure at 1 Hz navigation update rate.

Higher values are currently not in use and should not occur.
When they do, the configuration should be considered invalid.

### Reference Timestamp Seconds (4 bytes)

- **Offset**: 10
- **Size**: 4 bytes
- **Format**: Little-endian uint32
- **Purpose**: Unix epoch timestamp seconds (seconds since 1970-01-01 00:00:00 UTC)
- **Usage**: Base timestamp seconds for all observations in this file

### Reference Timestamp Milliseconds (2 bytes)

- **Offset**: 14
- **Size**: 2 bytes
- **Format**: Little-endian uint16
- **Purpose**: Millisecond component of the reference timestamp
- **Range**: 0-999 milliseconds
- **Usage**: Provides sub-second precision for the reference timestamp

## Observations

After the 16-byte header, the file contains a stream of variable-length observations. Each observation represents sensor readings at a specific point in time.

### Observation Structure

```
Observation
├── Row Configuration (1 byte)
├── Time Offset (4 bytes)
└── Sensor Data (variable length)
    ├── Accelerometer Data (6 bytes, if attempted and successful)
    ├── Gyroscope Data (6 bytes, if attempted and successful)
    ├── Magnetometer Data (6 bytes, if attempted and successful)
    └── GPS Data (10 bytes, if attempted and successful)
```

### Row Configuration (1 byte)

The row configuration byte uses a bitmap to indicate which sensors are present and which failed to read:

**Bits 0-3: Sensor Read Flags**

- Bit 0 (0x01): Accelerometer read attempted
- Bit 1 (0x02): Gyroscope read attempted
- Bit 2 (0x04): Magnetometer read attempted
- Bit 3 (0x08): GPS read attempted

**Bits 4-7: Sensor Failure Flags (NaN)**

- Bit 4 (0x10): Accelerometer reading failed
- Bit 5 (0x20): Gyroscope reading failed
- Bit 6 (0x40): Magnetometer reading failed
- Bit 7 (0x80): GPS reading failed

A sensor can be both attempted and failed (e.g., bits 0 and 4 both set means accelerometer read was attempted but the I2C read failed).  
When a sensor read is not attempted, there is nothing to fail. The failure flag bears no meaning in that scenario.

### Time Offset (4 bytes)

- **Size**: 4 bytes
- **Format**: Little-endian uint32
- **Units**: Milliseconds since the Reference Timestamp
- **Range**: 0 to 4,294,967,295 ms (~49.7 days)

**Absolute Timestamp Calculation**:

```
absolute_timestamp = reference_timestamp_seconds + (reference_timestamp_millis / 1000.0) + (time_offset / 1000.0)
```

### Sensor Data

Sensor data appears in the observation only if the corresponding read flag is set AND the corresponding failure flag is NOT set. The data appears in the following order:

#### Accelerometer Data (6 bytes, if attempted and successful)

- **Size**: 6 bytes
- **Format**: 3 consecutive little-endian int16 values
- **Order**: [X-axis, Y-axis, Z-axis]
- **Units**: Raw 16-bit accelerometer values
- **Range**: ±2g (configurable, see sensor configuration)

#### Gyroscope Data (6 bytes, if attempted and successful)

- **Size**: 6 bytes
- **Format**: 3 consecutive little-endian int16 values
- **Order**: [X-axis, Y-axis, Z-axis]
- **Units**: Raw 16-bit gyroscope values
- **Range**: ±250°/s (configurable, see sensor configuration)

#### Magnetometer Data (6 bytes, if attempted and successful)

- **Size**: 6 bytes
- **Format**: 3 consecutive little-endian int16 values
- **Order**: [X-axis, Y-axis, Z-axis]
- **Units**: Raw 16-bit magnetometer values
- **Note**: Currently unused (magnetometer interval = 0)

#### GPS Data (10 bytes, if attempted and successful)

- **Size**: 10 bytes
- **Format**:
  - Longitude: 4-byte little-endian float32
  - Latitude: 4-byte little-endian float32
  - HDOP: 1-byte uint8 (horizontal dilution of precision, rounded up)
  - Satellites: 1-byte uint8 (number of satellites)
- **Special Values**:
  - HDOP = 255: Clamped value (actual HDOP ≥ 255)
  - Satellites = 255: Clamped value (actual count ≥ 255)

## Data Collection Pattern

Based on the current Arduino configuration:

- **Loop frequency**: 100 Hz (10ms intervals)
- **Accelerometer**: Every loop iteration (100 Hz)
- **Gyroscope**: Every loop iteration (100 Hz)
- **GPS**: Every 100 iterations (1 Hz)
- **Magnetometer**: Disabled

**Typical observation sizes**:

- Accelerometer + Gyroscope: 1 + 4 + 6 + 6 = 17 bytes
- Accelerometer + Gyroscope + GPS: 1 + 4 + 6 + 6 + 10 = 27 bytes

## File Management

### File Size Limits

- **Maximum file size**: 1 MB per file
- **Buffer size**: 4 KB (flushed when nearly full)
- **File naming**: 8-digit hex timestamp (e.g., `D6066886.ATC`)

### File Boundaries

When a file approaches the 1 MB limit, a new file is created with:

- New Reference Timestamp
- Fresh 16-byte header
- Continuous observation stream

## Error Handling

### Sensor Failures

When a sensor read fails (I2C timeout, invalid data, etc.):

1. The read flag is set (sensor read was attempted)
2. The corresponding failure flag is set (read failed)
3. No sensor data bytes are written for that sensor

### Time Failures

If GPS time cannot be acquired:

- System will retry indefinitely (blocks operation)
- This ensures all observations have valid timestamps

## Parsing Guidelines

### Reading Algorithm

1. Read and validate 4-byte magic bytes
2. Read version and sensor configuration
3. Read reference timestamp seconds and milliseconds
4. For each observation:
   - Read row configuration byte
   - Read 4-byte time offset
   - Based on read flags (only if corresponding failure flag is NOT set), read sensor data in order:
     - Accelerometer (6 bytes if bit 0 set and bit 4 not set)
     - Gyroscope (6 bytes if bit 1 set and bit 5 not set)
     - Magnetometer (6 bytes if bit 2 set and bit 6 not set)
     - GPS (10 bytes if bit 3 set and bit 7 not set)

### Error Detection

- Invalid magic bytes indicate corrupted or wrong file type
- Unexpected end-of-file during observation parsing
- Time offsets that exceed reasonable bounds
- Both presence and NaN flags set indicates sensor hardware issues

## Example Files

A typical 1-second capture (100 observations) would contain:

- 1 observation with accelerometer + gyroscope + GPS (27 bytes)
- 99 observations with accelerometer + gyroscope only (17 bytes each)
- **Total**: 16 + 27 + (99 × 17) = 1,726 bytes

This compact format provides efficient storage while maintaining full timestamp precision and robust error handling for automotive telemetry applications.
