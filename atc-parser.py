#!/usr/bin/env python3
"""
ATC Format Parser

Parses Arduino Telemetry Collector files in the simplified format specified in data.md
Converts binary .ATC files to CSV format.

Usage:
    python atc-parser.py <input_file.ATC> [output_file.csv]
    
Arguments:
    input_file  - ATC file to parse (mandatory)
    output_file - CSV output file (optional, defaults to input_file.csv)
"""

import sys
import struct
import csv
from datetime import datetime, timezone

# ATC format constants
MAGIC_BYTES = b'ATC\x00'
HEADER_SIZE = 16

# Sensor read flags (bits 0-3)
ATC_ROW_ACCL = 0x01  # Bit 0
ATC_ROW_GYRO = 0x02  # Bit 1  
ATC_ROW_MAGN = 0x04  # Bit 2
ATC_ROW_GPS = 0x08   # Bit 3

# Sensor failure flags (bits 4-7)
ATC_NAN_ACCL = 0x10  # Bit 4
ATC_NAN_GYRO = 0x20  # Bit 5
ATC_NAN_MAGN = 0x40  # Bit 6
ATC_NAN_GPS = 0x80   # Bit 7

# Data sizes per sensor (in bytes)
TIME_OFFSET_SIZE = 4
ACCELEROMETER_SIZE = 6  # 3 × 16-bit integers
GYROSCOPE_SIZE = 6      # 3 × 16-bit integers  
MAGNETOMETER_SIZE = 6   # 3 × 16-bit integers
GPS_SIZE = 10           # 2 × 32-bit floats + 2 × 8-bit integers

CSV_HEADERS = [
    'timestamp_epoch',
    'timestamp_iso',
    'accel_x_g', 'accel_y_g', 'accel_z_g',
    'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps',
    'mag_x_raw', 'mag_y_raw', 'mag_z_raw',
    'gps_longitude', 'gps_latitude', 'gps_hdop', 'gps_satellites'
]

def parse_header(data):
    """Parse ATC file header and return (version, sensor_config, reference_timestamp, next_offset)"""
    if len(data) < HEADER_SIZE:
        raise ValueError("File too small for ATC header")
    
    # Check magic bytes
    magic = data[0:4]
    if magic != MAGIC_BYTES:
        raise ValueError(f"Invalid magic bytes: expected {MAGIC_BYTES}, got {magic}")
    
    # Parse header fields (all little-endian)
    version = struct.unpack('<H', data[4:6])[0]
    sensor_config = list(data[6:10])  # 4 bytes: [accel, gyro, magn, gps]
    reference_seconds = struct.unpack('<I', data[10:14])[0]
    reference_millis = struct.unpack('<H', data[14:16])[0]
    
    # Calculate reference timestamp as float
    reference_timestamp = reference_seconds + (reference_millis / 1000.0)
    
    return version, sensor_config, reference_timestamp, HEADER_SIZE

def get_accel_scale_factor(config):
    """Get accelerometer scale factor based on configuration byte."""
    scale_map = {
        0: None,      # No sensor
        1: 2.0,       # ±2g range
        2: 4.0,       # ±4g range  
        3: 8.0,       # ±8g range
        4: 16.0       # ±16g range
    }
    return scale_map.get(config, None)

def get_gyro_scale_factor(config):
    """Get gyroscope scale factor based on configuration byte."""
    scale_map = {
        0: None,      # No sensor
        1: 250.0,     # ±250°/s range
        2: 500.0,     # ±500°/s range
        3: 1000.0,    # ±1000°/s range
        4: 2000.0     # ±2000°/s range
    }
    return scale_map.get(config, None)

def convert_raw_to_physical(raw_value, range_value):
    """Convert raw 16-bit value to physical units."""
    if range_value is None:
        return raw_value
    # Raw value is 16-bit signed, scale to physical range
    return (raw_value / 32768.0) * range_value

def parse_observation(data, offset, reference_timestamp, sensor_config):
    """Parse single observation and return (observation_dict, next_offset)"""
    if len(data) < offset + 1 + TIME_OFFSET_SIZE:
        raise ValueError("Insufficient data for observation header")
    
    # Parse row configuration byte
    row_config = data[offset]
    offset += 1
    
    # Parse time offset
    time_offset = struct.unpack('<I', data[offset:offset + TIME_OFFSET_SIZE])[0]
    offset += TIME_OFFSET_SIZE
    
    # Calculate absolute timestamp
    absolute_timestamp = reference_timestamp + (time_offset / 1000.0)
    
    # Get scale factors for physical unit conversion
    accel_scale = get_accel_scale_factor(sensor_config[0])
    gyro_scale = get_gyro_scale_factor(sensor_config[1])
    
    # Initialize observation with all fields as None
    observation = {
        'timestamp_epoch': absolute_timestamp,
        'timestamp_iso': datetime.fromtimestamp(absolute_timestamp, tz=timezone.utc).isoformat(),
        'accel_x_g': None, 'accel_y_g': None, 'accel_z_g': None,
        'gyro_x_dps': None, 'gyro_y_dps': None, 'gyro_z_dps': None,
        'mag_x_raw': None, 'mag_y_raw': None, 'mag_z_raw': None,
        'gps_longitude': None, 'gps_latitude': None, 'gps_hdop': None, 'gps_satellites': None
    }
    
    # Parse accelerometer data if attempted and successful
    if (row_config & ATC_ROW_ACCL) and not (row_config & ATC_NAN_ACCL):
        if len(data) < offset + ACCELEROMETER_SIZE:
            raise ValueError("Insufficient data for accelerometer reading")
        
        accel_data = struct.unpack('<hhh', data[offset:offset + ACCELEROMETER_SIZE])
        
        # Convert to physical units (g)
        observation['accel_x_g'] = convert_raw_to_physical(accel_data[0], accel_scale)
        observation['accel_y_g'] = convert_raw_to_physical(accel_data[1], accel_scale)
        observation['accel_z_g'] = convert_raw_to_physical(accel_data[2], accel_scale)
        offset += ACCELEROMETER_SIZE
    
    # Parse gyroscope data if attempted and successful
    if (row_config & ATC_ROW_GYRO) and not (row_config & ATC_NAN_GYRO):
        if len(data) < offset + GYROSCOPE_SIZE:
            raise ValueError("Insufficient data for gyroscope reading")
        
        gyro_data = struct.unpack('<hhh', data[offset:offset + GYROSCOPE_SIZE])
        
        # Convert to physical units (degrees per second)
        observation['gyro_x_dps'] = convert_raw_to_physical(gyro_data[0], gyro_scale)
        observation['gyro_y_dps'] = convert_raw_to_physical(gyro_data[1], gyro_scale)
        observation['gyro_z_dps'] = convert_raw_to_physical(gyro_data[2], gyro_scale)
        offset += GYROSCOPE_SIZE
    
    # Parse magnetometer data if attempted and successful
    if (row_config & ATC_ROW_MAGN) and not (row_config & ATC_NAN_MAGN):
        if len(data) < offset + MAGNETOMETER_SIZE:
            raise ValueError("Insufficient data for magnetometer reading")
        
        mag_data = struct.unpack('<hhh', data[offset:offset + MAGNETOMETER_SIZE])
        observation['mag_x_raw'] = mag_data[0]
        observation['mag_y_raw'] = mag_data[1]
        observation['mag_z_raw'] = mag_data[2]
        offset += MAGNETOMETER_SIZE
    
    # Parse GPS data if attempted and successful
    if (row_config & ATC_ROW_GPS) and not (row_config & ATC_NAN_GPS):
        if len(data) < offset + GPS_SIZE:
            raise ValueError("Insufficient data for GPS reading")
        
        # GPS: longitude(float32), latitude(float32), hdop(uint8), satellites(uint8)
        gps_data = struct.unpack('<ffBB', data[offset:offset + GPS_SIZE])
        observation['gps_longitude'] = gps_data[0] if gps_data[0] != 0.0 else None
        observation['gps_latitude'] = gps_data[1] if gps_data[1] != 0.0 else None
        observation['gps_hdop'] = gps_data[2] / 10.0 if gps_data[2] != 0 else None
        observation['gps_satellites'] = gps_data[3] if gps_data[3] != 0 else None
        offset += GPS_SIZE
    
    return observation, offset

def parse_atc_file(filename):
    """Parse entire ATC file and return list of observations"""
    with open(filename, 'rb') as f:
        data = f.read()
    
    if len(data) == 0:
        raise ValueError("Empty file")
    
    print(f"Processing ATC file: {filename} ({len(data)} bytes)")
    
    # Parse header
    version, sensor_config, reference_timestamp, offset = parse_header(data)
    
    print(f"ATC version: {version}")
    print(f"Sensor config: Accel={sensor_config[0]}, Gyro={sensor_config[1]}, Magn={sensor_config[2]}, GPS={sensor_config[3]}")
    print(f"Reference timestamp: {reference_timestamp} ({datetime.fromtimestamp(reference_timestamp, tz=timezone.utc).isoformat()})")
    
    # Parse observations until end of data
    observations = []
    obs_count = 0
    
    while offset < len(data):
        try:
            observation, offset = parse_observation(data, offset, reference_timestamp, sensor_config)
            observations.append(observation)
            obs_count += 1
            
            # Progress indicator for large files
            if obs_count % 1000 == 0:
                print(f"Parsed {obs_count} observations...")
                
        except ValueError as e:
            print(f"Warning: Error parsing observation at offset {offset}: {e}")
            break
    
    print(f"Successfully parsed {len(observations)} observations")
    return observations

def write_csv(observations, filename):
    """Write observations to CSV file"""
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=CSV_HEADERS)
        writer.writeheader()
        
        for obs in observations:
            # Convert None values to empty strings for CSV
            row = {key: (value if value is not None else '') for key, value in obs.items()}
            writer.writerow(row)
    
    print(f"Wrote {len(observations)} observations to {filename}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python atc-parser.py <input_file.ATC> [output_file.csv]")
        print()
        print("Arguments:")
        print("  input_file  - ATC file to parse (mandatory)")
        print("  output_file - CSV output file (optional)")
        print()
        print("Examples:")
        print("  python atc-parser.py data.ATC")
        print("  python atc-parser.py data.ATC output.csv")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else input_file.rsplit('.', 1)[0] + '.csv'
    
    try:
        observations = parse_atc_file(input_file)
        if observations:
            write_csv(observations, output_file)
            print("Parsing completed successfully!")
        else:
            print("No observations found in ATC file")
    
    except Exception as e:
        print(f"Error processing {input_file}: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()