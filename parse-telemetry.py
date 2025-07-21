#!/usr/bin/env python3
"""
Arduino Telemetry Parser

Converts binary telemetry files (.atc) to CSV format with headers.
Processes single files or entire directories.

Usage:
    python parse-telemetry.py <source> [destination]
    
Arguments:
    source      - File or directory to read from (mandatory)
    destination - File or directory to write to (optional, defaults to current directory)
"""

import sys
import os
import struct
import csv
from pathlib import Path
from datetime import datetime, timezone

# Data structure matching Arduino compressed TelemetryData struct
# struct TelemetryData {
#   uint16_t days_since_1970;          // 2 bytes (days since Unix epoch)
#   uint32_t milliseconds_in_day;      // 4 bytes (1ms precision within day)
#   int16_t accel_x, accel_y, accel_z;  // 6 bytes (16-bit compressed)
#   int16_t gyro_x, gyro_y, gyro_z;     // 6 bytes (16-bit compressed)
#   float latitude, longitude;         // 8 bytes (keep float for GPS precision)
#   uint16_t gps_status_packed;        // 2 bytes (bit-packed GPS/timing data)
# } = 28 bytes total

TELEMETRY_STRUCT_FORMAT = '<HLhhhhhhffH'  # Little endian: uint16 + uint32 + 6×int16 + 2×float + uint16
TELEMETRY_STRUCT_SIZE = struct.calcsize(TELEMETRY_STRUCT_FORMAT)

CSV_HEADERS = [
    'timestamp',
    'accel_x', 'accel_y', 'accel_z',
    'gyro_x', 'gyro_y', 'gyro_z',
    'latitude', 'longitude',
    'has_gps_data',
    'gps_satellites',
    'gps_satellites_clamped',
    'gps_accuracy',
    'gps_accuracy_clamped', 
    'iterations_skipped',
    'iterations_skipped_clamped'
]

# Decoding functions for 16-bit compressed sensor data
def decode_accel(int16_val):
    """Convert 16-bit accelerometer value back to g-force."""
    return int16_val / 10000.0

def decode_gyro(int16_val):
    """Convert 16-bit gyroscope value back to degrees/second."""
    return int16_val / 131.0


def unpack_gps_status(packed_uint16):
    """Unpack GPS status from 16-bit packed value.
    
    Bit layout: [15:11]=iterations_skipped, [10:6]=satellites, [5:1]=accuracy, [0]=has_gps
    Returns: (has_gps, accuracy, satellites, iterations_skipped, accuracy_clamped, satellites_clamped, skipped_clamped)
    """
    # Extract bit fields
    has_gps = bool(packed_uint16 & 0x0001)
    accuracy_raw = (packed_uint16 >> 1) & 0x1F  # 5 bits
    satellites_raw = (packed_uint16 >> 6) & 0x1F  # 5 bits  
    skipped_raw = (packed_uint16 >> 11) & 0x1F  # 5 bits
    
    # Convert to physical units  
    gps_accuracy = accuracy_raw  # HDOP value as integer
    gps_satellites = satellites_raw
    iterations_skipped = skipped_raw
    
    # Check for clamping (max values indicate clamped data)
    accuracy_clamped = (accuracy_raw == 31)
    satellites_clamped = (satellites_raw == 31)
    skipped_clamped = (skipped_raw == 31)
    
    return (has_gps, gps_accuracy, gps_satellites, iterations_skipped, 
            accuracy_clamped, satellites_clamped, skipped_clamped)

def convert_split_timestamp(days_since_1970, milliseconds_in_day):
    """Convert split timestamp format to Unix timestamp with millisecond precision."""
    # Calculate Unix timestamp in seconds
    unix_seconds = days_since_1970 * 86400  # 86400 seconds per day
    unix_seconds += milliseconds_in_day // 1000  # Add seconds from milliseconds
    
    # Calculate fractional seconds
    fractional_seconds = (milliseconds_in_day % 1000) / 1000.0
    
    return unix_seconds + fractional_seconds

def parse_telemetry_file(input_file_path, output_file_path):
    """Parse a single binary telemetry file and convert to CSV."""
    
    try:
        with open(input_file_path, 'rb') as infile:
            binary_data = infile.read()
            
        if len(binary_data) == 0:
            print(f"Warning: {input_file_path} is empty")
            return False
            
        if len(binary_data) % TELEMETRY_STRUCT_SIZE != 0:
            print(f"Warning: {input_file_path} size ({len(binary_data)} bytes) is not a multiple of record size ({TELEMETRY_STRUCT_SIZE} bytes)")
            
        # Calculate number of complete records
        num_records = len(binary_data) // TELEMETRY_STRUCT_SIZE
        
        if num_records == 0:
            print(f"Warning: {input_file_path} contains no complete records")
            return False
            
        print(f"Processing {input_file_path}: {num_records} records ({len(binary_data)} bytes)")
        
        # Parse binary data
        records = []
        for i in range(num_records):
            offset = i * TELEMETRY_STRUCT_SIZE
            record_data = binary_data[offset:offset + TELEMETRY_STRUCT_SIZE]
            
            try:
                unpacked = struct.unpack(TELEMETRY_STRUCT_FORMAT, record_data)
                
                # Decode compressed values back to physical units
                # unpacked = (days_since_1970, milliseconds_in_day, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, lat, lon, gps_status_packed)
                
                # Unpack GPS status bits
                gps_status = unpack_gps_status(unpacked[10])
                has_gps, accuracy, satellites, skipped, acc_clamped, sat_clamped, skip_clamped = gps_status
                
                decoded_record = (
                    convert_split_timestamp(unpacked[0], unpacked[1]),  # timestamp (convert from split format)
                    decode_accel(unpacked[2]),  # accel_x
                    decode_accel(unpacked[3]),  # accel_y  
                    decode_accel(unpacked[4]),  # accel_z
                    decode_gyro(unpacked[5]),   # gyro_x
                    decode_gyro(unpacked[6]),   # gyro_y
                    decode_gyro(unpacked[7]),   # gyro_z
                    unpacked[8],  # latitude (unchanged)
                    unpacked[9],  # longitude (unchanged)
                    has_gps,      # has_gps_data (unpacked)
                    satellites,   # gps_satellites (unpacked)
                    sat_clamped,  # gps_satellites_clamped
                    accuracy,     # gps_accuracy (HDOP)
                    acc_clamped,  # gps_accuracy_clamped
                    skipped,      # iterations_skipped
                    skip_clamped  # iterations_skipped_clamped
                )
                records.append(decoded_record)
            except struct.error as e:
                print(f"Error unpacking record {i} in {input_file_path}: {e}")
                continue
                
        if not records:
            print(f"No valid records found in {input_file_path}")
            return False
            
        # Write CSV file
        with open(output_file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(CSV_HEADERS)
            writer.writerows(records)
            
        print(f"Created {output_file_path} with {len(records)} records")
        return True
        
    except FileNotFoundError:
        print(f"Error: Input file {input_file_path} not found")
        return False
    except PermissionError:
        print(f"Error: Permission denied accessing {input_file_path} or {output_file_path}")
        return False
    except Exception as e:
        print(f"Error processing {input_file_path}: {e}")
        return False

def process_directory(input_dir, output_dir):
    """Process all .atc files in a directory."""
    
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    
    if not input_path.is_dir():
        print(f"Error: {input_dir} is not a directory")
        return False
        
    # Create output directory if it doesn't exist
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Find all .atc files (case insensitive)
    atc_files = list(input_path.glob("*.atc")) + list(input_path.glob("*.ATC"))
    
    if not atc_files:
        print(f"No .atc/.ATC files found in {input_dir}")
        return False
        
    print(f"Found {len(atc_files)} .atc files in {input_dir}")
    
    success_count = 0
    for atc_file in atc_files:
        # Generate output filename
        csv_filename = atc_file.stem + ".csv"
        csv_path = output_path / csv_filename
        
        if parse_telemetry_file(atc_file, csv_path):
            success_count += 1
            
    print(f"Successfully processed {success_count}/{len(atc_files)} files")
    return success_count > 0

def process_single_file(input_file, output_file):
    """Process a single file."""
    
    input_path = Path(input_file)
    
    if not input_path.is_file():
        print(f"Error: {input_file} is not a file")
        return False
        
    if input_path.suffix.lower() != '.atc':
        print(f"Warning: {input_file} does not have .atc extension")
        
    # Determine output path
    if output_file:
        output_path = Path(output_file)
        if output_path.is_dir():
            # Output is directory, use input filename with .csv extension
            csv_filename = input_path.stem + ".csv"
            output_path = output_path / csv_filename
    else:
        # No output specified, use current directory
        csv_filename = input_path.stem + ".csv"
        output_path = Path.cwd() / csv_filename
        
    return parse_telemetry_file(input_path, output_path)

def main():
    if len(sys.argv) < 2:
        print("Usage: python parse-telemetry.py <source> [destination]")
        print()
        print("Arguments:")
        print("  source      - File or directory to read from (mandatory)")
        print("  destination - File or directory to write to (optional)")
        print()
        print("Examples:")
        print("  python parse-telemetry.py data.atc")
        print("  python parse-telemetry.py data.atc output.csv")
        print("  python parse-telemetry.py input_dir/ output_dir/")
        print("  python parse-telemetry.py input_dir/")
        sys.exit(1)
        
    source = sys.argv[1]
    destination = sys.argv[2] if len(sys.argv) > 2 else None
    
    source_path = Path(source)
    
    if not source_path.exists():
        print(f"Error: {source} does not exist")
        sys.exit(1)
        
    if source_path.is_dir():
        # Processing directory
        if destination:
            dest_path = Path(destination)
            if dest_path.is_file():
                print("Error: When source is a directory, destination must also be a directory")
                sys.exit(1)
            output_dir = destination
        else:
            output_dir = str(Path.cwd())
            
        success = process_directory(source, output_dir)
    else:
        # Processing single file
        success = process_single_file(source, destination)
        
    if success:
        print("Processing completed successfully")
        sys.exit(0)
    else:
        print("Processing failed")
        sys.exit(1)

if __name__ == "__main__":
    main()