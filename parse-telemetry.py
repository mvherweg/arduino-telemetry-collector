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

# Data structure matching Arduino TelemetryData struct with padding
# struct TelemetryData {
#   unsigned long timestamp;     // 4 bytes
#   float accel_x, accel_y, accel_z;  // 12 bytes
#   float gyro_x, gyro_y, gyro_z;     // 12 bytes  
#   float latitude, longitude, altitude; // 12 bytes
#   bool has_gps_data;           // 1 byte + 3 bytes padding = 4 bytes
# } = 32 bytes total (with Arduino struct padding)

TELEMETRY_STRUCT_FORMAT = '<Lfffffffff?'  # Little endian: unsigned long + 9 floats + bool
TELEMETRY_STRUCT_SIZE = struct.calcsize(TELEMETRY_STRUCT_FORMAT)

CSV_HEADERS = [
    'timestamp',
    'accel_x', 'accel_y', 'accel_z',
    'gyro_x', 'gyro_y', 'gyro_z', 
    'latitude', 'longitude', 'altitude',
    'has_gps_data'
]

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
                records.append(unpacked)
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
    
    # Find all .atc files
    atc_files = list(input_path.glob("*.atc"))
    
    if not atc_files:
        print(f"No .atc files found in {input_dir}")
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