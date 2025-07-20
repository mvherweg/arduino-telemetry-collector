# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Arduino-based telemetry collection system designed for automotive data logging. The system gathers sensor data (accelerometer, gyroscope, GPS) and writes it to an SD card for later analysis.

## Hardware Architecture

- **Compute unit**: Arduino Mega board
- **Sensors**: MPU-6050 (accelerometer/gyroscope), GY-NEO6MV2 (GPS)
- **Storage**: Micro SD card adapter
- **Interface**: LEDs for status indication, start/stop button
- **Assembly**: Breadboard-based wiring

## Key Components and Data Flow

### Setup Phase
The system performs a 5-step initialization sequence with specific error codes (LED blink patterns):
1. Configuration validation (5 blinks if failed)
2. GPS fix acquisition (4 blinks if failed) 
3. Time synchronization (3 blinks if failed)
4. MPU-6050 sensor test (2 blinks if failed)
5. SD card write test (1 blink if failed)

### Main Loop Architecture
- Fixed timing loop with configurable `loop_duration` (default 100ms)
- Interval-based task execution:
  - Telemetry collection: every `telemetry_interval` iterations
  - GPS reading: every `gps_interval` iterations  
  - Data writing: every `write_interval` iterations
- LED status indicators for each subsystem
- Data buffer management for batch writing

### Data Pipeline
1. **Collection**: Sensors → in-memory data buffer
2. **Storage**: Buffer → SD card in binary format (.atc files)
3. **Processing**: Binary files → CSV via `parse-telemetry.py` script

## Configuration Parameters

Critical timing and validation parameters:
- `loop_duration`: Main loop timing (≥1ms)
- `telemetry_interval`, `gps_interval`, `write_interval`: Task frequencies
- Constraint: `gps_interval ≥ telemetry_interval` and `write_interval ≥ gps_interval`
- GPS timing constraint: `gps_interval × loop_duration ≥ 200ms`

## Data Processing

The `parse-telemetry.py` script converts binary telemetry files to CSV format:
```
python parse-telemetry.py <source> [destination]
```
- Processes single files or entire directories
- Converts .atc binary format to .csv with headers
- Handles batch processing for multiple files

## Development Notes

- This appears to be an early-stage project with only documentation currently present
- Arduino source code (.ino), headers (.h), and C++ files (.cpp) are not yet implemented
- The Python parser script referenced in documentation is not yet present
- When implementing, follow the detailed behavioral specifications in README.md