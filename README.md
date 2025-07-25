# Arduino Telemetry Collector

Arduino setup gathering telemetry and location whilst active, writing it to an SD card.

## Purpose

To be installed in a car to gather telemetry about its location and the road it is driving on.  
This data is periodically written to an SD card.

## Physical Setup

### Components

Consists of:

- Compute unit: Arduino Mega board
- Accelerometer/Gyroscope/Temperature: MPU-6500 (upgrade from MPU-6050)
- Magnetometer: missing, future work
- GPS: GY-NEO6MV2
- Storage: micro SD card adapter

Further extended with Leds for informational use:

- Green led: blink on every read cycle finished of the Accelerometer and the Gyroscope. After all finished, not for each separately.
- Blue led: blink on every read cycle finished of the GPS.
- White led: blinks at start of every new minute. Based on wall-clock time.
- Yellow led: blinks at start of every new second. Based on wall-clock time.
- Red led: indicator for bad states/failures.

Finally, there is a simple button that enables stop-start.
All of it wired together on a breadboard.

### Pin Assignments

**Arduino Mega 2560 Connections:**

- **I2C Bus (MPU-6050):** SDA=Pin 20, SCL=Pin 21
- **SPI Bus (SD Card):** MOSI=Pin 51, MISO=Pin 50, SCK=Pin 52, CS=Pin 53
- **GPS Module:** RX1=Pin 19, TX1=Pin 18 (Serial1)
- **Status LEDs:**
  - Yellow LED: Pin 2
  - White LED: Pin 3
  - Blue LED: Pin 4
  - Green LED: Pin 5
  - Red LED: Pin 7
- **Start/Stop Button:** Pin 6 (with internal pull-up)

### Wiring Specifications

- **LED Current Limiting:** 220Ω resistors for each LED (~20mA @ 5V)
- **I2C Pull-ups:** 4.7kΩ resistors on SDA and SCL lines
- **Button:** Connect between Pin 6 and GND, with 10kΩ pull-up resistor from Pin 6 to 5V
- **Power Supply:** USB-C connection (5V direct from car's USB-C port)

### Power Requirements

**Component Power Consumption:**

- Arduino Mega: 225mW
- MPU-6050: 19.5mW
- GPS Module: 335mW
- SD Card: 125mW average, 500mW peak during writes
- LEDs: 100mW total when active
- **Total System:** 600mW average, 800mW peak
- **From USB-C Supply:** 600-800mW (no regulator losses needed)

### Physical Wiring Diagram

```
                    Arduino Mega 2560
                 ┌─────────────────────┐
    USB-C ──────►│VIN               GND├──── Common Ground
                 │                     │
    ┌────────────┤20(SDA)         53(CS)├─── SD Card Module
    │    ┌───────┤21(SCL)         52(SCK)├─── SD Card Module
    │    │   ┌───┤51(MOSI)       50(MISO)├─── SD Card Module
    │    │   │   │                     │
    │    │   │   ├19(RX1)         18(TX1)├─── GY-NEO6MV2 GPS
    │    │   │   │                     │
    │    │   │   ├2               Pin 6├─┬─ Start/Stop Button ──┤
    │    │   │   ├3                    │ │                      GND
    │    │   │   ├4                    │ └─[10kΩ]── +5V
    │    │   │   ├5                    │
    │    │   │   ├7                5V├──── +5V Power Rail
    │    │   │   └─────────────────────┘
    │    │   │
    │    │   └─── SD Card Module (SPI)
    │    │         ┌─────────────┐
    │    │         │  VCC    GND │
    │    │         │  MOSI   MISO│
    │    │         │  SCK    CS  │
    │    │         └─────────────┘
    │    │
    │    └───── MPU-6050 (I2C)
    │            ┌─────────────┐
    │            │ VCC     INT │
    │            │ GND     AD0 │
    │            │ SCL     XCL │
    │            │ SDA     XDA │
    │            └─────────────┘
    │
    └─────── GY-NEO6MV2 GPS
             ┌─────────────┐
             │ VCC     PPS │
             │ RX      GND │
             │ TX          │
             └─────────────┘

 Status LEDs (with 220Ω resistors):
 Pin 2 ──[220Ω]──►|── Yellow LED ─ GND
 Pin 3 ──[220Ω]──►|── White LED ── GND
 Pin 4 ──[220Ω]──►|── Blue LED ─── GND
 Pin 5 ──[220Ω]──►|── Green LED ── GND
 Pin 7 ──[220Ω]──►|── Red LED ──── GND

 I2C Pull-ups:
 +5V ──[4.7kΩ]── SDA (Pin 20)
 +5V ──[4.7kΩ]── SCL (Pin 21)

 Button Pull-up:
 +5V ──[10kΩ]─── Pin 6 ──[Button]── GND

 Power Supply:
 USB-C ──────────────── 5V ──[Common Rail]── All Components
```

## Behavior

### Setup

When the board is powered, it **always** goes through its setup routine below.
For each step, if it fails, it will blink an error state with the red led before retrying. During a blink, the led is on for `blink_on_duration` milliseconds (default: 500).
Each red blink is separated by `blink_off_duration` milliseconds (default: 500). After the pattern, the system waits `blink_pause_duration` milliseconds before retrying (default: 2000).

1. Configuration setup. _(If failed: 5 blinks)_
2. It acquires a GPS fix. _(If failed: 4 blinks)_
3. It acquires the time. _(If failed: 3 blinks)_
4. It tests whether it can read telemetry from the MPU-6050. _(If failed: 2 blinks)_
5. It tests whether it can write to the SD card. _(If failed: 1 blink)_

After succesful setup, the program will always set `running` variable to its true value and proceed to the loop step.

#### Configuration setup

Checks whether the set configuration is valid. Checks these parameters:

- `blink_on_duration`: Must be at least 100.
- `blink_off_duration`: Must be at least 100.
- `blink_pause_duration`: Must be at least 100.
- `loop_duration`: Must be at least 1.
- `telemetry_interval`: Must be at least 1.
- `gps_interval`: Must be at least 1. Must be equal or larger than `telemetry_interval`. `gps_interval` times `loop_duration` must be at least 200.
- `write_interval`: Must be at least 1. Must be equal or larger than `gps_interval`.
- `write_prefix`: Must be at least 1 and at most 8 ASCII characters.
- `gps_fix_duration`: Must be at least 5000 milliseconds. Duration to wait for GPS location fix in each attempt.
- `gps_fix_tries`: Number of GPS fix attempts during setup. 0 means retry indefinitely (original behavior).

Also establishes our data collection buffer and allocates sufficient space for it. See the [data buffer](#data-buffer) section.

#### Acquire GPS fix

Attempts to acquire initial GPS coordinates with configurable timeout and retry behavior:

- **Location fix attempts**: Up to `gps_fix_tries` attempts (0 = infinite retries)
- **Per-attempt timeout**: `gps_fix_duration` milliseconds (default: 30 seconds)
- **Failure handling**: If all attempts fail, setup continues without GPS location
- **Indoor operation**: GPS location may be unavailable indoors but system continues normally

#### Acquire time

Time acquisition from GPS is **mandatory** and will retry indefinitely until successful:

- Time is acquired by reading the GPS module NMEA time sentences
- Board timestamp is captured simultaneously for time tracking
- Time acquisition typically succeeds even when location fix fails (indoor operation)
- System cannot proceed without GPS time synchronization

#### GPS Behavior Summary

**Setup Phase:**
- GPS location: Optional (configurable attempts/timeouts)
- GPS time: Required (infinite retries until success)

**Main Loop Phase:**
- Every `gps_interval` iterations, attempt GPS data acquisition
- **Time budget constraint**: GPS acquisition limited by remaining loop time
- **Location recovery**: If no location fix during setup, main loop continues trying within time budget
- **Graceful degradation**: Missing location data recorded as empty fields, does not block operation

#### SD card access.

The SD card test is done by writing the file `setup.txt`. It contains the epoch timestamp of the acquired time in the 3rd step.
It then reads the file back in to verify the writing was successful.

### Loop

The loop handles one or multiple tasks, depending on the iteration.
Every iteration start-to-end must finish within `loop_duration` milliseconds (default: 10).
If it finishes faster, the program will wait at the end of the loop to ensure it took `loop_duration` milliseconds before the next loop iteration starts.  
If it finishes too late, sufficient iterations will be skipped to catch up with expected timing. These skips mean that no loop tasks are performed. Only the iteration counter is incremented. This will permit us to see afterwards in the data that iterations were skipped.

The loop tasks when the state is `running` are, in order:

1. Iteration start
2. Location acquisition.
3. Time acquisition.
4. Telemetry collection.
5. Data writing.

#### Tasks: iteration start

- Increment the iteration counter by 1.
- Turn off all leds except for the red one.

#### Tasks: location acquisition

Every `gps_interval` iterations (default: 100):

- The GPS location will be acquired and stored in the data buffer.
- Turns on the blue led for this iteration.

#### Tasks: time acquisition

Every `gps_interval` iterations, the time will be acquired through the GPS. While this is listed as a separate task, for efficacy the GPS time acquisition happens together with location acquisition.
On every iteration, the current board time will be captured as well as the iteration counter of the loop. The counter starts at 1.

Based on these timings, it will also turn on leds:

- If a new wall clock minute started since the previous iteration, the white led is turned on.
- If a new wall clock second started since the previous iteration, the yellow led is turned on.
- If the iteration before this one did not finish within `loop_duration` milliseconds, the red led is turned on.

If the iteration before **did** finish within `loop_duration` milliseconds, the red led will be turned off.

#### Tasks: telemetry collection

Every `telemetry_interval` iterations (default: 1):

- All accelerometer, gyroscope, and temperature values are collected from MPU-6500
- Values are compressed to 16-bit integers for efficient storage:
  - Accelerometer: ±3.2768g range with 0.0001g precision
  - Gyroscope: ±250°/s range with 0.0076°/s precision  
  - Temperature: -40°C to +85°C range with 0.002°C precision
- Data is stored in the compressed data buffer
- Turns on the green led.

#### Tasks: data writing

Every `write_interval` iterations (current: 10), the data buffer is written to the SD card.
The filename format is: `XXXXXXXX.ATC` where XXXXXXXX represents the last 8 digits of the Unix epoch timestamp for 8.3 filename compatibility.
The data is in a binary format to keep the writing operation as simple as possible and thus as fast as possible. For reading the data out, see the [data parser](#data-parser).

### Start-stop button

The physical setup contains a start-stop button.
When the state is `running` and the button is pressed:

- The current iteration finishes.
- The data writing task is executed.
  - Exception: if the last finished iteration included the data writing task. In that case, it is not executed again.

The state of `running` is set to the false value. The loop iterations will now do nothing until the button is pressed **again**.

When the state is not `running` (i.e., `running` had the false value) and the button is pressed:

- The `running` state is changed back to its true value.
- The [setup](#setup) is executed again.
- The [loop](#loop) is started (if setup was succesful).

## Data Buffer

**Memory Constraints:**
The Arduino Mega has 8KB SRAM total. The data buffer must fit within available memory after accounting for program variables and stack space.

**Data Structure per Sample (16-bit Compressed):**

- Raw sensor data: 6 int16 values (3-axis accelerometer + 3-axis gyroscope) = 12 bytes
- Temperature: 1 int16 value = 2 bytes
- GPS data: latitude, longitude, altitude = 12 bytes (float precision maintained)
- Timestamp: epoch seconds = 4 bytes
- GPS validity flag: 1 byte
- Padding: 2 bytes for alignment
- **Total per sample: 33 bytes (26.7% reduction)**

**Buffer Size Calculation:**
With current settings (write_interval=10):

- Buffer for 10 iterations × 33 bytes = 330 bytes total
- **Current buffer requirement: ~0.33KB**

**Maximum Supported write_interval:** Approximately 242 iterations (8KB SRAM ÷ 33 bytes per sample)

**File Size Estimates:**
With current configuration (write_interval=10, loop_duration=100ms):

- Per file (every 1 second): ~0.33KB
- Per minute: ~20KB (60 files)
- Per hour: ~1.2MB
- Per day: ~29MB
- Recommended SD card: 2GB+ for several weeks of data

## Data Parser

## Environmental Considerations

**Operating Temperature:**

- Arduino Mega specification: -40°C to +85°C
- Automotive cabin environment: typically -30°C to +70°C
- **Assessment:** Compatible with automotive use when mounted away from direct engine heat

**Vibration and Shock:**

- Breadboard construction is not ideal for automotive environment
- Recommend securing components and adding shock absorption
- Consider migration to PCB for production use

**Coordinate System**

**MPU-6050 Sensor Orientation:**
The accelerometer and gyroscope report data in a standard 3-axis coordinate system:

- **X-axis:** Typically forward/backward relative to device orientation
- **Y-axis:** Typically left/right relative to device orientation
- **Z-axis:** Typically up/down relative to device orientation

**Installation Notes:**

- Document the physical mounting orientation of the MPU-6050 in the vehicle
- Data will be reported as measured; no automatic orientation correction
- For consistent results, maintain same mounting orientation across installations
- Consider adding calibration routine to establish baseline orientation

## Data Parser

The `parse-telemetry.py` script is a pure Python script that can read the written binary files and convert them into CSVs with a header.
It takes 2 positional arguments:

- `source`: Where to read from. Mandatory.
- `destination`: Where to write to. Optional. If not specified, the current working directory is assumed.

`source` can be a single file or a directory. If it is a directory, all files with the `.atc` extension in that directory are read.
`destination` can be a file path or a directory path. If it is a directory (or not specified), the file names will be kept, replacing the `.atc` extension with `.csv`.
If `source` is a directory, `destination` must also be a directory.
