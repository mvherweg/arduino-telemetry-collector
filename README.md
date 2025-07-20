# Arduino Telemetry Collector

Arduino setup gathering telemetry and location whilst active, writing it to an SD card.

## Purpose

To be installed in a car to gather telemetry about its location and the road it is driving on.  
This data is periodically written to an SD card.

## Phyiscal Setup

### Components

Consists of:

- Compute unit: Arduino Mega board
- Accelerometer: MPU-6050
- Gyroscope: MPU-6050 (same unit as above)
- Magnetometer: missing, future work
- GPS: GY-NEO6MV2
- Storage: micro SD card adapter

Further extended with Leds for informational use:

- Green led: blink on every read cycle finished of the Accelerometer and Magnetometer. After both finished, not for each separately.
- Blue led: blink on every read cycle finished of the GPS.
- White led: blinks at start of every new minute. Based on wall-clock time.
- Yellow led: blinks at start of every new second. Based on wall-clock time.
- Red led: indicator for bad states/failures.

Finally, there is a simple button that enables stop-start.
All of it wired together on a breadboard.

### Diagram

TODO

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

Also establishes our data collection buffer and allocates sufficient space for it. See the [data buffer](#data-buffer) section.

#### Acquire GPS fix

Acquire initial coordinates. In case of a cold start, may take some time.

#### Acquire time

Time is acquired by reading the GPS module.
It also captures the board time at the same moment, or as close as possible after it.
This to enable time tracking without having to acquire it every time from the GPS module.

#### SD card access.

The SD card test is done by writing the file `setup.txt`. It contains the epoch timestamp of the acquired time in the 3rd step.
It then reads the file back in to verify the writing was successful.

### Loop

The loop handles one or multiple tasks, depending on the iteration.
Every iteration start-to-end must finish within `loop_duration` milliseconds (default: 100).
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

Every `gps_interval` iterations:

- The GPS location will be acquired and stored in the data buffer.
- Turns on the blue led for this iteration.

On other iterations, it will store zeros for these data points in the data buffer.

#### Tasks: time acquisition

Every `gps_interval` iterations, the time will be acquired through the GPS. While this is listed as a separate task, for efficacy the GPS time acquisition happens together with location acquisition.
On every iteration, the current board time will be captured as well as the iteration counter of the loop. The counter starts at 1.

Based on these timings, it will also turn on leds:

- If a new wall clock minute started since the previous iteration, the white led is turned on.
- If a new wall clock second started since the previous iteration, the yellow led is turned on.
- If the iteration before this one did not finish within `loop_duration` milliseconds, the red led is turned on.

If the iteration before **did** finish within `loop_duration` milliseconds, the red led will be turned off.

#### Tasks: telemetry collection

Every `telemtry_interval` iterations:

- All accelerometer and gyroscope values are collected and stored in the data buffer.
- Turns on the green led.

#### Tasks: data writing

Every `write_interval` iterations, the data buffer is written to the SD card.
The filename format is: `<write_prefix>_yyyy-mm-dd_hhmmssSSS.atc` where the datetime with millisecond precision is UTC-based and constructed based on the oldest time in the data buffer.  
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

The data buffer is memory space set aside to accumulate the data being gathered until it is written to the SD card.

## Data Parser

The `parse-telemetry.py` script is a pure Python script that can read the written binary files and conver them into CSVs with a header.
It takes 2 positional arguments:

- `source`: Where to read from. Mandatory.
- `destination`: Where to write to. Optional. If not specified, the current working directory is assumed.

`source` can be a single file or a directory. If it is a directory, all files with the `.atc` extension in that directory are read.
`destination` can be a file path or a directory path. If it is a directory (or not specified), the file names will be kept, replacing the `.atc` extension with `.csv`.
If `source` is a directory, `destination` must also be a directory.
