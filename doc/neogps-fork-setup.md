# NeoGPS Fork Setup for HDOP Support

This document outlines how to properly enable HDOP (Horizontal Dilution of Precision) support in the NeoGPS library for the Arduino Telemetry Collector project.

## Problem Statement

By default, NeoGPS disables HDOP parsing to save memory. Our telemetry system requires HDOP data for GPS accuracy assessment, but modifying the library's configuration files directly:

- Affects all projects using NeoGPS on the system
- Gets overwritten when updating the library
- Makes the project non-portable
- Is not version controlled

## Solution: Fork and Patch Approach

### Step 1: Fork the NeoGPS Repository

1. **Navigate to the NeoGPS GitHub repository:**

   ```
   https://github.com/SlashDevin/NeoGPS
   ```

2. **Create a fork:**
   - Click the "Fork" button in the top-right corner
   - Select your GitHub account as the destination
   - This creates `https://github.com/yourusername/NeoGPS`

### Step 2: Clone Your Fork Locally

```bash
# Clone your fork to a temporary directory
cd ~/temp
git clone https://github.com/yourusername/NeoGPS.git
cd NeoGPS

# Add the original repository as upstream
git remote add upstream https://github.com/SlashDevin/NeoGPS.git
```

### Step 3: Create a Custom Configuration Branch

```bash
# Create a new branch for your customizations
git checkout -b arduino-telemetry-config

# Verify you're on the right branch
git branch
```

### Step 4: Enable HDOP in Configuration

Edit the configuration file to enable HDOP:

```bash
# Edit the GPS fix configuration
nano src/GPSfix_cfg.h
```

**Find line 45 (approximately) and change:**

```cpp
//#define GPS_FIX_HDOP
```

**To:**

```cpp
#define GPS_FIX_HDOP
```

### Step 5: Enable GSA Sentence Parsing

Edit the NMEA configuration to parse GSA sentences (required for DOP data):

```bash
# Edit the NMEA GPS configuration
nano src/NMEAGPS_cfg.h
```

**Find the GSA line (approximately line 35) and change:**

```cpp
//#define NMEAGPS_PARSE_GSA
```

**To:**

```cpp
#define NMEAGPS_PARSE_GSA
```

### Step 6: Commit Your Changes

```bash
# Stage all changes
git add .

# Commit with a descriptive message
git commit -m "Enable HDOP, VDOP, PDOP support and GSA parsing for Arduino Telemetry Collector

- Uncommented GPS_FIX_HDOP in GPSfix_cfg.h
- Uncommented GPS_FIX_VDOP in GPSfix_cfg.h
- Uncommented GPS_FIX_PDOP in GPSfix_cfg.h
- Uncommented NMEAGPS_PARSE_GSA in NMEAGPS_cfg.h
- Required for automotive telemetry accuracy assessment"

# Push to your fork
git push origin arduino-telemetry-config
```

### Step 7: Install the Forked Library

#### Option A: Replace the Existing Library

```bash
# Remove the original library
rm -rf ~/Documents/Arduino/libraries/NeoGPS

# Copy your customized version
cp -r ~/temp/NeoGPS ~/Documents/Arduino/libraries/NeoGPS

# Switch to your custom branch
cd ~/Documents/Arduino/libraries/NeoGPS
git checkout arduino-telemetry-config
```

#### Option B: Use Arduino CLI Library Installation

```bash
# Remove the original library through Arduino CLI
arduino-cli lib uninstall NeoGPS

# Install from your fork
cd ~/Documents/Arduino/libraries
git clone https://github.com/yourusername/NeoGPS.git
cd NeoGPS
git checkout arduino-telemetry-config
```

### Step 8: Update Arduino Code

Now you can use HDOP in your Arduino code:

```cpp
inline uint8_t writeGPS(byte8_t *writePtr) {
  if (!fix.valid.location) {
    return 0; // Failure: return 0 bytes written
  }

  float32_t *castPtr = (float32_t *)writePtr;
  castPtr[0] = fix.longitude();
  castPtr[1] = fix.latitude();

  // HDOP handling - NeoGPS stores HDOP as uint16_t * 1000
  if (fix.valid.hdop) {
    float hdopValue = fix.hdop / 1000.0; // Convert back to actual HDOP value
    uint8_t hdopByte = (hdopValue >= 25.5) ? 255 : (uint8_t)(hdopValue * 10); // Store as tenths
    writePtr[8] = hdopByte;
  } else {
    writePtr[8] = 0;
  }

  // Satellites handling
  if (fix.valid.satellites) {
    uint8_t satValue = fix.satellites;
    writePtr[9] = (satValue >= 255) ? 255 : satValue;
  } else {
    writePtr[9] = 0;
  }

  return 10; // Success: return bytes written
}
```

### Step 9: Test Compilation

```bash
cd /path/to/arduino-telemetry-collector
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 .
```

### Step 10: Document the Dependency

Create a `.arduino-deps.md` file in your project root:

```markdown
# Arduino Library Dependencies

## NeoGPS (Custom Fork)

- **Repository:** https://github.com/yourusername/NeoGPS
- **Branch:** arduino-telemetry-config
- **Modifications:**
  - Enabled GPS_FIX_HDOP for accuracy data
  - Enabled NMEAGPS_PARSE_GSA for DOP sentence parsing
- **Installation:** See doc/neogps-fork-setup.md
```

## Maintenance

### Updating Your Fork

Periodically sync with the upstream repository:

```bash
cd ~/Documents/Arduino/libraries/NeoGPS

# Fetch upstream changes
git fetch upstream

# Switch to main branch and merge
git checkout main
git merge upstream/main

# Rebase your custom branch
git checkout arduino-telemetry-config
git rebase main

# Push updates
git push origin arduino-telemetry-config --force-with-lease
```

### Alternative: Patch File Approach

If you prefer not to fork, you can maintain a patch file:

```bash
# Create a patch file
cd ~/Documents/Arduino/libraries/NeoGPS
git diff > ../../arduino-telemetry-collector/patches/neogps-hdop-enable.patch

# Apply the patch (after fresh library installation)
cd ~/Documents/Arduino/libraries/NeoGPS
git apply ../../arduino-telemetry-collector/patches/neogps-hdop-enable.patch
```

## Benefits of This Approach

1. **Version Control:** Your modifications are tracked in git
2. **Reproducible:** Anyone can set up the same environment
3. **Maintainable:** Easy to update and sync with upstream changes
4. **Portable:** Works across different development machines
5. **Documented:** Clear record of what was changed and why

## Drawbacks

1. **Setup Complexity:** More initial setup than direct modification
2. **Maintenance Overhead:** Need to sync with upstream periodically
3. **Git Knowledge Required:** Team members need basic git skills

## Recommended Workflow

For this project, we recommend the fork approach because:

- It's a long-term project requiring HDOP data
- The modifications are minimal and stable
- It provides the best balance of maintainability and portability
