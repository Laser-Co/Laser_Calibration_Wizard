# Laser Calibration Wizard

A streamlined tool for calibrating laser driver response curves. Creates Look-Up Tables (LUTs) that linearize perceived brightness for RGB laser systems.

## Overview

Laser diodes don't respond linearly to PWM input - this tool lets you calibrate each channel (R/G/B) so that a 50% input produces 50% *perceived* brightness.

### Features
- Per-channel calibration (Red, Green, Blue)
- Add custom calibration points where you need more detail
- Smooth (monotonic cubic Hermite) or linear interpolation
- Real-time preview with manual slider and linear sweep
- Export as C header file (.h) for ESP32
- Save/load calibration profiles as JSON

## Hardware Requirements

- ESP32 microcontroller
- RGB laser driver modules with PWM input
- USB connection between computer and ESP32

### Wiring
```
ESP32 GPIO 25 -> Red laser driver PWM input
ESP32 GPIO 26 -> Green laser driver PWM input
ESP32 GPIO 27 -> Blue laser driver PWM input
GND           -> Common ground
```

## Software Requirements

- Python 3.8+
- PyQt6
- pyserial

Install dependencies:
```bash
pip install PyQt6 pyserial
```

## Usage

### 1. Prepare ESP32

Before calibrating, upload the ESP32 driver with `USE_LUT false`:

```cpp
#define USE_LUT false  // Disable LUT for raw passthrough during calibration
```

This ensures you're calibrating against raw PWM response.

### 2. Run the Wizard

```bash
python direct_calibration_wizard.py
```

Or use the launcher:
```bash
./run_direct_wizard.command
```

### 3. Connect

1. Select your ESP32's serial port from the dropdown
2. Click **Connect**
3. Status should show green "Connected"

### 4. Calibrate Each Channel

For each channel (Red, Green, Blue):

1. Start with the **0%** and **1%** points - set the PWM value where light just becomes visible
2. Adjust **50%** to match perceived half-brightness
3. **100%** is typically max (65535 for 16-bit)
4. Use **+ Add** buttons to insert more calibration points where needed
5. Use the **Test** button or slider to preview each point
6. Use **Linear Sweep** to verify smooth transitions

#### Tips
- **Smooth mode** (default) creates natural curves without overshoot
- **Linear mode** connects points with straight lines
- Add more points in problem areas (e.g., 5%, 10%, 25% for better low-end detail)

### 5. Export

1. Go to the **EXPORT** tab
2. Select LUT size:
   - **65536 (full 16-bit)** - recommended, no precision loss (~400KB file)
   - **4096** - smaller file, slight precision loss
3. Click **Save .h File**
4. Save as `laser_lut_smooth.h` in your ESP32 project folder

### 6. Deploy

1. Copy the exported `.h` file to your `esp32_laser_driver/` folder
2. Update the ESP32 code to enable LUT:
   ```cpp
   #define USE_LUT true  // Enable calibration LUT
   ```
3. Upload to ESP32

## File Structure

```
laser_calibration_direct/
├── direct_calibration_wizard.py   # Main application
├── run_direct_wizard.command      # macOS launcher script
├── README.md                      # This file
└── savedLUTs/
    ├── 12Bit/                     # 12-bit PWM calibration profiles
    │   ├── laser_lut_smooth.h
    │   └── smoothCalibration.json
    └── 16Bit/                     # 16-bit PWM calibration profiles
        ├── calibrationSmooth.h
        └── calibrationSmooth.json
```

## Technical Details

### Data Flow
```
Input (0-65535) -> LUT[65536] -> Calibrated PWM (0-65535) -> Laser
```

### LUT Format
The exported `.h` file contains three arrays:
```cpp
const uint16_t RED_LUT[65536] PROGMEM = { ... };
const uint16_t GREEN_LUT[65536] PROGMEM = { ... };
const uint16_t BLUE_LUT[65536] PROGMEM = { ... };
```

### Serial Protocol
- Baud rate: 250000
- Packet: 6 bytes (3x uint16_t, little-endian)
- Format: `[R_low, R_high, G_low, G_high, B_low, B_high]`

### ESP32 PWM Configuration
- Resolution: 16-bit (65536 levels)
- Frequency: 1kHz
- Verified flicker-free for human perception

## Saved Profiles

### 12-Bit (Legacy)
Located in `savedLUTs/12Bit/` - for older configurations using 12-bit PWM with 4096-entry LUTs.

### 16-Bit (Current)
Located in `savedLUTs/16Bit/` - full 16-bit resolution with 65536-entry LUTs for maximum precision.

## Troubleshooting

**Laser too dim during calibration:**
- Ensure `USE_LUT false` on ESP32 during calibration

**Double-LUT effect (weird curves):**
- You're calibrating with `USE_LUT true` - disable it first

**No serial ports found:**
- Check USB connection
- Install appropriate USB-serial drivers for your ESP32

**Calibration doesn't look right after deployment:**
- Verify you exported the correct LUT size (65536 for 16-bit ESP32)
- Ensure ESP32 has `USE_LUT true` after uploading new LUT

## License

MIT License - Feel free to use and modify for your projects.
