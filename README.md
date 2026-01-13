# Laser Calibration Wizard

A streamlined tool for calibrating laser driver response curves. Creates Look-Up Tables (LUTs) that linearize perceived brightness for RGB laser systems.

![Calibration Wizard Screenshot](images/screenshot_main.png)
*Screenshot: Main calibration interface showing curve adjustment*

## What Does This Do?

Laser diodes don't respond linearly to PWM input - this tool lets you calibrate each channel (R/G/B) so that a 50% input produces 50% *perceived* brightness. Without calibration, your lasers may appear too dim at low values or jump suddenly in brightness.

### Features
- Per-channel calibration (Red, Green, Blue)
- Add custom calibration points where you need more detail
- Smooth (monotonic cubic Hermite) or linear interpolation
- Real-time preview with manual slider and linear sweep
- Export as C header file (.h) for ESP32
- Save/load calibration profiles as JSON

---

## Complete Installation Guide (First-Time Setup)

This guide assumes you've never installed Python or used the command line before. Follow each step carefully.

### Step 1: Install Python (macOS)

1. **Open Terminal**
   - Press `Cmd + Space` to open Spotlight
   - Type `Terminal` and press Enter
   - A black/white window will open - this is Terminal

2. **Check if Python is already installed**
   ```bash
   python3 --version
   ```
   - If you see `Python 3.x.x`, skip to Step 2
   - If you see `command not found`, continue below

3. **Install Homebrew** (macOS package manager)

   Copy and paste this entire line into Terminal, then press Enter:
   ```bash
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```
   - Follow the prompts (you may need to enter your password)
   - This takes a few minutes

4. **Install Python**
   ```bash
   brew install python
   ```

5. **Verify installation**
   ```bash
   python3 --version
   ```
   You should see `Python 3.11.x` or similar.

### Step 2: Download This Project

**Option A: Download ZIP (Easiest)**
1. Click the green **Code** button at the top of this page
2. Click **Download ZIP**
3. Extract the ZIP to a location you'll remember (e.g., Documents)

**Option B: Clone with Git (If you have git installed)**
```bash
cd ~/Documents
git clone https://github.com/Laser-Co/Laser_Calibration_Wizard.git
```

### Step 3: Install Required Libraries

1. **Open Terminal** (if not already open)

2. **Navigate to the project folder**
   ```bash
   cd ~/Documents/Laser_Calibration_Wizard
   ```
   (Adjust the path if you extracted it elsewhere)

3. **Create a virtual environment** (keeps things tidy)
   ```bash
   python3 -m venv venv
   ```

4. **Activate the virtual environment**
   ```bash
   source venv/bin/activate
   ```
   You should see `(venv)` at the start of your Terminal prompt.

5. **Install the required packages**
   ```bash
   pip install PyQt6 pyserial
   ```
   Wait for the installation to complete.

### Step 4: Run the Application

**Option A: From Terminal**
```bash
python3 direct_calibration_wizard.py
```

**Option B: Double-click the launcher**
1. In Finder, navigate to the project folder
2. Double-click `run_direct_wizard.command`
3. If you see a security warning:
   - Go to System Preferences → Security & Privacy
   - Click "Open Anyway"

The calibration wizard window should now appear!

### Troubleshooting Installation

**"command not found: python3"**
- Restart Terminal after installing Homebrew/Python
- Try: `brew install python` again

**"No module named PyQt6"**
- Make sure you activated the venv: `source venv/bin/activate`
- Reinstall: `pip install PyQt6 pyserial`

**Permission denied on .command file**
```bash
chmod +x run_direct_wizard.command
```

**"App can't be opened because it is from an unidentified developer"**
- Right-click the .command file → Open → Click "Open" in the dialog

---

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

---

## Usage Guide

### 1. Prepare ESP32

Before calibrating, upload the ESP32 driver with `USE_LUT false`:

```cpp
#define USE_LUT false  // Disable LUT for raw passthrough during calibration
```

This ensures you're calibrating against raw PWM response, not an old calibration.

### 2. Connect to Your Laser

1. Plug in your ESP32 via USB
2. In the wizard, select your ESP32's serial port from the dropdown
3. Click **Connect**
4. Status should show green "Connected"

![Connection Panel](images/screenshot_connect.png)

### 3. Calibrate Each Channel

For each channel (Red, Green, Blue):

1. **Set 0%** - This should be completely off (usually 0)
2. **Set 1%** - The PWM value where light just becomes visible
3. **Set 50%** - Adjust until it looks like half brightness
4. **Set 100%** - Usually maximum (65535 for 16-bit)
5. Use **+ Add** buttons to insert more points where needed
6. Use the **Test** button or slider to preview
7. Use **Linear Sweep** to verify smooth transitions

![Calibration Points](images/screenshot_calibration.png)

#### Tips
- **Smooth mode** (default) creates natural curves without overshoot
- **Linear mode** connects points with straight lines
- Add more points in problem areas (e.g., 5%, 10%, 25% for better low-end detail)

### 4. Export Your Calibration

1. Go to the **EXPORT** tab
2. Select LUT size:
   - **65536 (full 16-bit)** - recommended, no precision loss (~400KB file)
   - **4096** - smaller file, slight precision loss
3. Click **Save .h File**
4. Save as `laser_lut_smooth.h` in your ESP32 project folder

![Export Tab](images/screenshot_export.png)

### 5. Deploy to ESP32

1. Copy the exported `.h` file to your `esp32_laser_driver/` folder
2. Update the ESP32 code to enable LUT:
   ```cpp
   #define USE_LUT true  // Enable calibration LUT
   ```
3. Upload to ESP32
4. Your lasers now have calibrated brightness response!

---

## File Structure

```
Laser_Calibration_Wizard/
├── direct_calibration_wizard.py   # Main application
├── run_direct_wizard.command      # macOS launcher script
├── README.md                      # This file
├── images/                        # Screenshots for documentation
└── savedLUTs/
    ├── 12Bit/                     # 12-bit PWM calibration profiles
    │   ├── laser_lut_smooth.h
    │   └── smoothCalibration.json
    └── 16Bit/                     # 16-bit PWM calibration profiles
        ├── calibrationSmooth.h
        └── calibrationSmooth.json
```

---

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

---

## Saved Profiles

### 12-Bit (Legacy)
Located in `savedLUTs/12Bit/` - for older configurations using 12-bit PWM with 4096-entry LUTs.

### 16-Bit (Current)
Located in `savedLUTs/16Bit/` - full 16-bit resolution with 65536-entry LUTs for maximum precision.

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Laser too dim during calibration | Ensure `USE_LUT false` on ESP32 |
| Double-LUT effect (weird curves) | You're calibrating with `USE_LUT true` - disable it |
| No serial ports found | Check USB connection, install USB-serial drivers |
| Calibration looks wrong after deploy | Verify correct LUT size (65536 for 16-bit ESP32) |
| App won't open on macOS | Right-click → Open, or allow in Security preferences |

---

## License

MIT License - Feel free to use and modify for your projects.

---

## Contributing

Found a bug or want to add a feature? Pull requests welcome!
