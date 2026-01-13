#!/bin/bash
# Direct Laser Calibration Wizard Launcher
cd "$(dirname "$0")"

echo "=========================================="
echo "   Direct Laser Calibration Wizard"
echo "=========================================="
echo ""

# Use the virtual environment from the main project
VENV_PATH="../DesktopLaserController_ShapeModes/venv"

if [ -d "$VENV_PATH" ]; then
    echo "Activating virtual environment..."
    source "$VENV_PATH/bin/activate"
else
    echo "WARNING: No venv found at $VENV_PATH"
    echo "Trying system Python..."
fi

echo "Using Python: $(which python3)"
echo ""

python3 direct_calibration_wizard.py

# Keep terminal open if app crashes
if [ $? -ne 0 ]; then
    echo ""
    echo "App exited with error. Press any key to close..."
    read -n 1
fi
