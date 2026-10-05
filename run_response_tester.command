#!/bin/bash
# Standalone Laser Response Tester launcher
cd "$(dirname "$0")"

echo "=========================================="
echo "   Laser Response Tester (standalone)"
echo "=========================================="
echo ""

# Reuse a venv from the main LaserSystem projects if present.
for VENV in \
    "../LaserSystem_V4/venv" \
    "../LaserSystem_V3/venv" \
    "../DesktopLaserController_ShapeModes/venv"; do
    if [ -d "$VENV" ]; then
        echo "Activating venv: $VENV"
        # shellcheck disable=SC1090
        source "$VENV/bin/activate"
        break
    fi
done

echo "Using Python: $(which python3)"
echo ""

python3 laser_response_tester.py

if [ $? -ne 0 ]; then
    echo ""
    echo "App exited with error. Press any key to close..."
    read -n 1
fi
