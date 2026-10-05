#!/usr/bin/env python3
"""
Direct Laser Calibration Wizard

A streamlined tool for calibrating laser driver response curves.
Manually input PWM values for specific brightness percentages.
Add more detail points where needed.
"""

import sys
import json
import struct
import time
import math
from pathlib import Path
from typing import Optional, List, Dict

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QSlider, QSpinBox, QDoubleSpinBox,
    QGroupBox, QTabWidget, QFrame, QScrollArea, QLineEdit, QSizePolicy,
    QFileDialog, QTextEdit, QMessageBox, QCheckBox, QDial
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QPainter, QColor, QPen, QFont, QMouseEvent

import serial
import serial.tools.list_ports

# Optional: pull the same LaserBath_Controller (USB pot box) reader the main
# LaserSystem_V3 app uses. Located outside this folder, so we splice the V3
# tree onto sys.path before importing. Import is wrapped — if V3 isn't
# present, the wizard still runs, just without the pot-driven brightness
# feature.
import os
_V3_ROOT = os.path.expanduser(
    "~/Documents/Projects/code/LaserSystem_V3"
)
if os.path.isdir(_V3_ROOT) and _V3_ROOT not in sys.path:
    sys.path.insert(0, _V3_ROOT)
try:
    from shared.serial_controller import SerialController as LaserBathController
    LASERBATH_AVAILABLE = True
except Exception as _e:
    LaserBathController = None  # type: ignore
    LASERBATH_AVAILABLE = False
    print(f"[laserbath] SerialController unavailable: {_e}")


# =============================================================================
# Constants
# =============================================================================

PWM_MAX = 65535  # 16-bit PWM resolution (native 16-bit on ESP32)
PWM_BITS = 16
LUT_SIZE = 65536  # Full 16-bit indexing - no precision loss

# Pot-index dropdown defaults. Firmware exposes 12 channels but ships with
# indices 0,1,4..11 enabled (2,3 are unwired). We list all 12 anyway; the
# user picks whichever pot they want to drive the calibration test.
LASERBATH_POT_CHOICES = list(range(12))


# =============================================================================
# Serial Communication
# =============================================================================

class LaserSerial:
    """Simple serial connection to ESP32 laser driver."""

    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.port = None

    def list_ports(self) -> List[str]:
        ports = []
        for port in serial.tools.list_ports.comports():
            if 'usbmodem' in port.device.lower() or 'usbserial' in port.device.lower():
                ports.append(port.device)
        return sorted(ports)

    def connect(self, port: str, baud: int = 250000) -> bool:
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.port = port
            time.sleep(0.5)
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.send_rgb(0, 0, 0)
            self.ser.close()
        self.ser = None
        self.port = None

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def send_rgb(self, r: int, g: int, b: int):
        if not self.is_connected():
            return
        r = max(0, min(PWM_MAX, int(r)))
        g = max(0, min(PWM_MAX, int(g)))
        b = max(0, min(PWM_MAX, int(b)))
        # Per-channel tagged protocol.  Each channel value is preceded by
        # its own tag byte so the firmware verifies alignment at every
        # channel boundary.  9 bytes on the wire.
        data = struct.pack('<BHBHBH',
                           0xFA, r,
                           0xFB, g,
                           0xFC, b)
        try:
            self.ser.write(data)
        except:
            pass

    def send_channel(self, channel: str, value: int):
        """Send value to a specific channel only."""
        value = max(0, min(PWM_MAX, int(value)))
        if channel == 'red':
            self.send_rgb(value, 0, 0)
        elif channel == 'green':
            self.send_rgb(0, value, 0)
        elif channel == 'blue':
            self.send_rgb(0, 0, value)

    def send_command(self, cmd: str):
        """ASCII command line (e.g. 'LUT,0' / 'LUT,1' / 'LUT,?').

        Firmware accepts text commands terminated with '\\n'; the first byte
        of each transmission decides binary-vs-command mode at the ESP32 end.
        """
        if not self.is_connected():
            return
        try:
            self.ser.write((cmd.rstrip() + "\n").encode("ascii"))
        except Exception as e:
            print(f"[send_command] {e}")

    def query_lut_state(self, timeout_s: float = 0.5) -> Optional[bool]:
        """Send 'LUT,?' and parse the firmware's reply.

        Returns True if the firmware reports 'LUT,ON', False on 'LUT,OFF',
        or None on timeout / parse failure. The driver echoes 'LUT,ON' or
        'LUT,OFF' (see processCommand in teensy_laser_driver_alt.ino).

        Read is a tight poll on the serial port — short enough that the user
        doesn't notice (the firmware replies in microseconds), simple enough
        that we don't need a background thread for a one-shot query.
        """
        if not self.is_connected():
            return None
        try:
            # Drain any stale bytes so we don't mistake an old line for
            # the fresh reply.  Firmware also prints boot banners and the
            # channel walker may have left noise in the rx buffer.
            self.ser.reset_input_buffer()
            self.ser.write(b"LUT,?\n")
            deadline = time.monotonic() + timeout_s
            buf = bytearray()
            while time.monotonic() < deadline:
                chunk = self.ser.read(64)
                if chunk:
                    buf.extend(chunk)
                    # Scan all complete lines we have so far.
                    while b"\n" in buf:
                        nl = buf.index(b"\n")
                        line = bytes(buf[:nl]).decode(
                            "ascii", errors="replace"
                        ).strip()
                        del buf[: nl + 1]
                        if line == "LUT,ON":
                            return True
                        if line == "LUT,OFF":
                            return False
        except Exception as e:
            print(f"[query_lut_state] {e}")
        return None


# =============================================================================
# Channel Calibration Data
# =============================================================================

class ChannelCalibration:
    """Calibration data for a single channel."""

    def __init__(self, name: str):
        self.name = name
        self.threshold = 0
        self.use_smooth = True  # Use spline interpolation for smooth curves
        # Points: {percent: pwm_value}
        # Start with 0%, 1%, 50%, 100%
        self.points: Dict[int, int] = {
            0: 0,
            1: 0,
            50: PWM_MAX // 2,  # 32767
            100: PWM_MAX       # 65535
        }

    def set_point(self, percent: int, pwm_value: int):
        self.points[percent] = max(0, min(PWM_MAX, pwm_value))

    def remove_point(self, percent: int):
        if percent not in [0, 100]:  # Keep 0% and 100% endpoints
            self.points.pop(percent, None)

    def add_point_between(self, lower_percent: int, upper_percent: int):
        """Add a new point halfway between two existing points."""
        new_percent = (lower_percent + upper_percent) // 2
        if new_percent not in self.points and new_percent != lower_percent and new_percent != upper_percent:
            # Interpolate initial value
            lower_val = self.points.get(lower_percent, 0)
            upper_val = self.points.get(upper_percent, PWM_MAX)
            new_val = (lower_val + upper_val) // 2
            self.points[new_percent] = new_val
            return new_percent
        return None

    def get_sorted_percents(self) -> List[int]:
        return sorted(self.points.keys())

    def interpolate_linear(self, percent: float) -> int:
        """Linear interpolation between points."""
        sorted_pts = self.get_sorted_percents()

        if percent <= sorted_pts[0]:
            return self.points[sorted_pts[0]]
        if percent >= sorted_pts[-1]:
            return self.points[sorted_pts[-1]]

        if percent in self.points:
            return self.points[percent]

        lower = sorted_pts[0]
        upper = sorted_pts[-1]

        for p in sorted_pts:
            if p < percent:
                lower = p
            elif p > percent:
                upper = p
                break

        if upper == lower:
            return self.points[lower]

        t = (percent - lower) / (upper - lower)
        return int(self.points[lower] + t * (self.points[upper] - self.points[lower]))

    def interpolate_smooth(self, percent: float) -> int:
        """
        Monotonic cubic Hermite interpolation.
        Creates smooth curves that NEVER overshoot control points.
        """
        sorted_pts = self.get_sorted_percents()
        n = len(sorted_pts)

        if n < 2:
            return self.points.get(sorted_pts[0], 0) if sorted_pts else 0

        if percent <= sorted_pts[0]:
            return self.points[sorted_pts[0]]
        if percent >= sorted_pts[-1]:
            return self.points[sorted_pts[-1]]

        # Find the segment we're in
        seg_idx = 0
        for i in range(n - 1):
            if sorted_pts[i] <= percent <= sorted_pts[i + 1]:
                seg_idx = i
                break

        # Get x and y values for all points
        x_vals = sorted_pts
        y_vals = [self.points[p] for p in sorted_pts]

        # Calculate slopes (secants) between each pair of points
        deltas = []
        for i in range(n - 1):
            dx = x_vals[i + 1] - x_vals[i]
            dy = y_vals[i + 1] - y_vals[i]
            deltas.append(dy / dx if dx != 0 else 0)

        # Calculate tangents at each point using monotonic method
        tangents = [0.0] * n

        # First point: use one-sided difference
        tangents[0] = deltas[0]

        # Interior points: average of adjacent secants, but enforce monotonicity
        for i in range(1, n - 1):
            if deltas[i - 1] * deltas[i] <= 0:
                # Sign change or zero - flat tangent to prevent overshoot
                tangents[i] = 0
            else:
                # Harmonic mean of adjacent slopes (works better than arithmetic mean)
                tangents[i] = 2 / (1 / deltas[i - 1] + 1 / deltas[i])

        # Last point: use one-sided difference
        tangents[n - 1] = deltas[-1]  # Last delta (n-2 index)

        # Enforce monotonicity by limiting tangent magnitudes
        for i in range(n - 1):
            if deltas[i] == 0:
                tangents[i] = 0
                tangents[i + 1] = 0
            else:
                alpha = tangents[i] / deltas[i]
                beta = tangents[i + 1] / deltas[i]

                # Limit to circle of radius 3 to ensure monotonicity
                if alpha * alpha + beta * beta > 9:
                    tau = 3.0 / math.sqrt(alpha * alpha + beta * beta)
                    tangents[i] = tau * alpha * deltas[i]
                    tangents[i + 1] = tau * beta * deltas[i]

        # Now interpolate in the found segment using Hermite basis
        i = seg_idx
        x0, x1 = x_vals[i], x_vals[i + 1]
        y0, y1 = y_vals[i], y_vals[i + 1]
        m0, m1 = tangents[i], tangents[i + 1]

        h = x1 - x0
        t = (percent - x0) / h if h != 0 else 0
        t2 = t * t
        t3 = t2 * t

        # Hermite basis functions
        h00 = 2 * t3 - 3 * t2 + 1
        h10 = t3 - 2 * t2 + t
        h01 = -2 * t3 + 3 * t2
        h11 = t3 - t2

        result = h00 * y0 + h10 * h * m0 + h01 * y1 + h11 * h * m1

        # Clamp to segment bounds as extra safety
        min_val = min(y0, y1)
        max_val = max(y0, y1)
        result = max(min_val, min(max_val, result))

        return int(max(0, min(PWM_MAX, result)))

    def interpolate(self, percent: float) -> int:
        """Interpolate PWM value for any percent."""
        if self.use_smooth and len(self.points) >= 3:
            raw_value = self.interpolate_smooth(percent)
        else:
            raw_value = self.interpolate_linear(percent)

        # Apply threshold
        if self.threshold > 0 and percent > 0:
            usable_range = PWM_MAX - self.threshold
            raw_normalized = raw_value / PWM_MAX
            return int(self.threshold + raw_normalized * usable_range)

        return raw_value

    def generate_lut(self, size: int = 256) -> List[int]:
        lut = []
        for i in range(size):
            percent = (i / (size - 1)) * 100
            lut.append(self.interpolate(percent))
        return lut

    def to_dict(self) -> dict:
        return {
            'name': self.name,
            'threshold': self.threshold,
            'use_smooth': self.use_smooth,
            'points': {str(k): v for k, v in self.points.items()}
        }

    def from_dict(self, data: dict):
        self.name = data.get('name', self.name)
        self.threshold = data.get('threshold', 0)
        self.use_smooth = data.get('use_smooth', True)
        self.points = {int(k): v for k, v in data.get('points', {}).items()}


# =============================================================================
# Fine-Resolution Slider with Decoupled Drag
# =============================================================================

class FineSlider(QSlider):
    """QSlider with two upgrades over the stock widget:

      1. Native full 16-bit range (0..65535) instead of 0..100.  The wire
         protocol carries uint16 per channel, so the slider can address
         every output value directly — no quantization to 1%-steps.

      2. Decoupled mouse drag.  Stock QSlider sets the value to whatever
         pixel the mouse is over.  That feels twitchy at 16-bit range
         (one pixel = ~70 PWM counts on a 900-px-wide slider; one count
         is the whole bottom of the laser's curve).
         Instead we track the drag delta in pixels and translate it
         through a SPEED RATIO.  Holding Shift drops the ratio to a
         very fine 0.05× for sub-pixel control of the low end.

         Default speed (0.5×): dragging 100 px moves the slider 50 px
         worth of value.  This is the *normal* feel — already finer
         than stock.  Fine mode (Shift, 0.05×) is for finding the
         exact PWM count where the laser just starts to glow.
    """

    NORMAL_SPEED = 0.5
    FINE_SPEED   = 0.05  # ~1/10th of normal; pixel-accurate at 16-bit range

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # 16-bit range on the wire.  Setting the QSlider range to 0..65535
        # means valueChanged().value() is already the PWM value — no rescale.
        self.setRange(0, PWM_MAX)
        self.setValue(0)
        # Tick spacing is purely cosmetic at this resolution; we hide
        # ticks entirely to avoid a wall of fenceposts.
        self.setTickPosition(QSlider.TickPosition.NoTicks)
        # Page step is what the keyboard arrows + click-in-trough move by.
        # 256 = ~0.4% of range; fine enough to nudge by hand, coarse
        # enough that a few presses cross the full range.
        self.setPageStep(256)
        self.setSingleStep(1)

        # Drag state.  _drag_active is True between mousePressEvent on the
        # handle and mouseReleaseEvent.  We capture the press position and
        # the value-at-press so we can do (value = base + delta * speed)
        # on every move event.
        self._drag_active = False
        self._drag_anchor_px = 0
        self._drag_anchor_value = 0

    # ---- helpers -----------------------------------------------------

    def _speed(self) -> float:
        """Per-pixel value ratio for the current modifier state.

        Shift is the universal "precision mode" modifier on macOS; we
        reuse the convention here.  We DON'T check for Option/Cmd —
        keeping the modifier story to one key avoids confusion.
        """
        mods = QApplication.keyboardModifiers()
        if mods & Qt.KeyboardModifier.ShiftModifier:
            return self.FINE_SPEED
        return self.NORMAL_SPEED

    def _value_per_pixel(self) -> float:
        """How many slider units one pixel of drag would equal at 1×."""
        groove_px = max(1, self.width() - 16)  # subtract handle width
        return (self.maximum() - self.minimum()) / float(groove_px)

    # ---- mouse handling ---------------------------------------------

    def mousePressEvent(self, ev: QMouseEvent):
        # Left-click only — let right-click/middle-click pass through to
        # Qt for any context menu behavior.
        if ev.button() != Qt.MouseButton.LeftButton:
            return super().mousePressEvent(ev)
        # Don't snap-to-click.  Stock QSlider jumps the value to the
        # click position; we want the click to start a *relative* drag
        # from wherever the slider already is.  This makes fine
        # adjustment from the current value possible without re-grabbing.
        self._drag_active = True
        self._drag_anchor_px = ev.position().x()
        self._drag_anchor_value = self.value()
        ev.accept()

    def mouseMoveEvent(self, ev: QMouseEvent):
        if not self._drag_active:
            return super().mouseMoveEvent(ev)
        dx = ev.position().x() - self._drag_anchor_px
        # Convert pixel delta → value delta with the active speed ratio.
        # Note: this is the WHOLE point of the widget — by scaling dx
        # by less than 1.0× we make every pixel of mouse travel move the
        # value LESS than a stock slider would.  Fine adjustments require
        # more travel; coarse swings still happen, just with more wrist.
        delta_value = dx * self._value_per_pixel() * self._speed()
        new_value = int(round(self._drag_anchor_value + delta_value))
        new_value = max(self.minimum(), min(self.maximum(), new_value))
        if new_value != self.value():
            self.setValue(new_value)
        ev.accept()

    def mouseReleaseEvent(self, ev: QMouseEvent):
        if ev.button() == Qt.MouseButton.LeftButton and self._drag_active:
            self._drag_active = False
            ev.accept()
            return
        return super().mouseReleaseEvent(ev)

    def wheelEvent(self, ev):
        # Mouse wheel = step by 1/250th of full range per notch normally,
        # 1/2500th in fine mode.  Stock QSlider uses singleStep here which
        # at 16-bit range would be one PWM count per notch — useless for
        # ranging quickly across the slider.
        notches = ev.angleDelta().y() / 120.0
        full = self.maximum() - self.minimum()
        per_notch = full / (250.0 if self._speed() == self.NORMAL_SPEED else 2500.0)
        new_value = int(round(self.value() + notches * per_notch))
        new_value = max(self.minimum(), min(self.maximum(), new_value))
        if new_value != self.value():
            self.setValue(new_value)
        ev.accept()


# =============================================================================
# Curve Display Widget
# =============================================================================

class CurveDisplayWidget(QFrame):
    """Widget to visualize the calibration curve."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 200)
        self.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Sunken)
        self.calibration: Optional[ChannelCalibration] = None
        self.color = QColor(255, 100, 100)
        self.margin = 35
        self.sweep_position = -1  # -1 means not showing

    def set_calibration(self, cal: ChannelCalibration, color: QColor):
        self.calibration = cal
        self.color = color
        self.update()

    def set_sweep_position(self, percent: int):
        """Set the current sweep position to display (-1 to hide)."""
        self.sweep_position = percent
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        m = self.margin
        x, y = m, 15
        w = self.width() - m - 15
        h = self.height() - m - 15

        # Background
        painter.fillRect(x, y, w, h, QColor(25, 25, 25))

        # Grid
        painter.setPen(QPen(QColor(50, 50, 50), 1))
        for i in range(5):
            gx = x + (w * i // 4)
            gy = y + (h * i // 4)
            painter.drawLine(gx, y, gx, y + h)
            painter.drawLine(x, gy, x + w, gy)

        # Linear reference
        painter.setPen(QPen(QColor(60, 60, 60), 1, Qt.PenStyle.DashLine))
        painter.drawLine(x, y + h, x + w, y)

        # Draw curve
        if self.calibration:
            # Threshold line
            if self.calibration.threshold > 0:
                thresh_y = y + h - int((self.calibration.threshold / PWM_MAX) * h)
                painter.setPen(QPen(QColor(255, 150, 0), 1, Qt.PenStyle.DashLine))
                painter.drawLine(x, thresh_y, x + w, thresh_y)

            # If smooth mode, draw linear curve faintly for comparison
            if self.calibration.use_smooth and len(self.calibration.points) >= 3:
                faint_color = QColor(self.color.red() // 3, self.color.green() // 3, self.color.blue() // 3)
                painter.setPen(QPen(faint_color, 1, Qt.PenStyle.DotLine))
                prev_px, prev_py = None, None
                for i in range(101):
                    percent = i
                    value = self.calibration.interpolate_linear(percent)
                    px = x + int((percent / 100) * w)
                    py = y + h - int((value / PWM_MAX) * h)
                    if prev_px is not None:
                        painter.drawLine(prev_px, prev_py, px, py)
                    prev_px, prev_py = px, py

            # Main curve
            painter.setPen(QPen(self.color, 2))
            prev_px, prev_py = None, None

            for i in range(101):
                percent = i
                value = self.calibration.interpolate(percent)

                px = x + int((percent / 100) * w)
                py = y + h - int((value / PWM_MAX) * h)

                if prev_px is not None:
                    painter.drawLine(prev_px, prev_py, px, py)
                prev_px, prev_py = px, py

            # Draw calibration points
            for percent, value in self.calibration.points.items():
                px = x + int((percent / 100) * w)
                py = y + h - int((value / PWM_MAX) * h)

                painter.setPen(QPen(Qt.GlobalColor.white, 2))
                painter.setBrush(self.color)
                painter.drawEllipse(px - 5, py - 5, 10, 10)

            # Draw sweep position indicator
            if 0 <= self.sweep_position <= 100:
                sweep_x = x + int((self.sweep_position / 100) * w)
                sweep_value = self.calibration.interpolate(self.sweep_position)
                sweep_y = y + h - int((sweep_value / PWM_MAX) * h)

                # Vertical line
                painter.setPen(QPen(QColor(255, 255, 255, 100), 1, Qt.PenStyle.DashLine))
                painter.drawLine(sweep_x, y, sweep_x, y + h)

                # Dot on curve
                painter.setPen(QPen(Qt.GlobalColor.white, 2))
                painter.setBrush(QColor(255, 255, 0))
                painter.drawEllipse(sweep_x - 6, sweep_y - 6, 12, 12)

        # Labels
        painter.setPen(QColor(120, 120, 120))
        painter.setFont(QFont("Arial", 8))
        painter.drawText(x - 5, y + h + 12, "0%")
        painter.drawText(x + w - 20, y + h + 12, "100%")
        painter.drawText(x - 30, y + h, "0")
        painter.drawText(x - 30, y + 8, "65k")

        # Show interpolation mode
        mode_text = "Smooth" if (self.calibration and self.calibration.use_smooth) else "Linear"
        painter.drawText(x + w - 45, y + 12, mode_text)


# =============================================================================
# Point Entry Widget
# =============================================================================

class PointEntryWidget(QWidget):
    """Widget for entering a single calibration point."""

    value_changed = pyqtSignal(int, int)  # (percent, pwm_value)
    test_requested = pyqtSignal(int)  # percent
    remove_requested = pyqtSignal(int)  # percent

    def __init__(self, percent: int, pwm_value: int, removable: bool = True, parent=None):
        super().__init__(parent)
        self.percent = percent

        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)

        # Percent label
        self.percent_label = QLabel(f"{percent}%")
        self.percent_label.setMinimumWidth(45)
        self.percent_label.setStyleSheet("font-weight: bold; color: #aaa;")
        layout.addWidget(self.percent_label)

        layout.addWidget(QLabel("="))

        # PWM input.  Keyboard tracking stays ON (the Qt default) so the
        # laser gets a LIVE preview while the user is typing -- each
        # keystroke drives the laser output.  The lag we used to see at
        # this point in the chain wasn't from the spinbox itself; it was
        # the export tab regenerating three 65,536-entry LUTs and a
        # giant text dump on every change.  That heavy work is now
        # debounced ~150 ms upstream in DirectCalibrationWizard.
        # _on_calibration_changed, so live updates feel snappy.
        self.pwm_input = QSpinBox()
        self.pwm_input.setRange(0, PWM_MAX)
        self.pwm_input.setValue(pwm_value)
        self.pwm_input.setMinimumWidth(80)
        self.pwm_input.valueChanged.connect(self._on_value_changed)
        layout.addWidget(self.pwm_input)

        # Compact ±1/±5/±10 nudge buttons.  Each one bumps the spinbox by
        # its delta and lets the normal valueChanged path run (which is
        # now debounced upstream so rapid clicks coalesce cleanly).
        # Order from "less" → "more" so the buttons read like a number
        # line: −10 −5 −1 [box] +1 +5 +10.  Layout-wise, the −buttons live
        # BEFORE pwm_input in the parent layout; we already added pwm_input
        # above, so we have to reach into layout and reorder.
        nudge_style = (
            "QPushButton { padding: 0 4px; min-width: 22px; max-width: 28px;"
            "              font-size: 10px; }"
        )
        # Build buttons up-front; placement is via insertWidget below.
        self.nudge_buttons = []  # keep references; deltas captured in lambdas

        def _make_nudge(label: str, delta: int) -> QPushButton:
            b = QPushButton(label)
            b.setStyleSheet(nudge_style)
            b.clicked.connect(lambda _=None, d=delta: self._nudge(d))
            self.nudge_buttons.append(b)
            return b

        # Insert −10/−5/−1 BEFORE the pwm_input (at the spinbox's current
        # position in the layout, pushing it to the right).
        pwm_idx = layout.indexOf(self.pwm_input)
        layout.insertWidget(pwm_idx, _make_nudge("−10", -10))
        layout.insertWidget(pwm_idx + 1, _make_nudge("−5", -5))
        layout.insertWidget(pwm_idx + 2, _make_nudge("−1", -1))
        # The pwm_input now sits at pwm_idx + 3.  Append +1 / +5 / +10
        # right after it.
        after_pwm = pwm_idx + 4
        layout.insertWidget(after_pwm, _make_nudge("+1", 1))
        layout.insertWidget(after_pwm + 1, _make_nudge("+5", 5))
        layout.insertWidget(after_pwm + 2, _make_nudge("+10", 10))

        layout.addWidget(QLabel("PWM"))

        # Test button
        self.test_btn = QPushButton("Test")
        self.test_btn.setMaximumWidth(50)
        self.test_btn.clicked.connect(lambda: self.test_requested.emit(self.percent))
        layout.addWidget(self.test_btn)

        # Remove button
        if removable:
            self.remove_btn = QPushButton("X")
            self.remove_btn.setMaximumWidth(30)
            self.remove_btn.setStyleSheet("background: #663333;")
            self.remove_btn.clicked.connect(lambda: self.remove_requested.emit(self.percent))
            layout.addWidget(self.remove_btn)

        layout.addStretch()

    def _on_value_changed(self, value):
        self.value_changed.emit(self.percent, value)

    def _nudge(self, delta: int):
        """Bump the PWM value by `delta`, clamped to the spinbox range.
        Goes through the normal setValue → valueChanged path so the
        upstream debounce coalesces rapid clicks."""
        new = self.pwm_input.value() + delta
        lo, hi = self.pwm_input.minimum(), self.pwm_input.maximum()
        if new < lo:
            new = lo
        elif new > hi:
            new = hi
        if new != self.pwm_input.value():
            self.pwm_input.setValue(new)

    def set_value(self, value: int):
        self.pwm_input.blockSignals(True)
        self.pwm_input.setValue(value)
        self.pwm_input.blockSignals(False)

    def highlight(self, active: bool):
        if active:
            self.setStyleSheet("background: #333355; border-radius: 3px;")
        else:
            self.setStyleSheet("")


# =============================================================================
# Add Point Button
# =============================================================================

class AddPointRow(QWidget):
    """Row between two adjacent calibration points.  Holds an editable
    percent spinbox (pre-filled with the midpoint between the two
    surrounding points) plus an "+ Add" button.  Clicking the button
    adds a new calibration point AT the spinbox's value -- so the user
    can either accept the midpoint and click, or type any percent in
    the gap and then click.

    Emits add_requested(lower, upper, target_percent).
    """

    add_requested = pyqtSignal(int, int, int)  # lower, upper, target_percent

    def __init__(self, lower_percent: int, upper_percent: int, parent=None):
        super().__init__(parent)
        self.lower = lower_percent
        self.upper = upper_percent

        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)
        layout.setSpacing(4)

        # Editable percent input.  Range is constrained to the GAP between
        # the two existing points -- so a row between 25% and 50% only
        # accepts 26..49, preventing duplicate-key collisions and points
        # added outside the row's intended range.
        midpoint = (lower_percent + upper_percent) // 2
        self.percent_spin = QSpinBox()
        self.percent_spin.setRange(lower_percent + 1, upper_percent - 1)
        self.percent_spin.setValue(midpoint)
        self.percent_spin.setSuffix("%")
        self.percent_spin.setFixedWidth(70)
        self.percent_spin.setStyleSheet(
            "QSpinBox { background: #2a2a2a; color: #aaa;"
            " border: 1px dashed #444; padding: 2px; font-size: 10px; }"
        )
        layout.addWidget(self.percent_spin)

        self.add_btn = QPushButton("+ Add point")
        self.add_btn.setStyleSheet("""
            QPushButton {
                background: #2a2a2a;
                color: #666;
                border: 1px dashed #444;
                padding: 3px;
                font-size: 10px;
            }
            QPushButton:hover {
                background: #3a3a3a;
                color: #aaa;
                border-color: #666;
            }
        """)
        self.add_btn.clicked.connect(
            lambda: self.add_requested.emit(
                self.lower, self.upper, self.percent_spin.value()
            )
        )
        layout.addWidget(self.add_btn, 1)


# =============================================================================
# Pot knob widget (bottom row — M/R/G/B mirrors of the LaserBath pot box)
# =============================================================================

class _PotKnob(QWidget):
    """Compact knob column: letter label + dial + value + pot-index selector
    + MIDI-learn button.

    Used in the wizard's bottom row to give the user a permanent visual
    readout of each LaserBath_Controller pot's position, matching the
    M/R/G/B layout in the main LaserSystem app.  The dial itself is
    read-only display (driven by incoming pot events) — the user can't
    grab it with the mouse, because the manual test slider is the real
    input affordance.

    Pot binding can be set two ways:
      - The dropdown directly (manual)
      - The MAP button (click → wiggle a physical pot → that pot gets
        bound automatically — much easier than guessing pot indices)
    """

    learn_requested = pyqtSignal()  # MAP button clicked

    def __init__(self, label: str, color: str, default_pot: int, parent=None):
        super().__init__(parent)
        self._color = color
        self._label_text = label
        self._learning = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Letter label (M / R / G / B)
        self.letter_label = QLabel(label)
        self.letter_label.setStyleSheet(
            f"color: {color}; font-weight: bold; font-size: 12px; border: none;"
        )
        self.letter_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.letter_label)

        # Dial (display-only — read from pot events)
        self.dial = QDial()
        self.dial.setRange(0, 1000)
        self.dial.setValue(0)
        self.dial.setFixedSize(44, 44)
        self.dial.setNotchesVisible(True)
        # Read-only display: disable interaction so the user doesn't mistake
        # it for a mouse target.  The mouse-driven calibration knob is the
        # manual test slider above; these reflect the physical pots.
        self.dial.setEnabled(False)
        self.dial.setStyleSheet(
            f"QDial {{ background: {color}; }}"
            "QDial:disabled { background: " + color + "; }"  # keep colour when disabled
        )
        layout.addWidget(self.dial, alignment=Qt.AlignmentFlag.AlignCenter)

        # Live readout: percent on first line, PWM equivalent on second.
        # Showing both at a glance helps the user reason about exactly what
        # PWM count their pot position corresponds to — useful when matching
        # a pot's twist to a specific brightness target during calibration.
        self.value_label = QLabel("—\n—")
        self.value_label.setStyleSheet(
            f"color: {color}; font-size: 9px; border: none;"
        )
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.value_label.setFixedWidth(64)
        layout.addWidget(self.value_label)

        # Pot-index selector (which physical pot feeds this knob).  Kept as
        # a manual escape hatch — the MAP button below is the primary
        # binding tool, but the dropdown is still useful for offline
        # configuration when no pot box is plugged in.
        self.pot_combo = QComboBox()
        for i in LASERBATH_POT_CHOICES:
            self.pot_combo.addItem(f"pot {i}", i)
        idx = self.pot_combo.findData(default_pot)
        if idx >= 0:
            self.pot_combo.setCurrentIndex(idx)
        self.pot_combo.setFixedWidth(64)
        self.pot_combo.setStyleSheet(
            "QComboBox { font-size: 9px; padding: 1px 2px; }"
        )
        layout.addWidget(self.pot_combo, alignment=Qt.AlignmentFlag.AlignCenter)

        # MAP / Learn button — primary binding affordance.
        # Click → main window enters learn mode for this knob → next
        # physical pot the user wiggles is auto-bound to this knob.
        # Click again to cancel.  See _start_pot_learn / _dispatch_pot.
        self.learn_btn = QPushButton("MAP")
        self.learn_btn.setFixedWidth(64)
        self.learn_btn.setFixedHeight(18)
        self.learn_btn.setToolTip(
            "Click then wiggle a physical pot to bind it to this knob.\n"
            "Click again to cancel."
        )
        self._learn_btn_style_idle = (
            "QPushButton { background: #2a2a2a; color: #aaa; "
            "border: 1px solid #555; font-size: 9px; padding: 1px 4px; "
            "border-radius: 2px; }"
            "QPushButton:hover { background: #3a3a3a; color: #fff; }"
        )
        self._learn_btn_style_active = (
            "QPushButton { background: #ffaa00; color: #1a1a1a; "
            "border: 1px solid #ffd060; font-size: 9px; font-weight: bold; "
            "padding: 1px 4px; border-radius: 2px; }"
        )
        self.learn_btn.setStyleSheet(self._learn_btn_style_idle)
        self.learn_btn.clicked.connect(self.learn_requested)
        layout.addWidget(self.learn_btn, alignment=Qt.AlignmentFlag.AlignCenter)

    def pot_index(self) -> int:
        """Which physical pot drives this knob."""
        return self.pot_combo.currentData()

    def set_pot_index(self, pot: int):
        """Programmatically change which pot drives this knob (used by the
        MIDI-learn flow once a wiggling pot has been identified)."""
        idx = self.pot_combo.findData(pot)
        if idx >= 0:
            self.pot_combo.setCurrentIndex(idx)

    def set_value_normalized(self, value: float):
        """Update visual from a 0.0..1.0 pot value (from LaserBath controller).

        Negative values mean the pot is offline; show 'off' and leave the
        dial untouched so a brief dropout doesn't snap to zero.
        """
        if value < 0.0:
            self.value_label.setText("off\n—")
            return
        v = max(0.0, min(1.0, value))
        self.dial.setValue(int(round(v * 1000)))
        # Two-line readout: percent on top, raw 16-bit PWM target below.
        # PWM is what apply_pot_value() will push to the test slider, so
        # this is also a live preview of what the laser will receive.
        self.value_label.setText(f"{v*100:.1f}%\n{int(round(v*PWM_MAX))}")

    def set_learning(self, learning: bool):
        """Show or hide the learn-mode visual highlight."""
        self._learning = bool(learning)
        if self._learning:
            self.learn_btn.setText("…wiggle pot")
            self.learn_btn.setStyleSheet(self._learn_btn_style_active)
            # Outline the whole knob column to make it OBVIOUS which one
            # is listening, in case the user clicked one and looked away.
            self.setStyleSheet(
                "_PotKnob { border: 1px solid #ffaa00; border-radius: 4px; }"
            )
        else:
            self.learn_btn.setText("MAP")
            self.learn_btn.setStyleSheet(self._learn_btn_style_idle)
            self.setStyleSheet("")


# =============================================================================
# Channel Calibration Tab
# =============================================================================

class ChannelTab(QWidget):
    """Tab for calibrating a single channel."""

    calibration_changed = pyqtSignal()
    # Compare-to-other-channels: ask the parent wizard to fire the SAME
    # percent the user is currently viewing on this tab on a DIFFERENT
    # channel.  Used by the "Compare to:" row to scrub one color at a
    # time at a fixed percent so the user can eye-match brightness
    # across the three diodes.
    compare_requested = pyqtSignal(str, float)  # target_channel, percent

    def __init__(self, channel_name: str, color: QColor, laser: LaserSerial, parent=None):
        super().__init__(parent)
        self.channel_name = channel_name.lower()
        self.color = color
        self.laser = laser
        self.calibration = ChannelCalibration(channel_name)

        self.point_widgets: Dict[int, PointEntryWidget] = {}
        self.current_test_index = 0

        self._setup_ui()

    def _setup_ui(self):
        layout = QHBoxLayout(self)

        # LEFT: Curve display
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)

        self.curve_display = CurveDisplayWidget()
        self.curve_display.set_calibration(self.calibration, self.color)
        left_layout.addWidget(self.curve_display)

        # Interpolation mode toggle
        interp_layout = QHBoxLayout()
        interp_layout.addWidget(QLabel("Interpolation:"))

        self.smooth_btn = QPushButton("Smooth")
        self.smooth_btn.setCheckable(True)
        self.smooth_btn.setChecked(True)
        self.smooth_btn.clicked.connect(self._on_smooth_toggle)
        self.smooth_btn.setStyleSheet("""
            QPushButton { padding: 4px 12px; }
            QPushButton:checked { background: #0a8; color: white; }
        """)
        interp_layout.addWidget(self.smooth_btn)

        self.linear_btn = QPushButton("Linear")
        self.linear_btn.setCheckable(True)
        self.linear_btn.setChecked(False)
        self.linear_btn.clicked.connect(self._on_linear_toggle)
        self.linear_btn.setStyleSheet("""
            QPushButton { padding: 4px 12px; }
            QPushButton:checked { background: #08a; color: white; }
        """)
        interp_layout.addWidget(self.linear_btn)

        interp_layout.addStretch()
        left_layout.addLayout(interp_layout)

        # Quick Test group
        sweep_group = QGroupBox("Quick Test")
        sweep_layout = QVBoxLayout(sweep_group)

        # Manual sweep slider — operates in BRIGHTNESS PERCENT through the
        # calibration curve, not in raw PWM.  That means dragging the
        # slider to 25% lights the laser at whatever PWM you've defined
        # for "25% brightness" (e.g. 1479), exactly like the per-point
        # "Test" buttons and the "Jump Between Values" button do.  Same
        # for pot input — see apply_pot_value.
        #
        # This matches the main LaserSystem app's behavior, where source
        # values are mapped through the LUT before they hit the driver.
        # For raw-PWM probing (e.g. finding the exact PWM count where
        # the laser starts to glow), use the per-row PWM spinboxes on
        # the right -- they accept raw values and live-preview as you
        # type or click the ±1/±5/±10 buttons.
        manual_label = QLabel(
            "Manual Test (brightness % through calibration curve "
            "— hold Shift for fine drag):"
        )
        manual_label.setStyleSheet("color: #888; font-size: 11px;")
        sweep_layout.addWidget(manual_label)

        sweep_ctrl = QHBoxLayout()
        self.sweep_slider = FineSlider(Qt.Orientation.Horizontal)
        # Override the FineSlider's default 0..65535 (raw PWM) range to
        # 0..10000 representing 0.00..100.00% in hundredths.  Two-decimal
        # precision is plenty for hand-control of brightness; the FineSlider
        # decoupled-drag stays useful for finding the bottom-end shape.
        self.sweep_slider.setRange(0, 10000)
        self.sweep_slider.setValue(0)
        # Page-step scaled to the new range: ~0.4% per click in the
        # trough -- same proportional feel as the old 256/65535 page step.
        self.sweep_slider.setPageStep(40)
        self.sweep_slider.valueChanged.connect(self._on_sweep_change)
        sweep_ctrl.addWidget(self.sweep_slider)

        # Numeric readout: percent (0.00..100.00) instead of raw PWM.
        # User can type a target percent and the slider follows.  The
        # resulting interpolated PWM is shown in the label to the right
        # so you can verify what the laser will actually receive.
        self.sweep_value_input = QDoubleSpinBox()
        self.sweep_value_input.setRange(0.0, 100.0)
        self.sweep_value_input.setDecimals(2)
        self.sweep_value_input.setSingleStep(0.5)
        self.sweep_value_input.setSuffix(" %")
        self.sweep_value_input.setValue(0.0)
        self.sweep_value_input.setMinimumWidth(90)
        self.sweep_value_input.valueChanged.connect(self._on_sweep_input_change)
        sweep_ctrl.addWidget(self.sweep_value_input)

        # Resulting interpolated PWM (what the laser actually receives).
        # Shown next to the percent input so the user can see both numbers.
        self.sweep_label = QLabel("→ PWM 0")
        self.sweep_label.setMinimumWidth(90)
        self.sweep_label.setStyleSheet("color: #888; font-size: 11px;")
        sweep_ctrl.addWidget(self.sweep_label)

        sweep_layout.addLayout(sweep_ctrl)

        # LaserBath Controller pot input — when the USB pot box is plugged
        # in, the user can route one of its pots straight into this
        # channel's manual test slider, exactly as the main LaserSystem app
        # consumes pots.  Off by default; the connection is opened by the
        # main window and *shared* across all three channel tabs.
        controller_layout = QHBoxLayout()
        self.use_controller_cb = QCheckBox("Use LaserBath pot:")
        self.use_controller_cb.setToolTip(
            "Drive this channel's brightness from a LaserBath_Controller pot.\n"
            "Mouse slider above still works; whichever moves last wins.\n"
            "Pot box must be plugged in — see the status indicator at the top."
        )
        self.use_controller_cb.toggled.connect(self._on_controller_toggle)
        controller_layout.addWidget(self.use_controller_cb)

        self.controller_pot_combo = QComboBox()
        # Firmware enables 10 of 12 pots (2,3 are unwired).  Show all 12 so
        # rewiring later doesn't force a UI change; offline pots will just
        # never fire pot_changed.
        for i in LASERBATH_POT_CHOICES:
            self.controller_pot_combo.addItem(f"pot {i}", i)
        # Defaults: 0=Red, 1=Green, 4=Blue.  Picked from POT_ENABLED in the
        # firmware so the wizard's defaults map to known-wired channels.
        default_pot = {"red": 0, "green": 1, "blue": 4}.get(self.channel_name, 0)
        idx = self.controller_pot_combo.findData(default_pot)
        if idx >= 0:
            self.controller_pot_combo.setCurrentIndex(idx)
        self.controller_pot_combo.setEnabled(False)
        controller_layout.addWidget(self.controller_pot_combo)
        controller_layout.addStretch()
        sweep_layout.addLayout(controller_layout)

        # Index of the pot we're currently listening to (None = no binding).
        self._bound_pot_index: Optional[int] = None

        # Jump test button
        self.jump_btn = QPushButton("Jump Between Values (Space)")
        self.jump_btn.clicked.connect(self._jump_to_next)
        sweep_layout.addWidget(self.jump_btn)

        # ── Compare-to-other-channels row ─────────────────────────────────
        # Workflow: while dialing in this channel's curve at some percent,
        # the user wants to see what the OTHER channels look like at the
        # same percent so they can eye-match brightness across colors.
        # The toggles pick which other channels are in the rotation;
        # clicking "Compare ▶" fires the next enabled target (and zeroes
        # the other two), so repeated clicks cycle through the enabled
        # set one channel at a time -- single-color comparison by design.
        compare_layout = QHBoxLayout()
        compare_layout.addWidget(QLabel("Compare to:"))

        # Which two colors are "other" for this tab.  Order matters --
        # it's the rotation cycle clicking the button walks through.
        OTHER_FOR = {
            "red":   ["green", "blue"],
            "green": ["red",   "blue"],
            "blue":  ["red",   "green"],
        }
        LETTER = {"red": "R", "green": "G", "blue": "B"}
        TINT   = {"red": "#ff4444", "green": "#44ff44", "blue": "#4488ff"}

        self.compare_toggles: Dict[str, QPushButton] = {}
        for other in OTHER_FOR.get(self.channel_name, []):
            t = QPushButton(LETTER[other])
            t.setCheckable(True)
            t.setChecked(False)
            t.setFixedSize(28, 22)
            tint = TINT[other]
            # Two-state style: dim when off, channel-coloured when on.
            t.setStyleSheet(
                "QPushButton { background: #2a2a2a; color: #888;"
                " border: 1px solid #444; border-radius: 3px;"
                " font-weight: bold; font-size: 11px; }"
                f"QPushButton:checked {{ background: {tint};"
                f" color: #000; border: 1px solid {tint}; }}"
            )
            t.toggled.connect(self._on_compare_toggle_changed)
            self.compare_toggles[other] = t
            compare_layout.addWidget(t)

        self.compare_btn = QPushButton("Compare ▶ (none)")
        self.compare_btn.setEnabled(False)
        self.compare_btn.setToolTip(
            "Fire the next enabled channel at the current viewing percent.\n"
            "Click repeatedly to cycle through enabled channels one at a time."
        )
        self.compare_btn.clicked.connect(self._on_compare_clicked)
        compare_layout.addWidget(self.compare_btn)
        compare_layout.addStretch()
        sweep_layout.addLayout(compare_layout)

        # State: which enabled toggle fires on the NEXT button click.
        # Reset to 0 whenever the toggle set changes (a fresh selection
        # always starts at the first enabled channel).
        self._compare_index = 0

        # Linear sweep section
        sweep_layout.addWidget(QLabel(""))  # Spacer
        linear_label = QLabel("Linear Sweep:")
        linear_label.setStyleSheet("color: #888; font-size: 11px;")
        sweep_layout.addWidget(linear_label)

        # Direction and mode
        dir_layout = QHBoxLayout()

        dir_layout.addWidget(QLabel("Direction:"))
        self.direction_combo = QComboBox()
        self.direction_combo.addItems(["Loop ↔", "Forward →", "Reverse ←"])
        self.direction_combo.setMinimumWidth(100)
        dir_layout.addWidget(self.direction_combo)

        dir_layout.addWidget(QLabel("Mode:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Continuous", "Single Shot"])
        self.mode_combo.setMinimumWidth(100)
        dir_layout.addWidget(self.mode_combo)

        dir_layout.addStretch()
        sweep_layout.addLayout(dir_layout)

        # Speed slider
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Speed:"))

        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(30)
        speed_layout.addWidget(self.speed_slider)

        self.speed_label = QLabel("30")
        self.speed_label.setMinimumWidth(30)
        self.speed_slider.valueChanged.connect(lambda v: self.speed_label.setText(str(v)))
        speed_layout.addWidget(self.speed_label)

        sweep_layout.addLayout(speed_layout)

        # Start/Stop button and progress
        run_layout = QHBoxLayout()

        self.sweep_btn = QPushButton("▶ Start Sweep")
        self.sweep_btn.setCheckable(True)
        self.sweep_btn.clicked.connect(self._toggle_sweep)
        self.sweep_btn.setMinimumWidth(120)
        run_layout.addWidget(self.sweep_btn)

        self.sweep_progress = QSlider(Qt.Orientation.Horizontal)
        self.sweep_progress.setRange(0, 100)
        self.sweep_progress.setValue(0)
        self.sweep_progress.setEnabled(False)
        run_layout.addWidget(self.sweep_progress)

        self.sweep_percent_label = QLabel("0%")
        self.sweep_percent_label.setMinimumWidth(40)
        run_layout.addWidget(self.sweep_percent_label)

        sweep_layout.addLayout(run_layout)

        # Sweep animation state
        self._sweep_running = False
        self._sweep_position = 0.0
        self._sweep_direction = 1  # 1 = forward, -1 = reverse
        self._sweep_timer = QTimer()
        self._sweep_timer.timeout.connect(self._sweep_tick)

        left_layout.addWidget(sweep_group)
        layout.addWidget(left_widget, stretch=2)

        # RIGHT: Point entries
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)

        # Threshold
        thresh_group = QGroupBox("Threshold (where light starts)")
        thresh_layout = QHBoxLayout(thresh_group)

        self.threshold_input = QSpinBox()
        self.threshold_input.setRange(0, 5000)
        self.threshold_input.setValue(0)
        self.threshold_input.valueChanged.connect(self._on_threshold_changed)
        thresh_layout.addWidget(self.threshold_input)

        self.thresh_test_btn = QPushButton("Find")
        self.thresh_test_btn.clicked.connect(self._find_threshold)
        thresh_layout.addWidget(self.thresh_test_btn)

        thresh_layout.addStretch()
        right_layout.addWidget(thresh_group)

        # Points header
        header = QLabel("Brightness Points (% = PWM value)")
        header.setStyleSheet("font-weight: bold; color: #aaa; padding: 5px;")
        right_layout.addWidget(header)

        # Scrollable points area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        self.points_container = QWidget()
        self.points_layout = QVBoxLayout(self.points_container)
        self.points_layout.setSpacing(2)
        self.points_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        scroll.setWidget(self.points_container)
        right_layout.addWidget(scroll)

        layout.addWidget(right_widget, stretch=1)

        # Build initial points
        self._rebuild_points_ui()

    def _rebuild_points_ui(self):
        """Rebuild the points list UI."""
        # Clear existing
        while self.points_layout.count():
            item = self.points_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        self.point_widgets.clear()

        # Add points with "add" buttons between them
        sorted_percents = self.calibration.get_sorted_percents()

        for i, percent in enumerate(sorted_percents):
            pwm_value = self.calibration.points[percent]

            # Add point widget
            removable = percent not in [0, 100]  # Can't remove 0% and 100%
            point_widget = PointEntryWidget(percent, pwm_value, removable)
            point_widget.value_changed.connect(self._on_point_value_changed)
            point_widget.test_requested.connect(self._on_test_point)
            point_widget.remove_requested.connect(self._on_remove_point)

            self.point_widgets[percent] = point_widget
            self.points_layout.addWidget(point_widget)

            # Add "add point" row between this and next.  Includes an
            # editable percent spinbox so the user can pick any value in
            # the gap, not just the auto-suggested midpoint.
            if i < len(sorted_percents) - 1:
                next_percent = sorted_percents[i + 1]
                if next_percent - percent > 1:  # Only if there's room
                    add_row = AddPointRow(percent, next_percent)
                    add_row.add_requested.connect(self._on_add_point)
                    self.points_layout.addWidget(add_row)

        self.points_layout.addStretch()

    def _on_point_value_changed(self, percent: int, value: int):
        # Update the calibration model first.
        self.calibration.set_point(percent, value)
        self.curve_display.update()
        # Live laser preview: send the raw PWM value the user just typed
        # / nudged so they can see the laser's response in real time.
        # Sending the RAW value (not the interpolated curve output) is
        # intentional -- the user is editing the setpoint for THIS point,
        # so they want to see what the laser does at exactly that PWM.
        self.laser.send_channel(self.channel_name, int(value))
        # Highlight the row being edited so it's visually clear which
        # point is currently driving the laser.
        for p, widget in self.point_widgets.items():
            widget.highlight(p == percent)
        # Curve display indicator follows the percent of the point we're
        # editing -- gives a vertical guide line at the right x-position.
        self.curve_display.set_sweep_position(percent)
        # Notify parent (export tab refresh, etc.) -- debounced upstream.
        self.calibration_changed.emit()

    def _on_test_point(self, percent: int):
        value = self.calibration.interpolate(percent)
        self.laser.send_channel(self.channel_name, value)
        # Manual slider is in percent units (0..10000 = 0..100%); set it to
        # the percent of the point we're testing so the visual indicator
        # lands exactly under the row.  Block both signals so the slider→
        # spinbox→laser cycle doesn't double-send.
        self.sweep_slider.blockSignals(True)
        self.sweep_value_input.blockSignals(True)
        self.sweep_slider.setValue(percent * 100)
        self.sweep_value_input.setValue(float(percent))
        self.sweep_slider.blockSignals(False)
        self.sweep_value_input.blockSignals(False)
        self.sweep_label.setText(f"→ PWM {value}")
        # Curve display still uses percent so the indicator sits on the
        # control point the user pressed.
        self.curve_display.set_sweep_position(percent)

        # Highlight current
        for p, widget in self.point_widgets.items():
            widget.highlight(p == percent)

    def _on_remove_point(self, percent: int):
        self.calibration.remove_point(percent)
        self._rebuild_points_ui()
        self.curve_display.update()
        self.calibration_changed.emit()

    def _on_add_point(self, lower: int, upper: int, target_percent: int):
        """Add a new calibration point at ``target_percent``, which the
        user picked via the spinbox in the AddPointRow (or accepted as
        the auto-suggested midpoint).  Initial PWM value is linearly
        interpolated between the two surrounding points so the curve
        doesn't move when the user adds an anchor -- only when they
        edit it afterward.
        """
        if target_percent <= lower or target_percent >= upper:
            return
        if target_percent in self.calibration.points:
            return  # already exists; nothing to add
        lower_val = self.calibration.points.get(lower, 0)
        upper_val = self.calibration.points.get(upper, PWM_MAX)
        # Linear interp by percent within the gap -- matches what the
        # original add_point_between() did for midpoints.
        t = (target_percent - lower) / float(upper - lower)
        new_val = int(round(lower_val + t * (upper_val - lower_val)))
        self.calibration.set_point(target_percent, new_val)
        self._rebuild_points_ui()
        self.curve_display.update()
        self.calibration_changed.emit()

    def _on_threshold_changed(self, value: int):
        self.calibration.threshold = value
        self.curve_display.update()
        self.calibration_changed.emit()

    def _find_threshold(self):
        """Open a simple threshold finder."""
        # For now, just do a slow sweep from 0
        self._threshold_value = 0
        self._threshold_timer = QTimer()
        self._threshold_timer.timeout.connect(self._threshold_tick)
        self._threshold_timer.start(50)
        self.thresh_test_btn.setText("Finding...")
        self.thresh_test_btn.setEnabled(False)

    def _threshold_tick(self):
        self._threshold_value += 10
        self.laser.send_channel(self.channel_name, self._threshold_value)
        self.threshold_input.setValue(self._threshold_value)

        if self._threshold_value >= 2000:
            self._threshold_timer.stop()
            self.thresh_test_btn.setText("Find")
            self.thresh_test_btn.setEnabled(True)
            self.laser.send_channel(self.channel_name, 0)

    def _on_smooth_toggle(self, checked: bool):
        """Switch to smooth interpolation."""
        if checked:
            self.calibration.use_smooth = True
            self.linear_btn.setChecked(False)
            self.curve_display.update()
            self.calibration_changed.emit()

    def _on_linear_toggle(self, checked: bool):
        """Switch to linear interpolation."""
        if checked:
            self.calibration.use_smooth = False
            self.smooth_btn.setChecked(False)
            self.curve_display.update()
            self.calibration_changed.emit()

    def _on_sweep_change(self, slider_value: int):
        """Manual slider moved — interpret position as brightness percent,
        run it through the calibration curve, and send the resulting PWM
        to the laser.

        Slider is in millipercent (0..10000 = 0.00..100.00%), so we just
        divide by 100 to get the percent and let self.calibration.interpolate
        do the curve math.  This is the same path the per-point Test
        buttons and Jump Between Values use, so the slider, pot, and
        Test buttons all produce identical PWM at identical percents.
        """
        percent = max(0.0, min(100.0, slider_value / 100.0))
        pwm = int(self.calibration.interpolate(percent))
        self.laser.send_channel(self.channel_name, pwm)
        # Mirror to spinbox without re-firing valueChanged → avoids feedback.
        self.sweep_value_input.blockSignals(True)
        self.sweep_value_input.setValue(percent)
        self.sweep_value_input.blockSignals(False)
        # Show the interpolated PWM the laser is actually receiving so the
        # user can verify the curve's effect at this percent.
        self.sweep_label.setText(f"→ PWM {pwm}")
        # Curve display indicator tracks the same percent — vertical
        # dashed line at this X on the curve.
        self.curve_display.set_sweep_position(int(round(percent)))

    def _on_sweep_input_change(self, percent_value: float):
        """Percent spinbox edited — push value back into the slider
        (which then cascades through _on_sweep_change to fire the laser
        and update the label)."""
        target = int(round(percent_value * 100))
        target = max(self.sweep_slider.minimum(),
                     min(self.sweep_slider.maximum(), target))
        if self.sweep_slider.value() != target:
            self.sweep_slider.setValue(target)

    # ---- Compare-to-other-channels ---------------------------------------

    def _compare_cycle(self) -> List[str]:
        """Build the rotation cycle the Compare button walks through.

        Each enabled comparison target is INTERLEAVED with the home
        channel (this tab's own colour), so a single button press goes:
        target → home → next target → home → ...  That makes A/B
        brightness matching at a fixed percent easy: tap once to see the
        comparison colour, tap again to flip back to the home colour,
        tap again to step to the next comparison, and so on.

        Returns [] when no toggles are enabled, which disables the button.
        """
        enabled = [c for c, btn in self.compare_toggles.items() if btn.isChecked()]
        if not enabled:
            return []
        cycle: List[str] = []
        for t in enabled:
            cycle.append(t)
            cycle.append(self.channel_name)  # return to home after each target
        return cycle

    def _on_compare_toggle_changed(self, _checked: bool):
        """User flipped one of the R/G/B toggles -- reset the rotation
        index so the next Compare click starts from the first enabled
        target, and refresh the button's label/enabled state."""
        self._compare_index = 0
        self._refresh_compare_btn()

    def _refresh_compare_btn(self):
        """Update the Compare button's text to preview which channel
        will fire on the next click, and disable it when nothing is
        enabled to compare to."""
        cycle = self._compare_cycle()
        if not cycle:
            self.compare_btn.setText("Compare ▶ (none)")
            self.compare_btn.setEnabled(False)
            return
        next_ch = cycle[self._compare_index % len(cycle)]
        self.compare_btn.setText(f"Compare ▶ {next_ch.title()}")
        self.compare_btn.setEnabled(True)

    def _on_compare_clicked(self):
        """Fire the NEXT step in the rotation at the current viewing
        percent.  Steps alternate target → home → target → home, so
        the user sees the comparison colour then snaps back to home
        on the very next click."""
        cycle = self._compare_cycle()
        if not cycle:
            return
        target = cycle[self._compare_index % len(cycle)]
        # Slider is in millipercent (0..10000) -> percent is /100.
        percent = self.sweep_slider.value() / 100.0
        self.compare_requested.emit(target, percent)
        self._compare_index = (self._compare_index + 1) % len(cycle)
        self._refresh_compare_btn()

    def _on_controller_toggle(self, checked: bool):
        """User flipped 'Use LaserBath pot:' — enable/disable the dropdown."""
        self.controller_pot_combo.setEnabled(checked)
        # No need to wire/unwire here — the central pot-routing in
        # DirectCalibrationWizard inspects each tab on every pot event.
        # We only flip enabled state for visual feedback.

    def is_listening_to_pot(self, pot_index: int) -> bool:
        """Used by the main window's pot-routing dispatcher to decide whether
        to drive this tab from a given pot index. True only when:
          - the 'Use LaserBath pot' checkbox is on
          - the dropdown is set to pot_index
        """
        return (
            self.use_controller_cb.isChecked()
            and self.controller_pot_combo.currentData() == pot_index
        )

    def apply_pot_value(self, value: float):
        """A bound LaserBath pot reports a new normalized value (0.0..1.0).

        The slider now operates in brightness-percent, so we just rescale
        0..1 to the slider's 0..10000 range and let the slider's own
        valueChanged → _on_sweep_change → curve-interpolation pipeline
        do the rest.  Pot, slider drag, spinbox typing, Test buttons,
        Jump Between Values, and the Linear Sweep all converge on the
        same interpolate() call.
        """
        if value < 0.0:
            return  # pot offline — ignore
        target = int(round(max(0.0, min(1.0, value)) * self.sweep_slider.maximum()))
        if self.sweep_slider.value() != target:
            self.sweep_slider.setValue(target)

    def _toggle_sweep(self, checked: bool):
        """Start or stop the linear sweep."""
        if checked:
            self._sweep_running = True
            self.sweep_btn.setText("⏹ Stop Sweep")

            # Set initial position based on direction
            direction_text = self.direction_combo.currentText()
            if "Reverse" in direction_text:
                self._sweep_position = 100.0
                self._sweep_direction = -1
            else:
                self._sweep_position = 0.0
                self._sweep_direction = 1

            # Start timer at ~50Hz, speed slider affects step size
            self._sweep_timer.start(20)
        else:
            self._stop_sweep()

    def _stop_sweep(self):
        """Stop the sweep and reset."""
        self._sweep_running = False
        self._sweep_timer.stop()
        self.sweep_btn.setChecked(False)
        self.sweep_btn.setText("▶ Start Sweep")
        self.laser.send_channel(self.channel_name, 0)
        self.curve_display.set_sweep_position(-1)  # Hide indicator

    def _sweep_tick(self):
        """Update sweep position on each timer tick."""
        if not self._sweep_running:
            return

        # Calculate step size based on speed (higher = faster)
        speed = self.speed_slider.value()
        step = speed * 0.02  # Range roughly 0.02 to 2.0 per tick

        # Update position
        self._sweep_position += step * self._sweep_direction

        direction_text = self.direction_combo.currentText()
        mode_text = self.mode_combo.currentText()

        # Handle boundaries
        if self._sweep_position >= 100:
            self._sweep_position = 100

            if "Loop" in direction_text:
                # Bounce back
                self._sweep_direction = -1
            elif "Single" in mode_text:
                # Stop at end
                self._stop_sweep()
                return
            else:
                # Continuous forward: wrap to start
                self._sweep_position = 0

        elif self._sweep_position <= 0:
            self._sweep_position = 0

            if "Loop" in direction_text:
                # Bounce forward
                self._sweep_direction = 1
            elif "Single" in mode_text:
                # Stop at start
                self._stop_sweep()
                return
            else:
                # Continuous reverse: wrap to end
                self._sweep_position = 100

        # Send value to laser
        percent = int(self._sweep_position)
        value = self.calibration.interpolate(percent)
        self.laser.send_channel(self.channel_name, value)

        # Update UI
        self.sweep_progress.setValue(percent)
        self.sweep_percent_label.setText(f"{percent}%")
        self.curve_display.set_sweep_position(percent)

    def _jump_to_next(self):
        """Jump to the next calibration point."""
        sorted_percents = self.calibration.get_sorted_percents()
        if not sorted_percents:
            return

        self.current_test_index = (self.current_test_index + 1) % len(sorted_percents)
        percent = sorted_percents[self.current_test_index]

        self._on_test_point(percent)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Space:
            self._jump_to_next()
        else:
            super().keyPressEvent(event)

    def stop(self):
        self.laser.send_channel(self.channel_name, 0)
        if hasattr(self, '_threshold_timer'):
            self._threshold_timer.stop()
        if hasattr(self, '_sweep_timer'):
            self._sweep_timer.stop()
            self._sweep_running = False


# =============================================================================
# Export Tab
# =============================================================================

class ExportTab(QWidget):
    """Tab for exporting calibration data."""

    # Fires after a successful Load-Calibration JSON read.  Updating the
    # underlying ChannelCalibration objects doesn't auto-refresh the
    # per-tab UI (the Brightness Points list, threshold spinbox, and
    # Smooth/Linear toggle are built ONCE in ChannelTab._setup_ui and
    # only rebuilt when the user adds/removes a point), so we ask the
    # main window to push the fresh state into each tab via this signal.
    calibration_loaded = pyqtSignal()

    def __init__(self, red_cal: ChannelCalibration, green_cal: ChannelCalibration,
                 blue_cal: ChannelCalibration, parent=None):
        super().__init__(parent)
        self.red_cal = red_cal
        self.green_cal = green_cal
        self.blue_cal = blue_cal
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        # LUT size
        size_layout = QHBoxLayout()
        size_layout.addWidget(QLabel("LUT Size:"))
        self.size_combo = QComboBox()
        self.size_combo.addItems(["256", "1024", "4096", "65536 (full 16-bit)"])
        self.size_combo.setCurrentIndex(3)  # Default to 65536
        self.size_combo.currentIndexChanged.connect(self._generate_code)
        size_layout.addWidget(self.size_combo)

        size_note = QLabel("(65536 = true 16-bit, ~400KB file)")
        size_note.setStyleSheet("color: #888; font-size: 10px;")
        size_layout.addWidget(size_note)

        size_layout.addStretch()
        layout.addLayout(size_layout)

        # Code output
        self.code_text = QTextEdit()
        self.code_text.setStyleSheet("font-family: monospace; font-size: 11px;")
        layout.addWidget(self.code_text)

        # Buttons
        btn_layout = QHBoxLayout()

        copy_btn = QPushButton("Copy to Clipboard")
        copy_btn.clicked.connect(self._copy_code)
        btn_layout.addWidget(copy_btn)

        save_btn = QPushButton("Save .h File")
        save_btn.clicked.connect(self._save_code)
        btn_layout.addWidget(save_btn)

        btn_layout.addStretch()

        save_json_btn = QPushButton("Save Calibration (.json)")
        save_json_btn.clicked.connect(self._save_json)
        btn_layout.addWidget(save_json_btn)

        load_json_btn = QPushButton("Load Calibration")
        load_json_btn.clicked.connect(self._load_json)
        btn_layout.addWidget(load_json_btn)

        layout.addLayout(btn_layout)

        self._generate_code()

    def _get_size(self) -> int:
        text = self.size_combo.currentText()
        # Handle "4096 (recommended)" format
        return int(text.split()[0])

    def _generate_code(self):
        size = self._get_size()

        lines = [
            f"// Laser Calibration LUTs ({size} entries each)",
            f"// Generated by Direct Calibration Wizard",
            "",
        ]

        for name, cal in [('RED', self.red_cal), ('GREEN', self.green_cal), ('BLUE', self.blue_cal)]:
            lut = cal.generate_lut(size)

            lines.append(f"// {name}: Threshold={cal.threshold}, Points={len(cal.points)}")
            lines.append(f"const uint16_t {name}_LUT[{size}] PROGMEM = {{")

            row = []
            for i, val in enumerate(lut):
                row.append(f"{val:5d}")
                if len(row) == 8 or i == len(lut) - 1:
                    comma = "," if i < len(lut) - 1 else ""
                    lines.append("    " + ", ".join(row) + comma)
                    row = []

            lines.append("};")
            lines.append("")

        self.code_text.setText("\n".join(lines))

    def _copy_code(self):
        QApplication.clipboard().setText(self.code_text.toPlainText())

    def _save_code(self):
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Save LUT", "laser_lut.h", "Header Files (*.h)"
        )
        if filepath:
            with open(filepath, 'w') as f:
                f.write(self.code_text.toPlainText())

    def _save_json(self):
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Save Calibration", "calibration.json", "JSON (*.json)"
        )
        if filepath:
            data = {
                'red': self.red_cal.to_dict(),
                'green': self.green_cal.to_dict(),
                'blue': self.blue_cal.to_dict()
            }
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)

    def _load_json(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Load Calibration", "", "JSON (*.json)"
        )
        if filepath:
            with open(filepath, 'r') as f:
                data = json.load(f)
            if 'red' in data:
                self.red_cal.from_dict(data['red'])
            if 'green' in data:
                self.green_cal.from_dict(data['green'])
            if 'blue' in data:
                self.blue_cal.from_dict(data['blue'])
            self._generate_code()
            # Tell the main window to push the freshly-loaded calibration
            # state into each tab's UI -- otherwise the curve display
            # repaints from the new data (since it shares the same cal
            # reference) but the right-side Brightness Points list,
            # threshold spinbox, and Smooth/Linear toggle stay frozen on
            # the old state.
            self.calibration_loaded.emit()

    def refresh(self):
        self._generate_code()


# =============================================================================
# Main Window
# =============================================================================

class DirectCalibrationWizard(QMainWindow):
    """Main window for direct calibration wizard."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Direct Laser Calibration Wizard")
        self.setMinimumSize(900, 600)

        self.laser = LaserSerial()

        # LaserBath_Controller (USB pot box).  Constructed but NOT started
        # until the user opts in via the controller status panel.  The
        # device self-identifies on connect — see shared.serial_controller.
        # The signal is wired to a dispatcher that forwards values to
        # whichever channel tab is bound to that pot index.
        self.usb_controller: Optional[LaserBathController] = None
        if LASERBATH_AVAILABLE:
            # Read the firmware's NORM line, same as every other consumer
            # of the LaserBath_Controller (main LaserSystem app, etc.).
            # NORM has the firmware's smoothing + per-pot gamma + range
            # remap already applied -- the same response curve the main
            # app feels -- so the wizard now matches that behavior end-
            # to-end.  RAW is reserved for the controller's own
            # calibrator GUI, which needs the unprocessed signal to tune
            # the very curves NORM is built from.
            self.usb_controller = LaserBathController(parent=self)
            self.usb_controller.pot_changed.connect(self._dispatch_pot)
            self.usb_controller.connection_status.connect(self._on_controller_status)

        self._setup_ui()
        self._apply_style()

        # Try to find the pot box on launch.  This is a fire-and-forget;
        # if it fails the wizard still runs in mouse-only mode and the
        # user can plug the box in and click "Reconnect" later.
        if self.usb_controller is not None:
            self.usb_controller.start()

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # Connection bar
        conn_layout = QHBoxLayout()
        conn_layout.addWidget(QLabel("Serial Port:"))

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        conn_layout.addWidget(self.port_combo)

        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_ports)
        conn_layout.addWidget(refresh_btn)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connection)
        conn_layout.addWidget(self.connect_btn)

        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #f66; font-weight: bold;")
        conn_layout.addWidget(self.status_label)

        # Firmware LUT indicator + toggle — sends "LUT,0" / "LUT,1" to the
        # Teensy laser driver.  Two widgets instead of one button so the
        # CURRENT firmware state and the ACTION the button performs read
        # as two separate things:
        #   - "Firmware LUT: ON"         <- status, what the driver is doing
        #   - [ Disable LUT (calibrate raw) ]   <- button, what clicking does
        # The previous combined toggle made it ambiguous which state the
        # firmware was actually in, especially right after launch when the
        # UI default didn't match the firmware default.
        self.lut_state_label = QLabel("Firmware LUT: unknown")
        self.lut_state_label.setStyleSheet(
            "color: #aaa; font-weight: bold; padding: 0 6px;"
        )
        conn_layout.addWidget(self.lut_state_label)

        self.lut_btn = QPushButton("Disable LUT")
        self.lut_btn.setFixedWidth(180)
        self.lut_btn.setToolTip(
            "Firmware LUT (calibration curve) on/off.\n"
            "For calibration: LUT must be OFF — we measure the laser's\n"
            "RAW response and define a NEW curve to compensate for it.\n"
            "Calibrating with a LUT already applied stacks one LUT on top\n"
            "of another and ruins the result.\n"
            "Flip ON only to audit a finished LUT in place."
        )
        # _on_lut_button now toggles the firmware state (no toggle-button
        # checked state) — the label above is the source of truth.
        self.lut_btn.clicked.connect(self._on_lut_button)
        conn_layout.addWidget(self.lut_btn)

        # ---- LaserBath_Controller status ----
        # Tiny readout that confirms the USB pot box is talking.  Hidden
        # entirely if the SerialController module wasn't importable.
        if LASERBATH_AVAILABLE:
            self.controller_status_label = QLabel("Pot box: searching…")
            self.controller_status_label.setStyleSheet(
                "color: #aaa; font-weight: bold; padding: 0 6px;"
            )
            conn_layout.addWidget(self.controller_status_label)
            self.controller_reconnect_btn = QPushButton("Reconnect Pots")
            self.controller_reconnect_btn.clicked.connect(
                self._reconnect_controller
            )
            conn_layout.addWidget(self.controller_reconnect_btn)
        else:
            self.controller_status_label = None
            self.controller_reconnect_btn = None

        conn_layout.addStretch()
        layout.addLayout(conn_layout)

        # ---- LUT-active warning banner ----
        # Hidden until we confirm via LUT,? that the firmware is calibrated.
        # Calibrating ON TOP of an existing LUT defeats the purpose, so the
        # banner is loud, with a one-click "Disable" action right inside it.
        self.lut_warning_bar = QFrame()
        self.lut_warning_bar.setStyleSheet(
            "QFrame { background: #6a3a00; border: 1px solid #c87000; "
            "  border-radius: 4px; padding: 4px; }"
        )
        warn_layout = QHBoxLayout(self.lut_warning_bar)
        warn_layout.setContentsMargins(8, 4, 8, 4)
        warn_label = QLabel(
            "⚠  Laser driver currently has a LUT applied. "
            "Calibrating now will create a LUT on top of another LUT. "
            "Disable LUT before continuing."
        )
        warn_label.setStyleSheet("color: #ffd060; font-weight: bold;")
        warn_label.setWordWrap(True)
        warn_layout.addWidget(warn_label, stretch=1)
        disable_btn = QPushButton("Disable LUT")
        disable_btn.setStyleSheet(
            "QPushButton { background: #c87000; color: white; font-weight: bold; }"
            "QPushButton:hover { background: #e08000; }"
        )
        disable_btn.clicked.connect(self._disable_lut_from_banner)
        warn_layout.addWidget(disable_btn)
        self.lut_warning_bar.hide()
        layout.addWidget(self.lut_warning_bar)

        # Tabs for each channel
        self.tabs = QTabWidget()

        self.red_tab = ChannelTab("Red", QColor(255, 80, 80), self.laser)
        self.red_tab.calibration_changed.connect(self._on_calibration_changed)
        self.red_tab.compare_requested.connect(self._on_compare_requested)
        self.tabs.addTab(self.red_tab, "RED")

        self.green_tab = ChannelTab("Green", QColor(80, 255, 80), self.laser)
        self.green_tab.calibration_changed.connect(self._on_calibration_changed)
        self.green_tab.compare_requested.connect(self._on_compare_requested)
        self.tabs.addTab(self.green_tab, "GREEN")

        self.blue_tab = ChannelTab("Blue", QColor(80, 80, 255), self.laser)
        self.blue_tab.calibration_changed.connect(self._on_calibration_changed)
        self.blue_tab.compare_requested.connect(self._on_compare_requested)
        self.tabs.addTab(self.blue_tab, "BLUE")

        # Style the tabs with channel-specific colors.  Earlier this used
        # ::first / ::middle / ::last pseudoclass selectors, but Qt's
        # stylesheet engine can't address a single tab by index -- and
        # once we added EXPORT as the 4th tab, BLUE drifted into the
        # "middle" bucket (taking green) and EXPORT became "last" (taking
        # blue).  Per-tab setTabTextColor() is unambiguous and survives
        # any future tab additions.
        self.tabs.setStyleSheet(
            "QTabBar::tab:selected { background: #333; }"
        )

        self.export_tab = ExportTab(
            self.red_tab.calibration,
            self.green_tab.calibration,
            self.blue_tab.calibration
        )
        # When the Export tab loads a calibration JSON, refresh each
        # channel tab's UI so its right-side Brightness Points list,
        # threshold spinbox, and interpolation toggle match the freshly
        # loaded data.  The cal objects themselves are already updated
        # in-place (ExportTab and the tabs share the same instances).
        self.export_tab.calibration_loaded.connect(self._on_calibration_loaded)
        self.tabs.addTab(self.export_tab, "EXPORT")

        # Index-pinned tab colors.  Has to happen AFTER addTab for each
        # tab; setTabTextColor looks up the tab by its current index.
        self.tabs.tabBar().setTabTextColor(0, QColor(255, 136, 136))  # RED
        self.tabs.tabBar().setTabTextColor(1, QColor(136, 255, 136))  # GREEN
        self.tabs.tabBar().setTabTextColor(2, QColor(136, 136, 255))  # BLUE
        # EXPORT is left with the default light-grey colour so it visually
        # stands apart from the three colour-channel tabs.

        layout.addWidget(self.tabs)

        # Bottom pot-knob row — M / R / G / B.  These mirror the
        # LaserBath_Controller's physical pots so the user can see all four
        # positions at a glance, and use them to drive the manual test
        # slider exactly like the main LaserSystem app does.  See
        # _dispatch_pot for the active-tab routing rules.
        layout.addWidget(self._build_pot_knob_row())

        # Cross-tab slider carry-over.  Lets the user dial a specific raw
        # PWM count on (say) red, then click the green tab and immediately
        # see what THAT same count looks like on green — essential for
        # eye-matching brightness across diodes.  See _on_tab_changed.
        self._last_tab_index = self.tabs.currentIndex()
        self.tabs.currentChanged.connect(self._on_tab_changed)

        # The bottom knob row is now the single point of pot-routing
        # configuration, so hide the legacy per-tab "Use LaserBath pot:"
        # widgets.  They're still alive in case future refactors want them;
        # apply_pot_value() is still the public API for "pot says X".
        for _t in (self.red_tab, self.green_tab, self.blue_tab):
            _t.use_controller_cb.hide()
            _t.controller_pot_combo.hide()

        self._refresh_ports()

    def _build_pot_knob_row(self) -> QWidget:
        """Build the bottom-right knob row: M / R / G / B mirroring the
        LaserBath pot box.  Stretch on the left so the row hugs the bottom
        right of the window, matching the user's spec.
        """
        container = QWidget()
        row = QHBoxLayout(container)
        row.setContentsMargins(6, 4, 12, 4)
        row.setSpacing(10)
        row.addStretch()

        title = QLabel("POT BOX")
        title.setStyleSheet(
            "color: #888; font-size: 10px; font-weight: bold;"
            " border: none; padding-right: 6px;"
        )
        title.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight)
        row.addWidget(title)

        # Default pot indices: M=5 (unused by R/G/B), R=0, G=1, B=4.
        # R/G/B defaults match the firmware POT_ENABLED mask the rest of the
        # wizard uses; M picks an arbitrary free wired pot so out-of-box the
        # user gets a working "any channel" input without having to fiddle
        # with dropdowns.
        self.pot_knob_master = _PotKnob("M", "#cccccc", default_pot=5)
        self.pot_knob_red    = _PotKnob("R", "#ff4444", default_pot=0)
        self.pot_knob_green  = _PotKnob("G", "#44ff44", default_pot=1)
        self.pot_knob_blue   = _PotKnob("B", "#4488ff", default_pot=4)

        for knob in (self.pot_knob_master, self.pot_knob_red,
                     self.pot_knob_green, self.pot_knob_blue):
            row.addWidget(knob)
            # Wire each knob's MAP button to the central learn-mode state
            # machine.  Using a default-arg lambda so each knob's button
            # carries a reference to ITS knob (avoids late-binding bugs).
            knob.learn_requested.connect(
                lambda _k=knob: self._toggle_pot_learn(_k)
            )

        # ── MIDI-learn state ─────────────────────────────────────────────
        # _learning_knob: which knob (if any) is currently in learn mode.
        # Only one knob can be learning at a time.
        # _last_pot_values: most recent value for every pot index that has
        # ever emitted.  Used as the BASELINE when learn mode starts so we
        # can detect which pot the user is wiggling (anything that moves by
        # more than LEARN_DELTA_THRESHOLD from its baseline gets bound).
        # _learn_baselines: snapshot of _last_pot_values at the moment learn
        # mode was entered.
        # _learn_timeout: QTimer that auto-cancels learn mode after a few
        # seconds of no significant pot movement, so the user can't leave
        # the wizard in a "listening forever" state.
        self._learning_knob: Optional[_PotKnob] = None
        self._last_pot_values: Dict[int, float] = {}
        self._learn_baselines: Dict[int, float] = {}
        self._learn_timeout = QTimer(self)
        self._learn_timeout.setSingleShot(True)
        self._learn_timeout.timeout.connect(self._cancel_pot_learn)

        return container

    # Threshold: how much a pot has to move (0.0..1.0 units) from its
    # baseline to count as "the one the user is wiggling".  3% is well
    # above ADS1115 jitter (~0.5%) but small enough that any deliberate
    # twist is unambiguous.
    LEARN_DELTA_THRESHOLD = 0.03
    LEARN_TIMEOUT_MS = 8000  # auto-cancel learn mode after 8 s of no input

    # Host-side snap-to-zero removed 2026-05-23 after the user upgraded the
    # pot hardware (the previous worst offender, pot 9, had a ~0.4%
    # mechanical rest-position offset that the firmware's gamma+snap on
    # NORM hid for the main app but the wizard's RAW path exposed).  If
    # the new pots show residuals again, re-introduce a threshold
    # constant here and reinstate the snap block in _dispatch_pot below.

    def _toggle_pot_learn(self, knob: "_PotKnob"):
        """MAP button clicked on `knob` -- start or cancel learn mode."""
        if self._learning_knob is knob:
            # Clicked the same knob's MAP again -- treat as cancel.
            self._cancel_pot_learn()
            return
        # Switching focus from one knob to another while a learn was
        # already in progress: clear the previous knob's visual state.
        if self._learning_knob is not None:
            self._learning_knob.set_learning(False)
        self._learning_knob = knob
        # Snapshot every pot we've seen so far so we can spot the one
        # that moves beyond the threshold.
        self._learn_baselines = dict(self._last_pot_values)
        knob.set_learning(True)
        self._learn_timeout.start(self.LEARN_TIMEOUT_MS)

    def _cancel_pot_learn(self):
        """Exit learn mode without binding."""
        if self._learning_knob is not None:
            self._learning_knob.set_learning(False)
        self._learning_knob = None
        self._learn_baselines = {}
        self._learn_timeout.stop()

    def _on_tab_changed(self, new_index: int):
        """Copy the manual test slider's value forward when the user jumps
        between color tabs, so a setting dialed on (say) red shows up on
        green right away.

        This makes the calibration loop "set 6000 PWM on red → tab to green
        → see how green looks at the same 6000" instead of "memorize 6000,
        tab over, retype it."
        """
        prev_idx = getattr(self, '_last_tab_index', None)
        self._last_tab_index = new_index
        if prev_idx is None or prev_idx == new_index:
            return
        prev_widget = self.tabs.widget(prev_idx)
        new_widget = self.tabs.widget(new_index)
        if (isinstance(prev_widget, ChannelTab)
                and isinstance(new_widget, ChannelTab)):
            prev_value = prev_widget.sweep_slider.value()
            if new_widget.sweep_slider.value() != prev_value:
                new_widget.sweep_slider.setValue(prev_value)

    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #1a1a1a; color: #ddd; }
            QGroupBox {
                color: #fff; font-weight: bold;
                border: 1px solid #444; border-radius: 5px;
                margin-top: 10px; padding-top: 10px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton {
                background-color: #333; color: #ddd;
                border: 1px solid #555; padding: 6px 12px; border-radius: 3px;
            }
            QPushButton:hover { background-color: #444; }
            QPushButton:pressed { background-color: #0af; color: black; }
            QComboBox, QSpinBox, QLineEdit {
                background-color: #333; color: #ddd;
                border: 1px solid #555; padding: 5px; border-radius: 3px;
            }
            QSlider::groove:horizontal { background: #333; height: 8px; border-radius: 4px; }
            QSlider::handle:horizontal { background: #0af; width: 16px; margin: -4px 0; border-radius: 8px; }
            QTextEdit { background-color: #252525; color: #ddd; border: 1px solid #444; }
            QTabWidget::pane { border: 1px solid #444; }
            QTabBar::tab {
                background-color: #2a2a2a; color: #888;
                padding: 10px 25px; border: 1px solid #444; border-bottom: none;
                font-weight: bold;
            }
            QTabBar::tab:selected { background-color: #1a1a1a; color: #fff; }
            QScrollArea { border: none; }
        """)

    def _refresh_ports(self):
        self.port_combo.clear()
        ports = self.laser.list_ports()
        if ports:
            self.port_combo.addItems(ports)
        else:
            self.port_combo.addItem("No ports found")

    def _toggle_connection(self):
        if self.laser.is_connected():
            self.laser.disconnect()
            self.connect_btn.setText("Connect")
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: #f66; font-weight: bold;")
            # Clear LUT state — we don't know the firmware's state anymore.
            self._set_lut_ui_state(None)
            self.lut_warning_bar.hide()
        else:
            port = self.port_combo.currentText()
            if port and "No ports" not in port:
                if self.laser.connect(port):
                    self.connect_btn.setText("Disconnect")
                    self.status_label.setText(f"Connected: {port}")
                    self.status_label.setStyleSheet("color: #6f6; font-weight: bold;")
                    # Query the firmware to find out what state the LUT is
                    # actually in.  Firmware boots LUT,ON by default — if
                    # we ASSUMED a default we'd be wrong about half the
                    # time. The query reply ('LUT,ON' / 'LUT,OFF') is the
                    # only source of truth.
                    state = self.laser.query_lut_state()
                    self._set_lut_ui_state(state)
                    # If the LUT is on, raise the warning banner so the
                    # user can't miss it before they start placing points.
                    if state is True:
                        self.lut_warning_bar.show()
                    else:
                        self.lut_warning_bar.hide()

    def _set_lut_ui_state(self, state: Optional[bool]):
        """Sync the LUT label + action-button text to a known firmware state.

        state == True  -> firmware LUT is ON  (calibrated)
        state == False -> firmware LUT is OFF (raw)
        state is None  -> unknown (disconnected, or no reply)
        """
        if state is True:
            self.lut_state_label.setText("Firmware LUT: ON (calibrated)")
            self.lut_state_label.setStyleSheet(
                "color: #ff8a3a; font-weight: bold; padding: 0 6px;"
            )
            self.lut_btn.setText("Disable LUT (calibrate raw)")
            self.lut_btn.setStyleSheet(
                "QPushButton { background: #5a3a00; color: #ffd060; "
                "  font-weight: bold; padding: 4px 8px; border: 1px solid #444; }"
                "QPushButton:hover { background: #6a4a00; }"
            )
        elif state is False:
            self.lut_state_label.setText("Firmware LUT: OFF (raw)")
            self.lut_state_label.setStyleSheet(
                "color: #6f6; font-weight: bold; padding: 0 6px;"
            )
            self.lut_btn.setText("Enable LUT (verify calibration)")
            self.lut_btn.setStyleSheet(
                "QPushButton { background: #2a4a2a; color: #cfc; "
                "  font-weight: bold; padding: 4px 8px; border: 1px solid #444; }"
                "QPushButton:hover { background: #3a5a3a; }"
            )
        else:
            self.lut_state_label.setText("Firmware LUT: unknown")
            self.lut_state_label.setStyleSheet(
                "color: #aaa; font-weight: bold; padding: 0 6px;"
            )
            self.lut_btn.setText("Disable LUT")
            self.lut_btn.setStyleSheet(
                "QPushButton { background: #333; color: #ddd; "
                "  font-weight: bold; padding: 4px 8px; border: 1px solid #444; }"
            )

    def _on_lut_button(self):
        """User clicked the LUT button — flip the firmware state and re-query.

        We don't trust the click alone: we send the toggle, then ask the
        firmware what state it's in. That way the displayed state always
        matches reality, even if the command is dropped or the firmware
        was already in the requested state.
        """
        # Determine current state from the label (cheaper than tracking
        # a separate variable, and the label IS our source of truth).
        currently_on = "ON" in self.lut_state_label.text()
        new_state = not currently_on
        self.laser.send_command("LUT,1" if new_state else "LUT,0")
        # Re-query to confirm — firmware echoes immediately so a brief
        # blocking read is fine.
        confirmed = self.laser.query_lut_state()
        self._set_lut_ui_state(confirmed)
        if confirmed is True:
            self.lut_warning_bar.show()
        else:
            self.lut_warning_bar.hide()
        print(
            f"[LUT] requested -> {'ON' if new_state else 'OFF'}, "
            f"firmware reports -> {confirmed}"
        )

    def _disable_lut_from_banner(self):
        """One-click 'Disable LUT' shortcut inside the warning banner."""
        self.laser.send_command("LUT,0")
        confirmed = self.laser.query_lut_state()
        self._set_lut_ui_state(confirmed)
        if confirmed is False:
            self.lut_warning_bar.hide()

    # ---- LaserBath_Controller (USB pot box) routing ---------------------

    def _dispatch_pot(self, pot_index: int, value: float):
        """A LaserBath pot reported a new value — route to knob visuals
        and to the appropriate channel tab's slider.

        Routing model (updated 2026-05-23):
          1. EVERY M/R/G/B knob whose pot index matches updates its visual.
             So all four bottom knobs always reflect their physical pots
             at a glance, even when the matching color tab isn't visible.
          2. If a knob is in MIDI-learn mode, check whether THIS event came
             from the pot the user is wiggling and, if so, bind it.  Done
             BEFORE slider routing so the binding gesture doesn't also
             dump its value onto the laser.
          3. Channel routing:
             - MASTER pot drives whichever ChannelTab is currently visible
               (so M is a "universal" control that follows the user).
             - R / G / B pots drive their fixed channel (red_tab, green_tab,
               blue_tab) regardless of which tab is on top.  This lets the
               user compare three lasers' brightness at the same percent
               by twisting all three at once, without flipping between
               tabs to apply each one.
        """
        # Always remember the latest value per pot; the MIDI-learn flow
        # uses this snapshot as the baseline against which "did this pot
        # move?" is computed.  Negative values (-1.0 = offline) are
        # excluded so a disconnected ADS board doesn't poison the baseline.
        if value >= 0.0:
            self._last_pot_values[pot_index] = value

        # 1) Visual mirror — update every knob whose pot index matches.
        knobs = (self.pot_knob_master, self.pot_knob_red,
                 self.pot_knob_green, self.pot_knob_blue)
        for knob in knobs:
            if knob.pot_index() == pot_index:
                knob.set_value_normalized(value)

        # 2) MIDI-learn intercept.  If a knob is listening, decide whether
        # this pot moved enough from its baseline to count as the user's
        # deliberate "this one" gesture.  When it has, rebind the knob and
        # exit learn mode.  Return early so we don't also drive the slider
        # on the binding gesture (the user almost certainly doesn't want
        # the laser to flash while they're picking a pot).
        if self._learning_knob is not None and value >= 0.0:
            baseline = self._learn_baselines.get(pot_index)
            # If we never saw this pot before the learn started, treat the
            # first emission as the baseline rather than instantly binding
            # (otherwise unrelated jitter from a pot the user isn't even
            # touching could win).
            if baseline is None:
                self._learn_baselines[pot_index] = value
            elif abs(value - baseline) >= self.LEARN_DELTA_THRESHOLD:
                self._learning_knob.set_pot_index(pot_index)
                self._cancel_pot_learn()
                return

        # 3a) MASTER -> active tab (whatever tab the user is looking at).
        # The Export tab has no slider, so skip Master routing in that case.
        if self.pot_knob_master.pot_index() == pot_index:
            active_tab = self.tabs.currentWidget()
            if isinstance(active_tab, ChannelTab):
                active_tab.apply_pot_value(value)
            return

        # 3b) R / G / B -> fixed channel tabs, regardless of which is active.
        # Each ChannelTab owns its own LaserSerial channel name + slider,
        # so apply_pot_value() drives the correct laser even when the user
        # is on a different tab.  No visual feedback on the off-tab slider
        # (it's hidden), but the laser still updates -- which is the
        # whole point of letting the user scrub three colors in parallel.
        fixed_routes = (
            (self.pot_knob_red,   self.red_tab),
            (self.pot_knob_green, self.green_tab),
            (self.pot_knob_blue,  self.blue_tab),
        )
        for knob, tab in fixed_routes:
            if knob.pot_index() == pot_index:
                tab.apply_pot_value(value)
                return

    def _on_controller_status(self, connected: bool, port: str):
        """LaserBath controller connection state changed."""
        if self.controller_status_label is None:
            return
        if connected:
            self.controller_status_label.setText(f"Pot box: connected ({port})")
            self.controller_status_label.setStyleSheet(
                "color: #6f6; font-weight: bold; padding: 0 6px;"
            )
        else:
            self.controller_status_label.setText("Pot box: not found")
            self.controller_status_label.setStyleSheet(
                "color: #aaa; font-weight: bold; padding: 0 6px;"
            )

    def _reconnect_controller(self):
        """Manual re-scan for the USB pot box (after a hot-plug)."""
        if self.usb_controller is None:
            return
        self.usb_controller.stop()
        self.usb_controller.start()

    def _on_calibration_changed(self):
        # Debounced: rapid edits (typing, holding an arrow, clicking ±1
        # repeatedly) coalesce into ONE LUT regeneration ~150 ms after
        # activity stops.  Before this, every keystroke / arrow click
        # ran _generate_code() -> 3× 65,536-entry LUT interpolations +
        # a giant QTextEdit rebuild, which produced the visible input
        # lag the user reported.  The timer is created lazily on first
        # use so we don't have to touch __init__.
        timer = getattr(self, "_export_refresh_timer", None)
        if timer is None:
            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.timeout.connect(self.export_tab.refresh)
            self._export_refresh_timer = timer
        timer.start(150)

    def _on_calibration_loaded(self):
        """ExportTab finished loading a JSON.  Each tab's underlying
        ChannelCalibration is already updated (shared reference), but the
        per-tab UI widgets that were populated ONCE at construction need
        to be pushed back to the fresh state:

          * Brightness Points list -- rebuild from points dict
          * Threshold spinbox     -- set to loaded threshold
          * Smooth / Linear pair  -- match loaded use_smooth flag
          * Curve display          -- explicit repaint (it shares the cal
            object but doesn't see set_point-style writes from from_dict)

        Signals are blocked while we push values back so that re-loading
        doesn't trigger calibration_changed cascades (we just changed
        everything; one debounced export refresh is enough).
        """
        for tab in (self.red_tab, self.green_tab, self.blue_tab):
            cal = tab.calibration
            # Threshold spinbox.
            tab.threshold_input.blockSignals(True)
            tab.threshold_input.setValue(int(cal.threshold))
            tab.threshold_input.blockSignals(False)
            # Smooth/Linear button pair.
            tab.smooth_btn.blockSignals(True)
            tab.linear_btn.blockSignals(True)
            tab.smooth_btn.setChecked(bool(cal.use_smooth))
            tab.linear_btn.setChecked(not bool(cal.use_smooth))
            tab.smooth_btn.blockSignals(False)
            tab.linear_btn.blockSignals(False)
            # Brightness Points list.
            tab._rebuild_points_ui()
            # Force a curve repaint.  The display shares the cal object
            # so a paintEvent would eventually catch up, but an explicit
            # update() makes the refresh feel instant.
            tab.curve_display.update()

    def _on_compare_requested(self, target_channel: str, percent: float):
        """A ChannelTab's "Compare ▶" button was pressed.  Fire the SAME
        percent the user is currently viewing on the target channel
        (looking up its calibration curve), and zero the other two
        channels so the target color is the only thing lit.

        Time-multiplexed comparison: each click in the source tab steps
        to the next enabled toggle, so the user sees one color at a
        time at a fixed percent and can eye-match brightness without
        flipping between tabs.

        Wire-protocol note: LaserSerial.send_channel(ch, v) sends a FULL
        RGB packet with the other two channels at 0.  Calling it three
        times in a row (once per channel) would zero each previous
        write -- last call wins, all channels end up at zero, laser
        goes dark.  We build the RGB triple in code and emit a SINGLE
        send_rgb() instead so the wire sees the comparison state in
        one atomic 9-byte packet.
        """
        target_tab = {
            "red":   self.red_tab,
            "green": self.green_tab,
            "blue":  self.blue_tab,
        }.get(target_channel)
        if target_tab is None:
            return
        clamped = max(0.0, min(100.0, percent))
        pwm = int(target_tab.calibration.interpolate(clamped))
        r = pwm if target_channel == "red"   else 0
        g = pwm if target_channel == "green" else 0
        b = pwm if target_channel == "blue"  else 0
        self.laser.send_rgb(r, g, b)

    def closeEvent(self, event):
        self.red_tab.stop()
        self.green_tab.stop()
        self.blue_tab.stop()
        # Stop the LaserBath pot reader thread before the Qt event loop
        # exits — otherwise the background thread can outlive the window
        # and try to emit signals into a deleted object.
        if self.usb_controller is not None:
            self.usb_controller.stop()
        self.laser.disconnect()
        event.accept()


# =============================================================================
# Entry Point
# =============================================================================

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = DirectCalibrationWizard()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
