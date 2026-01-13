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
    QLabel, QPushButton, QComboBox, QSlider, QSpinBox, QGroupBox,
    QTabWidget, QFrame, QScrollArea, QLineEdit, QSizePolicy,
    QFileDialog, QTextEdit, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QPainter, QColor, QPen, QFont

import serial
import serial.tools.list_ports


# =============================================================================
# Constants
# =============================================================================

PWM_MAX = 65535  # 16-bit PWM resolution (native 16-bit on ESP32)
PWM_BITS = 16
LUT_SIZE = 65536  # Full 16-bit indexing - no precision loss


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
        data = struct.pack('<HHH', r, g, b)  # 16-bit RGB values (little-endian)
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

        # PWM input
        self.pwm_input = QSpinBox()
        self.pwm_input.setRange(0, PWM_MAX)
        self.pwm_input.setValue(pwm_value)
        self.pwm_input.setMinimumWidth(80)
        self.pwm_input.valueChanged.connect(self._on_value_changed)
        layout.addWidget(self.pwm_input)

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

class AddPointButton(QPushButton):
    """Button to add a new calibration point between two existing ones."""

    add_requested = pyqtSignal(int, int)  # (lower_percent, upper_percent)

    def __init__(self, lower_percent: int, upper_percent: int, parent=None):
        super().__init__(parent)
        self.lower = lower_percent
        self.upper = upper_percent

        mid = (lower_percent + upper_percent) // 2
        self.setText(f"+ Add {mid}%")
        self.setStyleSheet("""
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
        self.clicked.connect(lambda: self.add_requested.emit(self.lower, self.upper))


# =============================================================================
# Channel Calibration Tab
# =============================================================================

class ChannelTab(QWidget):
    """Tab for calibrating a single channel."""

    calibration_changed = pyqtSignal()

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

        # Manual sweep slider
        manual_label = QLabel("Manual Test:")
        manual_label.setStyleSheet("color: #888; font-size: 11px;")
        sweep_layout.addWidget(manual_label)

        sweep_ctrl = QHBoxLayout()
        self.sweep_slider = QSlider(Qt.Orientation.Horizontal)
        self.sweep_slider.setRange(0, 100)
        self.sweep_slider.setValue(0)
        self.sweep_slider.valueChanged.connect(self._on_sweep_change)
        sweep_ctrl.addWidget(self.sweep_slider)

        self.sweep_label = QLabel("0% = 0")
        self.sweep_label.setMinimumWidth(100)
        sweep_ctrl.addWidget(self.sweep_label)

        sweep_layout.addLayout(sweep_ctrl)

        # Jump test button
        self.jump_btn = QPushButton("Jump Between Values (Space)")
        self.jump_btn.clicked.connect(self._jump_to_next)
        sweep_layout.addWidget(self.jump_btn)

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

            # Add "add point" button between this and next
            if i < len(sorted_percents) - 1:
                next_percent = sorted_percents[i + 1]
                if next_percent - percent > 1:  # Only if there's room
                    add_btn = AddPointButton(percent, next_percent)
                    add_btn.add_requested.connect(self._on_add_point)
                    self.points_layout.addWidget(add_btn)

        self.points_layout.addStretch()

    def _on_point_value_changed(self, percent: int, value: int):
        self.calibration.set_point(percent, value)
        self.curve_display.update()
        self.calibration_changed.emit()

    def _on_test_point(self, percent: int):
        value = self.calibration.interpolate(percent)
        self.laser.send_channel(self.channel_name, value)
        self.sweep_slider.blockSignals(True)
        self.sweep_slider.setValue(percent)
        self.sweep_slider.blockSignals(False)
        self.sweep_label.setText(f"{percent}% = {value}")

        # Highlight current
        for p, widget in self.point_widgets.items():
            widget.highlight(p == percent)

    def _on_remove_point(self, percent: int):
        self.calibration.remove_point(percent)
        self._rebuild_points_ui()
        self.curve_display.update()
        self.calibration_changed.emit()

    def _on_add_point(self, lower: int, upper: int):
        new_percent = self.calibration.add_point_between(lower, upper)
        if new_percent:
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

    def _on_sweep_change(self, percent: int):
        value = self.calibration.interpolate(percent)
        self.laser.send_channel(self.channel_name, value)
        self.sweep_label.setText(f"{percent}% = {value}")
        self.curve_display.set_sweep_position(percent)

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

        self._setup_ui()
        self._apply_style()

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

        conn_layout.addStretch()
        layout.addLayout(conn_layout)

        # Tabs for each channel
        self.tabs = QTabWidget()

        self.red_tab = ChannelTab("Red", QColor(255, 80, 80), self.laser)
        self.red_tab.calibration_changed.connect(self._on_calibration_changed)
        self.tabs.addTab(self.red_tab, "RED")

        self.green_tab = ChannelTab("Green", QColor(80, 255, 80), self.laser)
        self.green_tab.calibration_changed.connect(self._on_calibration_changed)
        self.tabs.addTab(self.green_tab, "GREEN")

        self.blue_tab = ChannelTab("Blue", QColor(80, 80, 255), self.laser)
        self.blue_tab.calibration_changed.connect(self._on_calibration_changed)
        self.tabs.addTab(self.blue_tab, "BLUE")

        # Style the tabs with colors
        self.tabs.setStyleSheet("""
            QTabBar::tab:selected { background: #333; }
            QTabBar::tab:first { color: #f88; }
            QTabBar::tab:middle { color: #8f8; }
            QTabBar::tab:last { color: #88f; }
        """)

        self.export_tab = ExportTab(
            self.red_tab.calibration,
            self.green_tab.calibration,
            self.blue_tab.calibration
        )
        self.tabs.addTab(self.export_tab, "EXPORT")

        layout.addWidget(self.tabs)

        self._refresh_ports()

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
        else:
            port = self.port_combo.currentText()
            if port and "No ports" not in port:
                if self.laser.connect(port):
                    self.connect_btn.setText("Disconnect")
                    self.status_label.setText(f"Connected: {port}")
                    self.status_label.setStyleSheet("color: #6f6; font-weight: bold;")

    def _on_calibration_changed(self):
        self.export_tab.refresh()

    def closeEvent(self, event):
        self.red_tab.stop()
        self.green_tab.stop()
        self.blue_tab.stop()
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
