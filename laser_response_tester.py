#!/usr/bin/env python3
"""
Laser Response Tester — standalone

Minimal tool for poking the ESP32 laser driver directly with 16-bit RGB
values. Each channel has a slider + spinbox (single-unit precision via
the keyboard arrows in the spinbox). LUT toggle wired to the firmware's
LUT,0 / LUT,1 commands.

Goal: characterize the actual PWM-to-light response of the laser without
any of the LaserSystem application layers in the way.
"""

import math
import sys
import struct
import time
from typing import Optional, List

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QButtonGroup, QComboBox, QSlider, QSpinBox,
    QDoubleSpinBox, QFrame, QCheckBox,
)

import serial
import serial.tools.list_ports


# ESP32 firmware expects: 6-byte binary RGB packets, 250kbaud, 16-bit per ch.
# Firmware (after 2026-05-16 bump) writes the value directly to a 16-bit
# PWM (no shift) at 1 kHz. Keep HW_PWM_BITS in sync with PWM_RESOLUTION
# in esp32_laser_driver.ino.
BAUD = 250000
PWM_MAX = 65535
PWM_BITS = 16
HW_PWM_BITS = 16          # what the firmware actually outputs (was 12 pre-bump)
HW_PWM_MAX = (1 << HW_PWM_BITS) - 1  # 65535


# ────────────────────────────────────────────────────────────────────────────
# Serial layer
# ────────────────────────────────────────────────────────────────────────────

class LaserSerial:
    """Direct ESP32 laser-driver link. 6-byte binary RGB + ASCII commands."""

    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.port: Optional[str] = None

    @staticmethod
    def list_ports() -> List[str]:
        out = []
        for p in serial.tools.list_ports.comports():
            d = p.device.lower()
            if "usbmodem" in d or "usbserial" in d:
                out.append(p.device)
        return sorted(out)

    def connect(self, port: str) -> bool:
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.1)
            self.port = port
            time.sleep(0.4)  # let ESP32 reset / open its CDC endpoint
            print(f"[CONN] opened {port} @ {BAUD} baud", flush=True)
            return True
        except Exception as e:
            print(f"[connect] {e}", file=sys.stderr)
            self.ser = None
            self.port = None
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            try:
                self.send_rgb(0, 0, 0)
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
        if self.port:
            print(f"[CONN] closed {self.port}", flush=True)
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
        # Per-channel tagged protocol — each channel value is prefixed
        # with its own tag byte so the firmware verifies alignment at every
        # channel boundary (not just once per packet).  9 bytes on the wire.
        #     0xFA  R_lo R_hi  0xFB  G_lo G_hi  0xFC  B_lo B_hi
        payload = struct.pack('<BHBHBH',
                              0xFA, r,
                              0xFB, g,
                              0xFC, b)
        try:
            self.ser.write(payload)
        except Exception as e:
            print(f"[send_rgb] {e}", file=sys.stderr)
            return
        # Terminal log: prove we're putting 9 bytes (tagged R/G/B)
        # on the wire.
        shift = PWM_BITS - HW_PWM_BITS  # 0 when firmware is at 16-bit
        hex_bytes = " ".join(f"{b:02X}" for b in payload)
        print(
            f"[TX RGB] R={r:5d} G={g:5d} B={b:5d}  "
            f"| {len(payload)} bytes (tagged R/G/B): {hex_bytes}  "
            f"| ESP32 will ledcWrite R={r >> shift:5d} G={g >> shift:5d} B={b >> shift:5d}",
            flush=True,
        )

    def send_command(self, cmd: str):
        """ASCII command like LUT,0 / LUT,1 / DEBUG,1."""
        if not self.is_connected():
            return
        line = cmd.rstrip() + "\n"
        payload = line.encode("ascii")
        try:
            self.ser.write(payload)
        except Exception as e:
            print(f"[send_command] {e}", file=sys.stderr)
            return
        hex_bytes = " ".join(f"{b:02X}" for b in payload)
        print(
            f"[TX CMD] {cmd!r}  | {len(payload)} bytes: {hex_bytes}",
            flush=True,
        )


# ────────────────────────────────────────────────────────────────────────────
# Fine-precision slider
# ────────────────────────────────────────────────────────────────────────────

class FineSlider(QSlider):
    """QSlider with DAW-style precision drag + mouse-wheel single-step.

    Why: a 0..65535 slider on a 1000-px window can only resolve ~65 units per
    pixel of drag. With modifier keys you can dial in single-unit precision
    without leaving the slider.

    Drag:
      plain drag       — default QSlider (1 px ≈ range/width units)
      Shift + drag     — 1/8 sensitivity   (~8 units / px on a 1000 px slider)
      Cmd / Ctrl drag  — 1/32 sensitivity  (~2 units / px — effectively
                                            per-unit-precision)
    Wheel:
      wheel            — ±1 unit / notch
      Shift + wheel    — ±16 units / notch
      Cmd / Ctrl wheel — ±256 units / notch
    """

    _DIVISOR_SHIFT = 8
    _DIVISOR_CTRL = 32

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._fine_divisor = None
        self._drag_start_x = 0
        self._drag_start_value = 0

    @staticmethod
    def _ctrl_like(mods):
        # Cmd on macOS = MetaModifier; treat the same as Ctrl elsewhere.
        return bool(mods & (Qt.KeyboardModifier.ControlModifier
                            | Qt.KeyboardModifier.MetaModifier))

    def mousePressEvent(self, event):
        mods = event.modifiers()
        if self._ctrl_like(mods):
            self._fine_divisor = self._DIVISOR_CTRL
        elif mods & Qt.KeyboardModifier.ShiftModifier:
            self._fine_divisor = self._DIVISOR_SHIFT
        else:
            self._fine_divisor = None

        if self._fine_divisor is not None:
            # Relative-drag mode: capture press point + current value, and
            # suppress QSlider's default jump-to-click so the handle stays put.
            self._drag_start_x = event.position().x()
            self._drag_start_value = self.value()
            event.accept()
            return

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if (self._fine_divisor is not None
                and event.buttons() & Qt.MouseButton.LeftButton):
            dx = event.position().x() - self._drag_start_x
            delta = int(dx / self._fine_divisor)
            new_value = self._drag_start_value + delta
            new_value = max(self.minimum(), min(self.maximum(), new_value))
            self.setValue(new_value)
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        self._fine_divisor = None
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event):
        mods = event.modifiers()
        if self._ctrl_like(mods):
            step = 256
        elif mods & Qt.KeyboardModifier.ShiftModifier:
            step = 16
        else:
            step = 1
        sign = 1 if event.angleDelta().y() > 0 else (
            -1 if event.angleDelta().y() < 0 else 0)
        if sign:
            new_value = max(self.minimum(),
                            min(self.maximum(), self.value() + sign * step))
            self.setValue(new_value)
        event.accept()


# ────────────────────────────────────────────────────────────────────────────
# Per-channel control row
# ────────────────────────────────────────────────────────────────────────────

class ChannelRow(QWidget):
    """Slider + spinbox + step buttons + bounce range for one channel."""

    def __init__(self, name: str, color: str, on_change, on_bounce_toggle):
        super().__init__()
        self._on_change = on_change
        self._on_bounce_toggle = on_bounce_toggle

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        label = QLabel(name)
        label.setFixedWidth(20)
        label.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 16px;")
        layout.addWidget(label)

        self.slider = FineSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, PWM_MAX)
        self.slider.setValue(0)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(256)
        self.slider.setToolTip(
            "Drag for coarse change.\n"
            "Shift+drag — 8× finer\n"
            "Cmd/Ctrl+drag — 32× finer (per-unit precision)\n"
            "Wheel: ±1   Shift+wheel: ±16   Cmd/Ctrl+wheel: ±256"
        )
        self.slider.setStyleSheet(
            "QSlider::groove:horizontal { background: #222; height: 6px; }"
            f"QSlider::handle:horizontal {{ background: {color}; width: 12px;"
            "  margin: -4px 0; border-radius: 2px; }"
        )
        layout.addWidget(self.slider, 1)

        # Single-step buttons for sub-pixel precision (slider can't hit every
        # 16-bit value with 1px granularity).
        for delta in (-1, +1):
            b = QPushButton("−" if delta < 0 else "+")
            b.setFixedSize(24, 22)
            b.clicked.connect(lambda _, d=delta: self.bump(d))
            layout.addWidget(b)

        self.spin = QSpinBox()
        self.spin.setRange(0, PWM_MAX)
        self.spin.setSingleStep(1)
        self.spin.setFixedWidth(80)
        self.spin.setKeyboardTracking(True)
        layout.addWidget(self.spin)

        self.pwm_label = QLabel("→ ledcWrite 0/4095")
        self.pwm_label.setFixedWidth(150)
        self.pwm_label.setStyleSheet("color: #888; font-size: 11px;")
        self.pwm_label.setToolTip(
            "What the ESP32 will write to ledcWrite() after applying LUT "
            "and any output shift. USB always carries the full 16-bit value."
        )
        layout.addWidget(self.pwm_label)

        # Vertical separator before the bounce-range controls.
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet("background: #333;")
        layout.addWidget(sep)

        # Min / Max bounds for the bounce sweep on this channel.
        min_lbl = QLabel("Min")
        min_lbl.setStyleSheet("color: #888; font-size: 10px;")
        layout.addWidget(min_lbl)
        self.min_spin = QSpinBox()
        self.min_spin.setRange(0, PWM_MAX)
        self.min_spin.setValue(0)
        self.min_spin.setFixedWidth(70)
        layout.addWidget(self.min_spin)

        max_lbl = QLabel("Max")
        max_lbl.setStyleSheet("color: #888; font-size: 10px;")
        layout.addWidget(max_lbl)
        self.max_spin = QSpinBox()
        self.max_spin.setRange(0, PWM_MAX)
        self.max_spin.setValue(PWM_MAX)
        self.max_spin.setFixedWidth(70)
        layout.addWidget(self.max_spin)

        self.bounce_check = QCheckBox("↕ Bounce")
        self.bounce_check.setToolTip(
            "Sweep this channel between Min and Max at the global period."
        )
        self.bounce_check.setStyleSheet(f"color: {color}; font-weight: bold;")
        self.bounce_check.toggled.connect(lambda _on: self._on_bounce_toggle())
        layout.addWidget(self.bounce_check)

        # Sync slider ↔ spinbox, fire callback once
        self._guard = False
        self.slider.valueChanged.connect(self._from_slider)
        self.spin.valueChanged.connect(self._from_spin)

    def value(self) -> int:
        return self.slider.value()

    def set_value(self, v: int, *, silent: bool = False):
        """Set slider+spinbox+pwm-label together.

        silent=True skips the per-row on_change callback — used by the
        bounce timer so we don't fire one TX per channel per tick.
        """
        v = max(0, min(PWM_MAX, int(v)))
        self._guard = True
        self.slider.setValue(v)
        self.spin.setValue(v)
        self._guard = False
        self._update_pwm_label(v)
        if not silent:
            self._on_change()

    def bump(self, delta: int):
        self.set_value(self.value() + delta)

    # ── Bounce helpers ────────────────────────────────────────────────────
    def is_bouncing(self) -> bool:
        return self.bounce_check.isChecked()

    def bounds(self) -> tuple:
        """Return (min, max) for the bounce sweep on this channel,
        normalised so min ≤ max."""
        lo = self.min_spin.value()
        hi = self.max_spin.value()
        if lo > hi:
            lo, hi = hi, lo
        return lo, hi

    def _from_slider(self, v: int):
        if self._guard:
            return
        self._guard = True
        self.spin.setValue(v)
        self._guard = False
        self._update_pwm_label(v)
        self._on_change()

    def _from_spin(self, v: int):
        if self._guard:
            return
        self._guard = True
        self.slider.setValue(v)
        self._guard = False
        self._update_pwm_label(v)
        self._on_change()

    def _update_pwm_label(self, v: int):
        # Mirrors the firmware's final shift to ledcWrite. With the 16-bit
        # firmware bump (2026-05-16) shift is 0 — slider value = PWM value.
        shift = PWM_BITS - HW_PWM_BITS
        self.pwm_label.setText(f"→ ledcWrite {v >> shift}/{HW_PWM_MAX}")


# ────────────────────────────────────────────────────────────────────────────
# Main window
# ────────────────────────────────────────────────────────────────────────────

class TesterWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Laser Response Tester")
        # Wider window = finer default slider sensitivity.
        self.resize(1280, 320)

        self.serial = LaserSerial()

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        # ── Port row ───────────────────────────────────────────────────────
        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(220)
        port_row.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        port_row.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        port_row.addWidget(self.connect_btn)

        self.status_dot = QLabel("●")
        self.status_dot.setStyleSheet("color: #f44; font-size: 18px;")
        self.status_dot.setToolTip("Disconnected")
        port_row.addWidget(self.status_dot)

        port_row.addStretch(1)
        root.addLayout(port_row)

        # ── Quick actions / LUT ────────────────────────────────────────────
        action_row = QHBoxLayout()

        self.lut_check = QCheckBox("LUT on  (firmware calibration table)")
        self.lut_check.setChecked(False)  # default OFF for raw response testing
        self.lut_check.toggled.connect(self._on_lut_toggled)
        action_row.addWidget(self.lut_check)

        action_row.addStretch(1)

        for label, val in (("All 0", 0), ("All Max", PWM_MAX)):
            b = QPushButton(label)
            b.clicked.connect(lambda _, v=val: self._set_all(v))
            action_row.addWidget(b)

        self.kill_btn = QPushButton("KILL (0,0,0)")
        self.kill_btn.setStyleSheet("background: #511; color: #fff; font-weight: bold;")
        self.kill_btn.clicked.connect(lambda: self._set_all(0))
        action_row.addWidget(self.kill_btn)

        root.addLayout(action_row)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("background: #333;")
        root.addWidget(sep)

        # Data-path banner — kept in sync with the firmware config.
        path = QLabel(
            f"Slider <b>{PWM_BITS}-bit</b> (0–{PWM_MAX})  →  USB <b>16-bit</b> "
            "(struct '&lt;HHH', 6 bytes)  →  ESP32 LUT 16→16 bit  →  "
            f"<span style='color:#8f0;'>PWM <b>{HW_PWM_BITS}-bit</b> "
            f"(0–{HW_PWM_MAX}) @ 1 kHz</span>"
        )
        path.setStyleSheet("color: #aaa; font-size: 11px; padding: 2px;")
        path.setTextFormat(Qt.TextFormat.RichText)
        root.addWidget(path)

        # ── Channel rows ───────────────────────────────────────────────────
        self.r_row = ChannelRow("R", "#ff5050", self._send_rgb, self._on_bounce_toggle)
        self.g_row = ChannelRow("G", "#50ff70", self._send_rgb, self._on_bounce_toggle)
        self.b_row = ChannelRow("B", "#5080ff", self._send_rgb, self._on_bounce_toggle)
        root.addWidget(self.r_row)
        root.addWidget(self.g_row)
        root.addWidget(self.b_row)

        # ── Bounce controls (global) ──────────────────────────────────────
        bounce_row = QHBoxLayout()
        bounce_row.addWidget(QLabel("Sweep period:"))
        self.period_spin = QDoubleSpinBox()
        self.period_spin.setRange(0.05, 60.0)
        self.period_spin.setSingleStep(0.1)
        self.period_spin.setDecimals(2)
        self.period_spin.setSuffix(" s")
        self.period_spin.setValue(2.0)
        self.period_spin.setFixedWidth(95)
        self.period_spin.setToolTip(
            "Time for one full Min → Max → Min cycle. Smaller = faster sweep."
        )
        bounce_row.addWidget(self.period_spin)

        bounce_row.addSpacing(12)
        bounce_row.addWidget(QLabel("Waveform:"))
        self.wave_combo = QComboBox()
        self.wave_combo.addItems(["Triangle", "Sine", "Saw up"])
        self.wave_combo.setToolTip(
            "Triangle — linear up then linear down (default).\n"
            "Sine     — smoother turnarounds at min/max.\n"
            "Saw up   — ramp up then snap back to min (asymmetric test)."
        )
        bounce_row.addWidget(self.wave_combo)

        # ── Sweep update rate (Hz) ──────────────────────────────────────────
        # 1 kHz matches the ESP32's USB-CDC frame interval and the 16-bit
        # PWM period — sending faster doesn't produce more distinct PWM
        # outputs. 500 Hz is the safe default (Qt PreciseTimer scheduling
        # is reliable down to ~2 ms on macOS).
        bounce_row.addSpacing(12)
        bounce_row.addWidget(QLabel("Rate:"))
        self._rate_btns = QButtonGroup(self)
        self._rate_btns.setExclusive(True)
        rate_btn_style = (
            "QPushButton { background: #2a2a2a; padding: 3px 8px; }"
            "QPushButton:checked { background: #3a6a3a; color: #cfc;"
            "                      font-weight: bold; }"
        )
        for hz in (100, 250, 500, 1000):
            btn = QPushButton(f"{hz} Hz")
            btn.setCheckable(True)
            btn.setStyleSheet(rate_btn_style)
            btn.setToolTip(
                f"{hz} Hz bounce update rate — interval {int(1000 / hz)} ms.\n"
                f"1 kHz matches the ESP32 PWM period; higher than that "
                f"sends duplicates."
            )
            btn.clicked.connect(lambda _checked, h=hz: self._set_bounce_rate(h))
            self._rate_btns.addButton(btn, hz)
            bounce_row.addWidget(btn)

        bounce_row.addSpacing(12)
        self.bounce_status = QLabel("(no channels bouncing)")
        self.bounce_status.setStyleSheet("color: #888; font-size: 11px;")
        bounce_row.addWidget(self.bounce_status)

        bounce_row.addStretch(1)
        root.addLayout(bounce_row)

        # Bounce engine — runs at user-selected rate (default 500 Hz).
        # Started only when at least one channel is enabled, so idle CPU
        # is zero. PreciseTimer is essential on macOS; the default coarse
        # QTimer has 1–15 ms jitter, which would visibly stutter sweeps.
        self._bounce_timer = QTimer(self)
        self._bounce_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._bounce_timer.timeout.connect(self._bounce_tick)
        self._bounce_phase = 0.0  # 0..1
        self._bounce_last_t = 0.0
        # Default 500 Hz (2 ms). Select the button and apply the interval.
        self._rate_btns.button(500).setChecked(True)
        self._set_bounce_rate(500)

        # ── Precision-drag hint ────────────────────────────────────────────
        hint = QLabel(
            "Slider precision:  drag = coarse  |  "
            "<b>Shift</b>+drag = 8× finer  |  "
            "<b>Cmd/Ctrl</b>+drag = 32× finer (per-unit)  |  "
            "wheel = ±1  |  Shift+wheel = ±16  |  Cmd/Ctrl+wheel = ±256"
        )
        hint.setTextFormat(Qt.TextFormat.RichText)
        hint.setStyleSheet("color: #888; font-size: 10px; padding: 2px;")
        root.addWidget(hint)

        # ── TX readout ─────────────────────────────────────────────────────
        self.tx_label = QLabel(
            f"USB TX (16-bit): R=    0 G=    0 B=    0     "
            f"ESP32 ledcWrite ({HW_PWM_BITS}-bit): R=    0 G=    0 B=    0"
        )
        self.tx_label.setStyleSheet("color: #ddd; font-family: monospace; font-size: 11px;")
        root.addWidget(self.tx_label)

        self.refresh_ports()
        self._update_connection_ui()

    # ── Port handling ──────────────────────────────────────────────────────

    def refresh_ports(self):
        prev = self.port_combo.currentText()
        self.port_combo.clear()
        ports = LaserSerial.list_ports()
        if ports:
            self.port_combo.addItems(ports)
            idx = self.port_combo.findText(prev)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
        else:
            self.port_combo.addItem("(no usbmodem/usbserial ports)")

    def toggle_connection(self):
        if self.serial.is_connected():
            self.serial.disconnect()
        else:
            port = self.port_combo.currentText()
            if port.startswith("("):
                return
            self.serial.connect(port)
            # Push current LUT state on connect.
            self._on_lut_toggled(self.lut_check.isChecked())
            self._send_rgb()
        self._update_connection_ui()

    def _update_connection_ui(self):
        if self.serial.is_connected():
            self.connect_btn.setText("Disconnect")
            self.status_dot.setStyleSheet("color: #4f4; font-size: 18px;")
            self.status_dot.setToolTip(f"Connected to {self.serial.port}")
        else:
            self.connect_btn.setText("Connect")
            self.status_dot.setStyleSheet("color: #f44; font-size: 18px;")
            self.status_dot.setToolTip("Disconnected")

    # ── Output ─────────────────────────────────────────────────────────────

    def _send_rgb(self):
        r, g, b = self.r_row.value(), self.g_row.value(), self.b_row.value()
        self.serial.send_rgb(r, g, b)
        shift = PWM_BITS - HW_PWM_BITS
        self.tx_label.setText(
            f"USB TX (16-bit): R={r:5d} G={g:5d} B={b:5d}     "
            f"ESP32 ledcWrite ({HW_PWM_BITS}-bit): "
            f"R={r >> shift:5d} G={g >> shift:5d} B={b >> shift:5d}"
        )

    def _set_all(self, v: int):
        for row in (self.r_row, self.g_row, self.b_row):
            row.set_value(v)
        # set_value already calls _send_rgb via the per-row callback

    def _on_lut_toggled(self, checked: bool):
        cmd = "LUT,1" if checked else "LUT,0"
        self.serial.send_command(cmd)

    # ── Bounce sweep engine ────────────────────────────────────────────────

    def _set_bounce_rate(self, hz: int):
        """Apply a new bounce-update rate (in Hz) to the live timer.
        Reconfigure on the fly even if the timer is currently running."""
        interval_ms = max(1, int(round(1000.0 / hz)))
        was_active = self._bounce_timer.isActive()
        self._bounce_timer.setInterval(interval_ms)
        # Reset the phase-clock anchor so dt doesn't blow up on the next tick
        # (the wall clock has advanced but we'd have over-counted).
        self._bounce_last_t = time.monotonic()
        if was_active:
            # Some Qt versions require a restart for setInterval to take effect
            # while the timer is running. Cheap to do unconditionally.
            self._bounce_timer.start()

    def _channels(self):
        return (self.r_row, self.g_row, self.b_row)

    def _bouncing_rows(self):
        return [r for r in self._channels() if r.is_bouncing()]

    def _on_bounce_toggle(self):
        """Start/stop the bounce timer depending on enabled channels.
        Keeps the phase across enable toggles so flipping doesn't snap."""
        active = self._bouncing_rows()
        if active and not self._bounce_timer.isActive():
            self._bounce_last_t = time.monotonic()
            self._bounce_timer.start()
        elif not active and self._bounce_timer.isActive():
            self._bounce_timer.stop()

        # Update the status label: which channels are bouncing.
        labels = []
        for r, lbl in zip(self._channels(), ("R", "G", "B")):
            if r.is_bouncing():
                labels.append(lbl)
        if labels:
            self.bounce_status.setText(
                f"bouncing {''.join(labels)}  ·  {int(round(1000.0 / max(1, self._bounce_timer.interval())))} Hz update"
            )
            self.bounce_status.setStyleSheet("color: #6f6; font-size: 11px;")
        else:
            self.bounce_status.setText("(no channels bouncing)")
            self.bounce_status.setStyleSheet("color: #888; font-size: 11px;")

    def _bounce_tick(self):
        """Advance phase by elapsed wall-clock time and update all enabled
        channels. One combined RGB TX per tick (silent set_value)."""
        now = time.monotonic()
        dt = now - self._bounce_last_t
        self._bounce_last_t = now

        period = max(0.05, self.period_spin.value())
        self._bounce_phase = (self._bounce_phase + dt / period) % 1.0

        # Compute waveform value v in [0, 1].
        waveform = self.wave_combo.currentText()
        p = self._bounce_phase
        if waveform == "Sine":
            # 0.5 - 0.5*cos(2πp) → 0 at p=0, 1 at p=0.5, 0 at p=1
            v = 0.5 - 0.5 * math.cos(2.0 * math.pi * p)
        elif waveform == "Saw up":
            v = p
        else:  # Triangle
            v = 2.0 * p if p < 0.5 else 2.0 * (1.0 - p)

        any_changed = False
        for row in self._channels():
            if not row.is_bouncing():
                continue
            lo, hi = row.bounds()
            new_val = int(round(lo + v * (hi - lo)))
            if new_val != row.value():
                row.set_value(new_val, silent=True)
                any_changed = True

        if any_changed:
            # One combined RGB packet per tick, not three.
            self._send_rgb()

    def closeEvent(self, e):
        try:
            self._bounce_timer.stop()
        except Exception:
            pass
        try:
            self.serial.disconnect()
        finally:
            super().closeEvent(e)


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet("QWidget { background: #1a1a1a; color: #ddd; }"
                      "QPushButton { background: #2a2a2a; padding: 4px 10px; }"
                      "QPushButton:hover { background: #333; }"
                      "QSpinBox, QComboBox { background: #222; padding: 2px; }")
    w = TesterWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
