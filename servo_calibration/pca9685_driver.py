"""Minimal PCA9685 wrapper for raw servo pulse-width control.

Deliberately thin: the calibration tool needs to command an exact pulse
width in microseconds on a given channel, nothing more. Higher-level
angle-based control belongs elsewhere, built on top of ServoCalibration.
"""

from __future__ import annotations

import board
import busio
from adafruit_pca9685 import PCA9685

DEFAULT_FREQUENCY_HZ = 50
DEFAULT_I2C_ADDRESS = 0x40


class Pca9685Driver:
    def __init__(
        self,
        frequency_hz: int = DEFAULT_FREQUENCY_HZ,
        address: int = DEFAULT_I2C_ADDRESS,
    ) -> None:
        i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(i2c, address=address)
        self._pca.frequency = frequency_hz
        self.frequency_hz = frequency_hz

    def set_pulse_us(self, channel: int, pulse_us: float) -> None:
        period_us = 1_000_000 / self.frequency_hz
        duty_cycle = round(pulse_us / period_us * 65535)
        duty_cycle = max(0, min(65535, duty_cycle))
        self._pca.channels[channel].duty_cycle = duty_cycle

    def release(self, channel: int) -> None:
        """Cut the PWM signal on a channel so the servo goes limp."""
        self._pca.channels[channel].duty_cycle = 0

    def close(self) -> None:
        self._pca.deinit()

    def __enter__(self) -> "Pca9685Driver":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
