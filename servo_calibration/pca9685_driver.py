"""Minimal PCA9685 wrapper for raw servo pulse-width control.

Deliberately thin: the calibration tool needs to command an exact pulse
width in microseconds on a given channel, nothing more. Higher-level
angle-based control belongs elsewhere, built on top of ServoCalibration.
"""

from __future__ import annotations

from adafruit_extended_bus import ExtendedI2C
from adafruit_pca9685 import PCA9685

DEFAULT_FREQUENCY_HZ = 50
DEFAULT_I2C_ADDRESS = 0x40

# rpi502's hardware I2C1 (GPIO2/GPIO3, the header's usual SDA/SCL) started
# throwing "lost arbitration" errors and stopped responding on 2026-08-27,
# reproduced across 3 different PCA9685 boards -> the Pi's own I2C1
# controller/pins, not the boards. Bus 3 is a software (bit-banged) I2C
# bus on GPIO23/GPIO24 instead, set up via `dtoverlay=i2c-gpio,bus=3` in
# /boot/firmware/config.txt, confirmed working. Wire the PCA9685's SDA/SCL
# there instead of the header's dedicated I2C pins until rpi502's I2C1 is
# repaired or the board is replaced.
DEFAULT_I2C_BUS = 3


class Pca9685Driver:
    def __init__(
        self,
        frequency_hz: int = DEFAULT_FREQUENCY_HZ,
        address: int = DEFAULT_I2C_ADDRESS,
        i2c_bus: int = DEFAULT_I2C_BUS,
    ) -> None:
        i2c = ExtendedI2C(i2c_bus)
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
