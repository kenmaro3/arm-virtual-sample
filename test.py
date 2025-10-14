#!/usr/bin/env python3
"""
Simple smoke test that sweeps channel 0 between two pulse widths using the
same adafruit-circuitpython stack as the rest of the project.
"""
from __future__ import annotations

import time

from board import SCL, SDA  # type: ignore
import busio  # type: ignore
from adafruit_pca9685 import PCA9685  # type: ignore

from pca9685_dual_ds3218_sweep import DEFAULT_ADDR, DEFAULT_FREQ, us_to_duty  # type: ignore


def main() -> None:
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c, address=DEFAULT_ADDR)
    pca.frequency = DEFAULT_FREQ

    try:
        while True:
            # low angle pulse
            duty_lo = us_to_duty(500, DEFAULT_FREQ)
            pca.channels[0].duty_cycle = duty_lo
            time.sleep(1.0)

            # high angle pulse
            duty_hi = us_to_duty(2500, DEFAULT_FREQ)
            pca.channels[0].duty_cycle = duty_hi
            time.sleep(1.0)
    finally:
        pca.channels[0].duty_cycle = 0
        pca.deinit()


if __name__ == "__main__":
    main()
