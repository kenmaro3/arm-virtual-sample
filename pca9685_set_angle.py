#!/usr/bin/env python3
import time
import argparse
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

DEFAULT_ADDR = 0x40
DEFAULT_FREQ = 50
DEFAULT_MIN_US = 500
DEFAULT_MID_US = 1500
DEFAULT_MAX_US = 2500
DEFAULT_MAX_DEG = 270.0  # 270°品も想定。180°品は --max-deg 180 に

def us_to_duty16(us: float, freq: float) -> int:
    # 16bit(0..65535) で出力する
    ticks = 65535
    period_s = 1.0 / freq
    duty = int((us / 1_000_000.0) / period_s * ticks)
    return max(0, min(65535, duty))

def angle_to_us(angle_deg: float, min_us: float, max_us: float, max_deg: float) -> float:
    angle = max(0.0, min(max_deg, float(angle_deg)))
    return min_us + (max_us - min_us) * (angle / max_deg)

def main():
    ap = argparse.ArgumentParser(description="PCA9685: 任意CHを指定角度に移動/センター/リリース")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--angle", type=float)
    g.add_argument("--center", action="store_true")
    g.add_argument("--release", action="store_true")

    ap.add_argument("--channel", "-c", type=int, default=None)
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=DEFAULT_ADDR)
    ap.add_argument("--freq", type=float, default=DEFAULT_FREQ)
    ap.add_argument("--min-us", type=float, default=DEFAULT_MIN_US)
    ap.add_argument("--mid-us", type=float, default=DEFAULT_MID_US)
    ap.add_argument("--max-us", type=float, default=DEFAULT_MAX_US)
    ap.add_argument("--max-deg", type=float, default=DEFAULT_MAX_DEG)

    args = ap.parse_args()

    if args.channel is None:
        if args.angle is not None:
            ap.error("--angle を使うときは --channel を指定してください")
        channels = list(range(16))  # center/release は全CH
    else:
        if not (0 <= args.channel <= 15):
            ap.error("--channel は 0..15")
        channels = [args.channel]

    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c, address=args.addr)
    pca.frequency = int(args.freq)

    try:
        if args.center:
            duty = us_to_duty16(args.mid_us, args.freq)
            print(f"[INFO] center -> {args.mid_us}us (duty16={duty}) ch={channels}")
            for ch in channels:
                pca.channels[ch].duty_cycle = duty
                time.sleep(0.002)
            return

        if args.release:
            print(f"[INFO] release PWM ch={channels}")
            for ch in channels:
                pca.channels[ch].duty_cycle = 0
                time.sleep(0.002)
            return

        us = angle_to_us(args.angle, args.min_us, args.max_us, args.max_deg)
        duty = us_to_duty16(us, args.freq)
        print(f"[INFO] set ch={channels[0]} angle={args.angle:.1f}° -> {us:.1f}us (duty16={duty})")
        pca.channels[channels[0]].duty_cycle = duty

    finally:
        pca.deinit()

if __name__ == "__main__":
    main()

