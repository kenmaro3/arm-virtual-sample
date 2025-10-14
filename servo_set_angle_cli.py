#!/usr/bin/env python3
"""
PCA9685 + DS3218 を CLI から制御するユーティリティ。

`pca9685_dual_ds3218_sweep.py` と同じオプション体系 (--angle/--center/--release) を
提供し、同じデフォルト値でパルス幅や角度の変換を行います。
"""
from __future__ import annotations

import argparse
import time
from typing import Iterable, List

from board import SCL, SDA  # type: ignore
import busio  # type: ignore
from adafruit_pca9685 import PCA9685  # type: ignore

from pca9685_dual_ds3218_sweep import (  # type: ignore
    DEFAULT_ADDR,
    DEFAULT_FREQ,
    DEFAULT_MAX_DEG,
    DEFAULT_MAX_US,
    DEFAULT_MID_US,
    DEFAULT_MIN_US,
    angle_to_us,
    us_to_duty,
)


def parse_args() -> tuple[argparse.ArgumentParser, argparse.Namespace]:
    parser = argparse.ArgumentParser(
        description="PCA9685 DS3218 サーボ制御 CLI (--angle/--center/--release)"
    )
    g = parser.add_mutually_exclusive_group(required=True)
    g.add_argument("--angle", type=float, help="指定角度 (0..MAX_DEG)")
    g.add_argument("--center", action="store_true", help="センターへ移動 (mid-us)")
    g.add_argument("--release", action="store_true", help="PWM を停止して解放")

    parser.add_argument(
        "--channel",
        "-c",
        type=int,
        default=None,
        help="対象チャネル (0..15)。未指定時 --center/--release は全チャネル",
    )
    parser.add_argument(
        "--addr",
        type=lambda s: int(s, 0),
        default=DEFAULT_ADDR,
        help="I2C アドレス (例: 0x40)",
    )
    parser.add_argument("--freq", type=float, default=DEFAULT_FREQ, help="PWM 周波数 (Hz)")
    parser.add_argument("--min-us", type=float, default=DEFAULT_MIN_US, help="最小パルス幅 (µs)")
    parser.add_argument("--mid-us", type=float, default=DEFAULT_MID_US, help="センターパルス幅 (µs)")
    parser.add_argument("--max-us", type=float, default=DEFAULT_MAX_US, help="最大パルス幅 (µs)")
    parser.add_argument("--max-deg", type=float, default=DEFAULT_MAX_DEG, help="扱う最大角度 (deg)")
    parser.add_argument(
        "--sleep",
        type=float,
        default=0.0,
        help="マルチチャネル処理時の待ち時間 (秒)",
    )
    return parser, parser.parse_args()


def resolve_channels(args: argparse.Namespace, parser: argparse.ArgumentParser) -> List[int]:
    if args.channel is None:
        if args.angle is not None:
            parser.error("--angle を使用する場合は --channel を指定してください")
        return list(range(16))

    if not (0 <= args.channel <= 15):
        parser.error("--channel は 0..15 の範囲で指定してください")
    return [args.channel]


def apply_duty_cycle(pca: PCA9685, channels: Iterable[int], duty: int, delay: float) -> None:
    for ch in channels:
        pca.channels[ch].duty_cycle = duty
        if delay > 0:
            time.sleep(delay)


def main() -> None:
    parser, args = parse_args()
    channels = resolve_channels(args, parser)

    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c, address=args.addr)
    pca.frequency = int(args.freq)

    try:
        if args.center:
            duty = us_to_duty(args.mid_us, args.freq)
            print(f"[INFO] center: us={args.mid_us} -> channels={channels}")
            apply_duty_cycle(pca, channels, duty, args.sleep)
            return

        if args.release:
            print(f"[INFO] release: PWM off -> channels={channels}")
            apply_duty_cycle(pca, channels, 0, args.sleep)
            return

        # angle specified (channels を 1 つに限定済み)
        angle = float(args.angle)
        us = angle_to_us(angle, args.min_us, args.max_us, args.max_deg)
        duty = us_to_duty(us, args.freq)
        ch = channels[0]
        print(
            f"[INFO] set: ch={ch} angle={angle:.2f}° "
            f"-> us={us:.1f} duty={duty}"
        )
        pca.channels[ch].duty_cycle = duty
    finally:
        pca.deinit()


if __name__ == "__main__":
    main()
