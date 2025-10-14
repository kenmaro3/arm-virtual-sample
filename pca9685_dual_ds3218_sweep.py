#!/usr/bin/env python3
import time
import argparse
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

# ===== デフォルト設定（必要に応じて調整） =====
DEFAULT_ADDR = 0x40
DEFAULT_FREQ = 50               # サーボは 50Hz
DEFAULT_MIN_US = 500            # 端の安全側。個体差に合わせて微調整可
DEFAULT_MID_US = 1500           # センター
DEFAULT_MAX_US = 2500           # 端の安全側。個体差に合わせて微調整可
DEFAULT_MAX_DEG = 270.0         # 要望に合わせ 0–270° を扱う

def us_to_duty(us: float, freq: float) -> int:
    """us→12bit duty(0..4095)"""
    ticks = 4096
    period_s = 1.0 / freq
    duty = int((us / 1_000_000.0) / period_s * ticks)
    return max(0, min(4095, duty))

def angle_to_us(angle_deg: float, min_us: float, max_us: float, max_deg: float) -> float:
    """角度(0..max_deg)→us。レンジ外はクランプ"""
    angle = max(0.0, min(max_deg, float(angle_deg)))
    return min_us + (max_us - min_us) * (angle / max_deg)

def main():
    ap = argparse.ArgumentParser(
        description="PCA9685 + DS3218: 任意CHを指定角度に移動 / センター / リリース"
    )
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--angle", type=float, help="指定角度（0..MAX_DEG）")
    g.add_argument("--center", action="store_true", help="センター(1500us)に戻す")
    g.add_argument("--release", action="store_true", help="PWM停止（トルク解除）")

    ap.add_argument("--channel", "-c", type=int, default=None,
                    help="対象チャネル（0..15）。未指定で --center/--release の場合は全CH処理")
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=DEFAULT_ADDR,
                    help="I2Cアドレス（例: 0x40）")
    ap.add_argument("--freq", type=float, default=DEFAULT_FREQ, help="PWM周波数(Hz)")
    ap.add_argument("--min-us", type=float, default=DEFAULT_MIN_US, help="最小パルス幅(us)")
    ap.add_argument("--mid-us", type=float, default=DEFAULT_MID_US, help="センター(us)")
    ap.add_argument("--max-us", type=float, default=DEFAULT_MAX_US, help="最大パルス幅(us)")
    ap.add_argument("--max-deg", type=float, default=DEFAULT_MAX_DEG,
                    help="角度レンジの最大値（例：180 or 270）")

    args = ap.parse_args()

    # チャネルの解決
    if args.channel is None:
        if args.angle is not None:
            ap.error("--angle を使うときは --channel を指定してください")
        # center / release で channel 未指定 → 全CH
        channels = list(range(16))
    else:
        if not (0 <= args.channel <= 15):
            ap.error("--channel は 0..15 の範囲です")
        channels = [args.channel]

    # I2C 初期化
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c, address=args.addr)
    pca.frequency = int(args.freq)

    try:
        if args.center:
            print(f"[INFO] center: us={args.mid_us} → channels={channels}")
            duty = us_to_duty(args.mid_us, args.freq)
            for ch in channels:
                pca.channels[ch].duty_cycle = duty
                time.sleep(0.002)
            return

        if args.release:
            print(f"[INFO] release: PWM off → channels={channels}")
            for ch in channels:
                pca.channels[ch].duty_cycle = 0
                time.sleep(0.002)
            return

        # angle 指定
        us = angle_to_us(args.angle, args.min_us, args.max_us, args.max_deg)
        duty = us_to_duty(us, args.freq)
        print(f"[INFO] set: ch={channels[0]} angle={args.angle:.2f}° "
              f"-> us={us:.1f} duty={duty}")
        pca.channels[channels[0]].duty_cycle = duty

    finally:
        # 連続制御の予定がなければ毎回閉じる
        pca.deinit()

if __name__ == "__main__":
    main()

