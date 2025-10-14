#!/usr/bin/env python3
import atexit
import signal
import threading
from typing import Dict, Any

from flask import Flask, request, jsonify

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

# ===== ハード設定 =====
I2C_ADDR = 0x40
PWM_FREQ = 50
CH_VALID = {0, 1}

MIN_US = 500
MID_US = 1500
MAX_US = 2500

def us_to_duty(us: float, freq: float = PWM_FREQ) -> int:
    period_s = 1.0 / freq
    ticks = 4096
    duty = int((us / 1_000_000.0) / period_s * ticks)
    return max(0, min(4095, duty))

def angle_to_us(angle_deg: float,
                min_us: float = MIN_US,
                max_us: float = MAX_US,
                min_deg: float = 0.0,
                max_deg: float = 180.0) -> float:
    angle = max(min_deg, min(max_deg, float(angle_deg)))
    return min_us + (max_us - min_us) * (angle - min_deg) / (max_deg - min_deg)

# ===== PCA9685 =====
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=I2C_ADDR)
pca.frequency = PWM_FREQ
lock = threading.Lock()

def set_us(ch: int, us: float):
    with lock:
        print(f"[DEBUG] set_us: channel={ch}, us={us}")
        pca.channels[ch].duty_cycle = us_to_duty(us)

def release(ch: int):
    with lock:
        print(f"[DEBUG] release: channel={ch}")
        pca.channels[ch].duty_cycle = 0

def cleanup(*_args):
    print("[DEBUG] cleanup: releasing all channels")
    try:
        for ch in CH_VALID:
            release(ch)
    finally:
        pca.deinit()

signal.signal(signal.SIGTERM, cleanup)
signal.signal(signal.SIGINT, cleanup)
atexit.register(cleanup)

# ===== Flask =====
app = Flask(__name__)

def bad_request(msg: str, status: int = 400):
    print(f"[ERROR] bad_request: {msg}")
    return jsonify({"ok": False, "error": msg}), status

@app.route("/servo", methods=["POST"])
def set_servo_angle():
    try:
        data: Dict[str, Any] = request.get_json(force=True, silent=False)
        print(f"[DEBUG] /servo called with data={data}")
    except Exception as e:
        return bad_request(f"Invalid JSON: {e}")

    if "channel" not in data or "angle" not in data:
        return bad_request("Required keys: channel, angle")

    try:
        ch = int(data["channel"])
        angle = float(data["angle"])
    except ValueError:
        return bad_request("channel must be int, angle must be number")

    if ch not in CH_VALID:
        return bad_request(f"channel must be in {sorted(CH_VALID)}")

    us = angle_to_us(angle)
    set_us(ch, us)

    return jsonify({"ok": True, "channel": ch, "angle": angle, "us": us})

@app.route("/center", methods=["POST"])
def center_all():
    print("[DEBUG] /center called")
    for ch in CH_VALID:
        set_us(ch, MID_US)
    return jsonify({"ok": True, "center_us": MID_US})

@app.route("/release", methods=["POST"])
def release_all():
    print("[DEBUG] /release called")
    for ch in CH_VALID:
        release(ch)
    return jsonify({"ok": True, "released_channels": sorted(CH_VALID)})

@app.route("/healthz", methods=["GET"])
def health():
    print("[DEBUG] /healthz called")
    return jsonify({"ok": True})

if __name__ == "__main__":
    print("[DEBUG] Starting Flask server on 0.0.0.0:8000")
    app.run(host="0.0.0.0", port=8000, debug=True)

