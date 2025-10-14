#!/usr/bin/env python3
# fastapi_app.py
from __future__ import annotations
import asyncio
import logging
import atexit, signal
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Mapping, MutableMapping, Optional, Tuple

from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import HTMLResponse

# ---- HW deps ----
try:
    from board import SCL, SDA  # type: ignore
    import busio  # type: ignore
    from adafruit_pca9685 import PCA9685  # type: ignore
except Exception as exc:
    raise RuntimeError(
        "Install adafruit-circuitpython-pca9685 and run on Raspberry Pi."
    ) from exc

# ===== Constants =====
BOARD_CHANNEL_LAYOUT: Mapping[int, Iterable[int]] = {
    0x40: range(4),  # first PCA9685 board
    0x41: range(4),  # second PCA9685 board
}


def _build_channel_map(layout: Mapping[int, Iterable[int]]) -> Dict[int, Tuple[int, int]]:
    mapping: Dict[int, Tuple[int, int]] = {}
    global_idx = 0
    for addr in sorted(layout.keys()):
        for local_ch in layout[addr]:
            mapping[global_idx] = (addr, int(local_ch))
            global_idx += 1
    return mapping


CHANNEL_MAP = _build_channel_map(BOARD_CHANNEL_LAYOUT)
PWM_FREQUENCY = 50  # Hz

CHANNELS: Tuple[int, ...] = tuple(CHANNEL_MAP.keys())  # UIで操作するチャネル
MIN_US, MID_US, MAX_US = 500.0, 1500.0, 2500.0
MIN_DEG, MAX_DEG = 0.0, 270.0   # 180°機なら 180.0 に
CENTER_ANGLE = MAX_DEG / 2.0

# 速度（deg/s）
SPEED_MIN_DPS, SPEED_MAX_DPS, SPEED_DEF_DPS = 10.0, 270.0, 90.0

# ===== Helpers =====
def us_to_duty16(us: float, freq_hz: float) -> int:
    ticks = 65535.0
    duty = int((us / 1_000_000.0) * freq_hz * ticks)
    return max(0, min(65535, duty))

def angle_to_us(angle_deg: float, min_us: float, max_us: float, max_deg: float) -> float:
    a = max(MIN_DEG, min(max_deg, float(angle_deg)))
    return min_us + (max_us - min_us) * (a / max_deg)

def clamp_angle(angle: float) -> float:
    return max(MIN_DEG, min(MAX_DEG, angle))

# ===== Controller =====
@dataclass
class ServoController:
    channel_map: Dict[int, Tuple[int, int]] = field(default_factory=lambda: dict(CHANNEL_MAP))
    pwm_freq: int = PWM_FREQUENCY

    _i2c: Optional[busio.I2C] = field(default=None, init=False)
    _pcas: Dict[int, PCA9685] = field(default_factory=dict, init=False)
    _channels: Tuple[int, ...] = field(init=False)
    _bus_lock: asyncio.Lock = field(default_factory=asyncio.Lock, init=False)     # I2C書き込み用ロック
    _angles: MutableMapping[int, float] = field(default_factory=dict, init=False)
    _speeds: MutableMapping[int, float] = field(default_factory=dict, init=False) # deg/s
    _move_tasks: MutableMapping[int, asyncio.Task] = field(default_factory=dict, init=False)

    def __post_init__(self) -> None:
        # defensive copy to avoid shared mutable default
        self.channel_map = dict(self.channel_map)
        self._channels = tuple(sorted(self.channel_map.keys()))

    @property
    def channels(self) -> Tuple[int, ...]:
        return self._channels

    def _resolve_channel(self, channel: int) -> Tuple[int, int]:
        try:
            return self.channel_map[channel]
        except KeyError:
            raise HTTPException(status_code=404, detail=f"Channel {channel} not available")

    # ---- lifecycle ----
    def start(self) -> None:
        addresses = sorted({addr for addr, _ in self.channel_map.values()})
        logging.info(
            "Initializing I2C + PCA9685 boards at: %s",
            ", ".join(f"0x{addr:02X}" for addr in addresses),
        )
        self._i2c = busio.I2C(SCL, SDA)
        self._pcas.clear()
        for addr in addresses:
            pca = PCA9685(self._i2c, address=addr)
            pca.frequency = int(self.pwm_freq)
            self._pcas[addr] = pca

        self._angles.clear()
        self._speeds.clear()

        for ch in self.channels:
            self._write_us(ch, MID_US)               # センターPWM出力
            self._angles[ch] = CENTER_ANGLE
            self._speeds[ch] = SPEED_DEF_DPS

    def stop(self) -> None:
        logging.info("Releasing all channels and deinit PCA9685")
        # タスク停止
        for ch, t in list(self._move_tasks.items()):
            t.cancel()
        self._move_tasks.clear()
        if self._pcas:
            try:
                for ch in self.channels:
                    self._write_duty(ch, 0)
            finally:
                for addr, pca in self._pcas.items():
                    try:
                        pca.deinit()
                    except Exception:
                        logging.exception("Failed to deinit PCA9685 at 0x%02X", addr)
                self._pcas.clear()
        if self._i2c:
            try:
                self._i2c.deinit()  # type: ignore[attr-defined]
            except AttributeError:
                pass
            self._i2c = None

    # ---- public APIs ----
    async def set_speed(self, channel: int, dps: float) -> float:
        self._validate_channel(channel)
        dps = float(max(SPEED_MIN_DPS, min(SPEED_MAX_DPS, dps)))
        self._speeds[channel] = dps
        return dps

    async def set_angle(self, channel: int, target_angle: float) -> float:
        self._validate_channel(channel)
        target_angle = clamp_angle(target_angle)

        # 既存の移動をキャンセルして最新に
        self._cancel_move(channel)

        # 新しい移動タスクを起動（非同期でスムーズ移動）
        task = asyncio.create_task(self._move_to_angle(channel, target_angle))
        self._move_tasks[channel] = task
        # すぐに応答を返す（実移動はバックグラウンドで進行）
        return target_angle

    async def center_all(self) -> float:
        # 各チャネルにスムーズ移動を指示
        tasks = [self.set_angle(ch, CENTER_ANGLE) for ch in self.channels]
        await asyncio.gather(*tasks, return_exceptions=True)
        return CENTER_ANGLE

    async def release_all(self) -> None:
        # すべての移動を止めて PWM停止
        for ch in list(self.channels):
            self._cancel_move(ch)
        for ch in self.channels:
            await self._write_duty_async(ch, 0)
            self._angles[ch] = 0.0

    def current_state(self) -> Dict[str, Any]:
        return {
            "angles": {str(ch): ang for ch, ang in self._angles.items()},
            "speeds": {str(ch): spd for ch, spd in self._speeds.items()},
            "max_deg": MAX_DEG,
            "channel_info": self.channel_metadata(),
        }

    def channel_metadata(self) -> Dict[str, Dict[str, Any]]:
        info: Dict[str, Dict[str, Any]] = {}
        for ch in self.channels:
            addr, local_ch = self.channel_map[ch]
            info[str(ch)] = {
                "address": addr,
                "address_hex": f"0x{addr:02X}",
                "channel": local_ch,
            }
        return info

    # ---- movement core ----
    async def _move_to_angle(self, channel: int, target: float) -> None:
        """
        現在角度 self._angles[channel] から target まで、
        設定された速度（deg/s）で線形補間しながら更新する。
        """
        try:
            cur = float(self._angles.get(channel, CENTER_ANGLE))
            dps = float(self._speeds.get(channel, SPEED_DEF_DPS))
            if dps <= 0.0:
                dps = SPEED_MIN_DPS

            dt = 0.02  # 20ms ステップ
            eps = 0.2  # 収束閾値（deg）
            while True:
                err = target - cur
                if abs(err) <= eps:
                    cur = target
                    await self._write_angle_async(channel, cur)
                    self._angles[channel] = cur
                    break

                step = dps * dt
                if abs(err) < step:
                    cur = target
                else:
                    cur += step if err > 0 else -step

                await self._write_angle_async(channel, cur)
                self._angles[channel] = cur
                await asyncio.sleep(dt)
        except asyncio.CancelledError:
            # 新しい指令でキャンセルされた
            pass
        except (OSError, RuntimeError) as e:
            addr, _ = self.channel_map.get(channel, (None, None))
            addr_display = addr if addr is not None else 0
            logging.warning(
                "I2C/PCA error during move (ch=%d addr=0x%02X): %s",
                channel,
                addr_display,
                e,
            )
            # 簡易リカバリ：再初期化して一度だけ再試行（角度は最新を保持）
            try:
                if addr is None or not self._i2c:
                    return
                existing = self._pcas.get(addr)
                if existing:
                    existing.deinit()
                new_pca = PCA9685(self._i2c, address=addr)
                new_pca.frequency = int(self.pwm_freq)
                self._pcas[addr] = new_pca
            except Exception as ee:
                logging.error("Re-init failed for 0x%02X: %s", addr_display, ee)

    def _cancel_move(self, channel: int) -> None:
        t = self._move_tasks.pop(channel, None)
        if t and not t.done():
            t.cancel()

    # ---- low-level writes ----
    async def _write_angle_async(self, channel: int, angle_deg: float) -> None:
        us = angle_to_us(angle_deg, MIN_US, MAX_US, MAX_DEG)
        duty = us_to_duty16(us, self.pwm_freq)
        await self._write_duty_async(channel, duty)

    async def _write_duty_async(self, channel: int, duty: int) -> None:
        async with self._bus_lock:
            self._write_duty(channel, duty)

    def _write_us(self, channel: int, us: float) -> None:
        duty = us_to_duty16(us, self.pwm_freq)
        self._write_duty(channel, duty)

    def _write_duty(self, channel: int, duty: int) -> None:
        addr, local_ch = self._resolve_channel(channel)
        p = self._ensure_pca(addr)
        p.channels[local_ch].duty_cycle = duty

    def _ensure_pca(self, addr: int) -> PCA9685:
        p = self._pcas.get(addr)
        if not p:
            raise RuntimeError(f"PCA9685 at 0x{addr:02X} is not initialized")
        return p

    def _validate_channel(self, ch: int) -> Tuple[int, int]:
        if not (0 <= ch <= 15):
            raise HTTPException(status_code=400, detail="channel must be 0..15")
        try:
            return self.channel_map[ch]
        except KeyError:
            raise HTTPException(status_code=404, detail=f"Channel {ch} not available")

# ===== App wiring =====
controller = ServoController()

def _cleanup(*_args) -> None:
    try:
        controller.stop()
    except Exception:
        logging.exception("cleanup failed")

signal.signal(signal.SIGTERM, _cleanup)
signal.signal(signal.SIGINT, _cleanup)
atexit.register(_cleanup)

app = FastAPI(title="PCA9685 Servo Control", version="1.2.0")

@app.on_event("startup")
async def startup_event() -> None:
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, controller.start)

@app.on_event("shutdown")
async def shutdown_event() -> None:
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, controller.stop)

@app.get("/", response_class=HTMLResponse)
async def index() -> str:
    return HTML_TEMPLATE

@app.get("/status")
async def status() -> Dict[str, Any]:
    return {"ok": True, **controller.current_state()}

@app.post("/servo/{channel}")
async def set_servo_angle(
    channel: int,
    angle: float = Query(..., ge=MIN_DEG, le=MAX_DEG),
) -> Dict[str, Any]:
    target = await controller.set_angle(channel, angle)
    return {"ok": True, "channel": channel, "angle": target}

@app.post("/speed/{channel}")
async def set_speed(
    channel: int,
    dps: float = Query(..., ge=SPEED_MIN_DPS, le=SPEED_MAX_DPS),
) -> Dict[str, Any]:
    v = await controller.set_speed(channel, dps)
    return {"ok": True, "channel": channel, "dps": v}

@app.post("/center")
async def center() -> Dict[str, Any]:
    angle = await controller.center_all()
    return {"ok": True, "angle": angle}

@app.post("/release")
async def release() -> Dict[str, Any]:
    await controller.release_all()
    return {"ok": True}

# ===== UI =====
HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="ja">
  <head>
    <meta charset="utf-8" />
    <title>PCA9685 Servo Control</title>
    <style>
      :root {{ color-scheme: light dark; font-family: system-ui, sans-serif; }}
      body {{ max-width: 680px; margin: 0 auto; padding: 1.5rem; }}
      h1 {{ margin-bottom: 0.5rem; text-align: center; }}
      .servo-card {{ border: 1px solid #888; border-radius: 8px; padding: 1rem; margin-bottom: 1rem; }}
      .servo-title {{ display: flex; justify-content: space-between; margin-bottom: 0.5rem; }}
      .row {{ display: grid; grid-template-columns: 80px 1fr 80px; gap: .5rem; align-items: center; }}
      input[type="range"] {{ width: 100%; }}
      button {{ padding: 0.4rem 0.8rem; margin-right: 0.5rem; }}
      #status {{ font-size: 0.9rem; margin-bottom: 1.5rem; }}
      .actions {{ display: flex; gap: 0.5rem; justify-content: center; margin-bottom: 1rem; flex-wrap: wrap; }}
      .muted {{ opacity: .8; font-size: .9em; }}
    </style>
  </head>
  <body>
    <h1>PCA9685 Servo Control</h1>
    <p id="status">サーボ位置を取得しています...</p>
    <div class="actions">
      <button onclick="centerAll()">Center All</button>
      <button onclick="releaseAll()">Release All</button>
      <button onclick="refreshStatus()">Refresh</button>
      <span class="muted">Max Deg: <strong id="maxdeg"></strong></span>
    </div>
    <div id="servo-container"></div>
    <script>
      let CHANNELS = [];
      let CHANNEL_META = {{}};
      const SPEED_MIN = {SPEED_MIN_DPS}, SPEED_MAX = {SPEED_MAX_DPS};

      async function api(url, options={{}}) {{
        const res = await fetch(url, {{ headers: {{ "Content-Type": "application/json" }}, ...options }});
        if (!res.ok) throw new Error(await res.text() || "Request failed");
        return res.json();
      }}

      const moveTimers = new Map();
      const speedTimers = new Map();

      function debounceMap(map, key, fn, wait=200) {{
        if (map.has(key)) clearTimeout(map.get(key));
        map.set(key, setTimeout(fn, wait));
      }}

      function render(data) {{
        const angles = data.angles || {{}};
        const speeds = data.speeds || {{}};
        const maxdeg = data.max_deg ?? 270;
        CHANNEL_META = data.channel_info || {{}};
        CHANNELS = Object.keys(CHANNEL_META).map((key) => Number(key)).sort((a, b) => a - b);
        document.getElementById("maxdeg").textContent = String(maxdeg);

        const container = document.getElementById("servo-container");
        container.innerHTML = "";

        if (CHANNELS.length === 0) {{
          container.innerHTML = "<p class='muted'>制御可能なサーボが見つかりません。</p>";
          return;
        }}

        CHANNELS.forEach((ch) => {{
          const ang = Number(angles[String(ch)] ?? maxdeg/2);
          const spd = Number(speeds[String(ch)] ?? {SPEED_DEF_DPS});
          const meta = CHANNEL_META[String(ch)] || {{}};
          const addr = meta.address;
          const addrHex = meta.address_hex || (addr !== undefined ? "0x" + Number(addr).toString(16).padStart(2, "0") : "0x??");
          const localCh = meta.channel ?? ch;

          const card = document.createElement("div");
          card.className = "servo-card";
          card.innerHTML = `
            <div class="servo-title">
              <div>
                <strong>Channel ${{ch}}</strong>
                <div class="muted">Board ${{addrHex}} / CH ${{localCh}}</div>
              </div>
              <span>
                <span id="angle-display-${{ch}}">${{ang.toFixed(1)}}°</span> /
                <span id="speed-display-${{ch}}">${{spd.toFixed(0)}} deg/s</span>
              </span>
            </div>
            <div class="row">
              <span>Angle</span>
              <input type="range" min="0" max="${{maxdeg}}" step="1" value="${{ang}}" id="slider-angle-${{ch}}">
              <button onclick="nudge(${{ch}}, -5)">-5°</button>
            </div>
            <div class="row" style="margin-top:.5rem">
              <span>Speed</span>
              <input type="range" min="${{SPEED_MIN}}" max="${{SPEED_MAX}}" step="1" value="${{spd}}" id="slider-speed-${{ch}}">
              <button onclick="nudge(${{ch}}, +5)">+5°</button>
            </div>
          `;
          container.appendChild(card);

          const angleSlider = document.getElementById(`slider-angle-${{ch}}`);
          angleSlider.addEventListener("input", (e) => {{
            const v = Number(e.target.value);
            document.getElementById(`angle-display-${{ch}}`).textContent = v.toFixed(1) + "°";
            debounceMap(moveTimers, ch, async () => {{
              try {{
                setStatus(`CH${{ch}}: 目標 ${{v}}°`);
                await api(`/servo/${{ch}}?angle=${{v}}`, {{ method: "POST" }});
              }} catch (err) {{
                console.error(err);
                setStatus(`CH${{ch}}: 送信失敗 ${{err.message}}`, true);
              }}
            }}, 150);
          }});

          const speedSlider = document.getElementById(`slider-speed-${{ch}}`);
          speedSlider.addEventListener("input", (e) => {{
            const dps = Number(e.target.value);
            document.getElementById(`speed-display-${{ch}}`).textContent = dps.toFixed(0) + " deg/s";
            debounceMap(speedTimers, `s-${{ch}}`, async () => {{
              try {{
                await api(`/speed/${{ch}}?dps=${{dps}}`, {{ method: "POST" }});
                setStatus(`CH${{ch}}: 速度を ${{dps.toFixed(0)}} deg/s に設定`);
              }} catch (err) {{
                console.error(err);
                setStatus(`CH${{ch}}: 速度設定失敗 ${{err.message}}`, true);
              }}
            }}, 200);
          }});
        }});
      }}

      function setStatus(msg, isError=false) {{
        const s = document.getElementById("status");
        s.textContent = msg;
        s.style.color = isError ? "tomato" : "inherit";
      }}

      async function refreshStatus() {{
        try {{
          setStatus("更新中...");
          const data = await api("/status");
          render(data);
          setStatus("最新化しました。");
        }} catch (err) {{
          console.error(err);
          setStatus("ステータス取得に失敗: " + err.message, true);
        }}
      }}

      async function centerAll() {{
        try {{
          setStatus("センター移動中...");
          await api("/center", {{ method: "POST" }});
          await refreshStatus();
          setStatus("センター指示を出しました（速度に従って移動）。");
        }} catch (err) {{
          console.error(err);
          setStatus("センター失敗: " + err.message, true);
        }}
      }}

      async function releaseAll() {{
        try {{
          setStatus("解放中...");
          await api("/release", {{ method: "POST" }});
          await refreshStatus();
          setStatus("すべてのサーボを解放しました。");
        }} catch (err) {{
          console.error(err);
          setStatus("解放失敗: " + err.message, true);
        }}
      }}

      async function nudge(ch, delta) {{
        try {{
          const status = await api("/status");
          const maxdeg = status.max_deg ?? 270;
          const cur = Number(status.angles?.[String(ch)] ?? maxdeg/2);
          const v = Math.max(0, Math.min(maxdeg, cur + delta));
          await api(`/servo/${{ch}}?angle=${{v}}`, {{ method: "POST" }});
          setStatus("CH" + ch + ": " + (delta > 0 ? "+5°" : "-5°") + " 指示");
          await refreshStatus();
        }} catch (err) {{
          console.error(err);
          setStatus("Nudge失敗: " + err.message, true);
        }}
      }}

      window.addEventListener("load", refreshStatus);
    </script>
  </body>
</html>
""".format(
    SPEED_MIN_DPS=SPEED_MIN_DPS,
    SPEED_MAX_DPS=SPEED_MAX_DPS,
    SPEED_DEF_DPS=SPEED_DEF_DPS,
)

if __name__ == "__main__":
    import uvicorn
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
    uvicorn.run("fastapi_app:app", host="0.0.0.0", port=8000, reload=False)
