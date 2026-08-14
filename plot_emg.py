#!/usr/bin/env python3
"""Realtime EMG plotter. Connects to the collector TCP hub as a LAN client."""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import threading
import time
from collections import deque

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError as exc:
    raise SystemExit(
        "Thiếu matplotlib. Cài dependency bằng: python -m pip install -r requirements.txt"
    ) from exc


PACKET_HEADER = 0xAA55
PACKET_VERSION = 2
PACKET_STRUCT = struct.Struct("<HBIIHQH")
PACKET_SIZE = PACKET_STRUCT.size
HEADER_BYTES = struct.pack("<H", PACKET_HEADER)
DEFAULT_TCP_PORT = 8765
TIMESTAMP_WRAP = 1 << 32


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class PacketParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        while True:
            header_index = self.buffer.find(HEADER_BYTES)
            if header_index < 0:
                del self.buffer[: max(0, len(self.buffer) - 1)]
                return
            if header_index:
                del self.buffer[:header_index]
            if len(self.buffer) < PACKET_SIZE:
                return
            raw = bytes(self.buffer[:PACKET_SIZE])
            header, version, packet_id, timestamp_us, emg, _utc, expected_crc = (
                PACKET_STRUCT.unpack(raw)
            )
            if (
                header != PACKET_HEADER
                or version != PACKET_VERSION
                or crc16_ccitt(raw[:-2]) != expected_crc
            ):
                del self.buffer[0]
                continue
            del self.buffer[:PACKET_SIZE]
            yield packet_id, timestamp_us, emg


class LiveBuffer:
    def __init__(self, window_s: float) -> None:
        self.window_s = window_s
        self.lock = threading.Lock()
        self.times: deque[float] = deque()
        self.values: deque[float] = deque()
        self.packets = 0
        self.started = time.monotonic()
        self._wraps = 0
        self._prev_ts: int | None = None
        self._origin_us: int | None = None

    def add(self, timestamp_us: int, emg: int, vref: float | None) -> None:
        if self._prev_ts is not None and self._prev_ts - timestamp_us > TIMESTAMP_WRAP // 2:
            self._wraps += 1
        self._prev_ts = timestamp_us
        unwrapped = timestamp_us + self._wraps * TIMESTAMP_WRAP
        if self._origin_us is None:
            self._origin_us = unwrapped
        t = (unwrapped - self._origin_us) / 1_000_000
        y = emg * vref / 1023.0 if vref is not None else float(emg)
        cutoff = t - self.window_s
        with self.lock:
            self.times.append(t)
            self.values.append(y)
            self.packets += 1
            while self.times and self.times[0] < cutoff:
                self.times.popleft()
                self.values.popleft()

    def reset_timeline(self) -> None:
        with self.lock:
            self.times.clear()
            self.values.clear()
            self._wraps = 0
            self._prev_ts = None
            self._origin_us = None

    def snapshot(self) -> tuple[list[float], list[float], int, float]:
        with self.lock:
            elapsed = max(time.monotonic() - self.started, 1e-9)
            return list(self.times), list(self.values), self.packets, self.packets / elapsed


def minmax_downsample(
    times: list[float], values: list[float], max_points: int
) -> tuple[list[float], list[float]]:
    """Keep min/max in each bin so a dense EMG trace stays readable."""
    n = len(values)
    if n <= max_points:
        return times, values
    bins = max(1, max_points // 2)
    xs: list[float] = []
    ys: list[float] = []
    for i in range(bins):
        start = (i * n) // bins
        end = ((i + 1) * n) // bins
        if start >= end:
            continue
        sl_t = times[start:end]
        sl_y = values[start:end]
        min_i = 0
        max_i = 0
        min_v = sl_y[0]
        max_v = sl_y[0]
        for j, value in enumerate(sl_y):
            if value < min_v:
                min_v = value
                min_i = j
            if value > max_v:
                max_v = value
                max_i = j
        first, second = (min_i, max_i) if min_i <= max_i else (max_i, min_i)
        xs.append(sl_t[first])
        ys.append(sl_y[first])
        if second != first:
            xs.append(sl_t[second])
            ys.append(sl_y[second])
    return xs, ys


def rolling_rms(values: list[float], win: int) -> list[float]:
    n = len(values)
    if n == 0:
        return []
    win = max(1, min(win, n))
    out = [0.0] * n
    sum_sq = 0.0
    for i, value in enumerate(values):
        sum_sq += value * value
        if i >= win:
            old = values[i - win]
            sum_sq -= old * old
        count = win if i >= win else i + 1
        out[i] = math.sqrt(max(sum_sq, 0.0) / count)
    return out


def y_limits(values: list[float], vref: float | None) -> tuple[float, float]:
    ymax = max(values)
    ymin = min(values)
    if vref is None:
        step = 200.0
        pad = max((ymax - ymin) * 0.12, 40.0)
        hi = math.ceil((ymax + pad) / step) * step
        lo = 0.0 if ymin >= 0 else math.floor((ymin - pad * 0.25) / step) * step
        return lo, max(hi, step)
    step = 0.5
    pad = max((ymax - ymin) * 0.12, 0.08)
    hi = math.ceil((ymax + pad) / step) * step
    lo = 0.0 if ymin >= 0 else math.floor((ymin - pad * 0.25) / step) * step
    return lo, max(hi, step)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Vẽ EMG realtime từ TCP hub của collect_emg.py."
    )
    parser.add_argument("--host", default="127.0.0.1", help="IP máy collector")
    parser.add_argument(
        "--port", type=int, default=DEFAULT_TCP_PORT, help="Cổng TCP"
    )
    parser.add_argument(
        "--window",
        type=float,
        default=5.0,
        help="Độ dài cửa sổ trượt (giây)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Số lần vẽ mỗi giây (không vẽ từng packet)",
    )
    parser.add_argument(
        "--vref",
        type=float,
        help="Quy đổi ADC sang Volt theo Vref (5 hoặc 3.3). Mặc định giữ ADC count",
    )
    return parser.parse_args()


def receiver_loop(
    host: str,
    port: int,
    buffer: LiveBuffer,
    vref: float | None,
    stop: threading.Event,
    conn_state: dict[str, str],
) -> None:
    while not stop.is_set():
        conn_state["status"] = "connecting"
        try:
            sock = socket.create_connection((host, port), timeout=3.0)
        except OSError:
            conn_state["status"] = "reconnecting"
            stop.wait(1.0)
            continue
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(0.2)
        parser = PacketParser()
        buffer.reset_timeline()
        conn_state["status"] = "connected"
        try:
            while not stop.is_set():
                try:
                    chunk = sock.recv(4096)
                except TimeoutError:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                for _packet_id, timestamp_us, emg in parser.feed(chunk):
                    buffer.add(timestamp_us, emg, vref)
        finally:
            try:
                sock.close()
            except OSError:
                pass
        if not stop.is_set():
            conn_state["status"] = "reconnecting"
            stop.wait(0.5)


def run(args: argparse.Namespace) -> int:
    if args.window <= 0:
        raise ValueError("--window phải lớn hơn 0")
    if args.fps <= 0:
        raise ValueError("--fps phải lớn hơn 0")
    if args.vref is not None and args.vref <= 0:
        raise ValueError("--vref phải lớn hơn 0")

    buffer = LiveBuffer(args.window)
    stop = threading.Event()
    conn_state = {"status": "connecting"}
    thread = threading.Thread(
        target=receiver_loop,
        args=(args.host, args.port, buffer, args.vref, stop, conn_state),
        daemon=True,
    )
    thread.start()

    ylabel = "EMG (V)" if args.vref is not None else "EMG (ADC count)"
    fig, ax = plt.subplots(figsize=(11, 4.5))
    (raw_line,) = ax.plot([], [], lw=0.7, color="#4c78a8", alpha=0.55, label="raw")
    (env_line,) = ax.plot([], [], lw=1.8, color="#e45756", label="RMS")
    status = ax.text(
        0.01,
        0.97,
        "",
        transform=ax.transAxes,
        va="top",
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "alpha": 0.85},
    )
    ax.set_xlim(-args.window, 0)
    ax.set_ylim(0, 1200 if args.vref is None else args.vref * 1.2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(f"SparkFun MyoWare  {args.host}:{args.port}")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", framealpha=0.85)
    fig.tight_layout()
    last_ylim = [0.0, 1200.0 if args.vref is None else args.vref * 1.2]

    def fmt_value(value: float) -> str:
        return f"{value:.0f} ADC" if args.vref is None else f"{value:.3f} V"

    def update(_frame: int):
        times, values, packets, rate = buffer.snapshot()
        if times:
            t0 = times[-1]
            rel = [t - t0 for t in times]
            xs, ys = minmax_downsample(rel, values, max_points=1600)
            raw_line.set_data(xs, ys)
            rms_win = max(8, int(rate * 0.08))
            env_line.set_data(rel, rolling_rms(values, rms_win))
            lo, hi = y_limits(values, args.vref)
            if (lo, hi) != (last_ylim[0], last_ylim[1]):
                ax.set_ylim(lo, hi)
                last_ylim[0], last_ylim[1] = lo, hi
            vmin = min(values)
            vmax = max(values)
            status.set_text(
                f"{rate:.0f} Hz   {packets} pkt   "
                f"now {fmt_value(values[-1])}   "
                f"min {fmt_value(vmin)}   max {fmt_value(vmax)}   "
                f"[{conn_state['status']}]"
            )
        else:
            status.set_text(f"TCP {conn_state['status']}   {args.host}:{args.port}")
        return raw_line, env_line, status

    _anim = FuncAnimation(
        fig,
        update,
        interval=max(int(1000 / args.fps), 10),
        blit=False,
        cache_frame_data=False,
    )
    plt.show()
    stop.set()
    thread.join(timeout=1.0)
    return 0


def main() -> int:
    try:
        return run(parse_args())
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"Lỗi: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
