#!/usr/bin/env python3
"""Realtime EMG plotter. Connects to the collector TCP hub as a LAN client."""

from __future__ import annotations

import argparse
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

    def snapshot(self) -> tuple[list[float], list[float], int, float]:
        with self.lock:
            elapsed = max(time.monotonic() - self.started, 1e-9)
            return list(self.times), list(self.values), self.packets, self.packets / elapsed


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
) -> None:
    parser = PacketParser()
    sock = socket.create_connection((host, port), timeout=5.0)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(0.2)
    try:
        while not stop.is_set():
            try:
                chunk = sock.recv(4096)
            except TimeoutError:
                continue
            if not chunk:
                break
            for _packet_id, timestamp_us, emg in parser.feed(chunk):
                buffer.add(timestamp_us, emg, vref)
    finally:
        sock.close()
        stop.set()


def run(args: argparse.Namespace) -> int:
    if args.window <= 0:
        raise ValueError("--window phải lớn hơn 0")
    if args.fps <= 0:
        raise ValueError("--fps phải lớn hơn 0")
    if args.vref is not None and args.vref <= 0:
        raise ValueError("--vref phải lớn hơn 0")

    buffer = LiveBuffer(args.window)
    stop = threading.Event()
    thread = threading.Thread(
        target=receiver_loop,
        args=(args.host, args.port, buffer, args.vref, stop),
        daemon=True,
    )
    thread.start()

    ylabel = "EMG (V)" if args.vref is not None else "EMG (ADC count)"
    fig, ax = plt.subplots(figsize=(10, 4))
    (line,) = ax.plot([], [], lw=0.8, color="#1f77b4")
    status = ax.text(0.01, 0.95, "", transform=ax.transAxes, va="top")
    ax.set_xlim(-args.window, 0)
    if args.vref is None:
        ax.set_ylim(0, 1024)
    else:
        ax.set_ylim(0, args.vref)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(f"SparkFun MyoWare  {args.host}:{args.port}")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    def update(_frame: int):
        times, values, packets, rate = buffer.snapshot()
        if times:
            t0 = times[-1]
            line.set_data([t - t0 for t in times], values)
            latest = values[-1]
            if args.vref is None:
                latest_text = f"{latest:.0f} ADC"
            else:
                latest_text = f"{latest:.3f} V"
        else:
            latest_text = "waiting"
        status.set_text(f"{rate:.0f} Hz   {packets} pkt   {latest_text}")
        if stop.is_set() and not thread.is_alive() and packets == 0:
            status.set_text("TCP disconnected / no data")
        return line, status

    _anim = FuncAnimation(
        fig,
        update,
        interval=max(int(1000 / args.fps), 10),
        blit=True,
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
