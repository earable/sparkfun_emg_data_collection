#!/usr/bin/env python3
"""Receive SparkFun MyoWare EMG packets from the LAN TCP hub."""

from __future__ import annotations

import argparse
import json
import signal
import socket
import struct
import sys
import time
from pathlib import Path


PACKET_HEADER = 0xAA55
PACKET_VERSION = 2
PACKET_STRUCT = struct.Struct("<HBIIHQH")
PACKET_SIZE = PACKET_STRUCT.size
HEADER_BYTES = struct.pack("<H", PACKET_HEADER)
DEFAULT_TCP_PORT = 8765


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
        self.bad_crc = 0
        self.bad_version = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes):
        self.buffer.extend(data)
        while True:
            header_index = self.buffer.find(HEADER_BYTES)
            if header_index < 0:
                discard = max(0, len(self.buffer) - 1)
                self.discarded_bytes += discard
                del self.buffer[:discard]
                return
            if header_index:
                self.discarded_bytes += header_index
                del self.buffer[:header_index]
            if len(self.buffer) < PACKET_SIZE:
                return

            raw = bytes(self.buffer[:PACKET_SIZE])
            header, version, _packet_id, _ts, _emg, _utc, expected_crc = (
                PACKET_STRUCT.unpack(raw)
            )
            if header != PACKET_HEADER:
                del self.buffer[0]
                self.discarded_bytes += 1
                continue
            if version != PACKET_VERSION:
                self.bad_version += 1
                del self.buffer[0]
                continue
            if crc16_ccitt(raw[:-2]) != expected_crc:
                self.bad_crc += 1
                del self.buffer[0]
                continue
            del self.buffer[:PACKET_SIZE]
            yield raw


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Nhận SparkFun MyoWare EMG từ TCP LAN và lưu file binary."
    )
    parser.add_argument("--host", required=True, help="IP máy chạy collect_emg.py")
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_TCP_PORT,
        help=f"Cổng TCP (mặc định: {DEFAULT_TCP_PORT})",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("tcp_raw.bin"),
        help="File binary đầu ra (mặc định: tcp_raw.bin)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="Tự dừng sau số giây; mặc định chạy đến Ctrl+C",
    )
    parser.add_argument(
        "--flush-interval",
        type=float,
        default=1.0,
        help="Chu kỳ flush file theo giây",
    )
    parser.add_argument(
        "--quiet", action="store_true", help="Không in trạng thái định kỳ"
    )
    return parser.parse_args()


def write_metadata(
    path: Path,
    *,
    host: str,
    port: int,
    started_at: str,
    sample_count: int,
    parser: PacketParser,
) -> None:
    metadata = {
        "format": {
            "endianness": "little",
            "version": PACKET_VERSION,
            "struct": "<HBIIHQH",
            "packet_size_bytes": PACKET_SIZE,
            "fields": [
                "header",
                "version",
                "packet_id",
                "device_timestamp_us",
                "emg",
                "utc_timestamp_s",
                "crc16",
            ],
        },
        "tcp": {"host": host, "port": port},
        "session": {
            "started_at": started_at,
            "ended_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "samples": sample_count,
            "bad_crc": parser.bad_crc,
            "bad_version": parser.bad_version,
            "discarded_bytes": parser.discarded_bytes,
        },
    }
    path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> int:
    if not (1 <= args.port <= 65535):
        raise ValueError("--port phải nằm trong 1..65535")
    if args.duration is not None and args.duration <= 0:
        raise ValueError("--duration phải lớn hơn 0")
    if args.flush_interval <= 0:
        raise ValueError("--flush-interval phải lớn hơn 0")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    metadata_path = args.output.with_suffix(args.output.suffix + ".json")
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    started_monotonic = time.monotonic()
    last_flush = started_monotonic
    last_report = started_monotonic
    sample_count = 0
    parser = PacketParser()
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    print(f"TCP:  {args.host}:{args.port}")
    print(f"Data: {args.output}")
    print("Đang nhận; nhấn Ctrl+C để dừng. Mất kết nối sẽ tự reconnect.")

    try:
        with args.output.open("wb") as output:
            while not stop_requested:
                now = time.monotonic()
                if args.duration is not None and now - started_monotonic >= args.duration:
                    break
                try:
                    sock = socket.create_connection((args.host, args.port), timeout=3.0)
                except OSError:
                    if not args.quiet:
                        print("\rTCP reconnecting...", end="", flush=True)
                    time.sleep(1.0)
                    continue
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(0.2)
                parser.buffer.clear()
                if not args.quiet:
                    print(f"\nTCP connected: {args.host}:{args.port}", flush=True)
                try:
                    while not stop_requested:
                        now = time.monotonic()
                        if (
                            args.duration is not None
                            and now - started_monotonic >= args.duration
                        ):
                            stop_requested = True
                            break
                        try:
                            chunk = sock.recv(4096)
                        except TimeoutError:
                            continue
                        except OSError:
                            break
                        if not chunk:
                            break
                        for packet in parser.feed(chunk):
                            output.write(packet)
                            sample_count += 1

                        now = time.monotonic()
                        if now - last_flush >= args.flush_interval:
                            output.flush()
                            last_flush = now
                        if not args.quiet and now - last_report >= 1.0:
                            elapsed = max(now - started_monotonic, 1e-9)
                            print(
                                f"\rSamples: {sample_count} | "
                                f"Rate: {sample_count / elapsed:.1f} Hz",
                                end="",
                                flush=True,
                            )
                            last_report = now
                finally:
                    try:
                        sock.close()
                    except OSError:
                        pass
                if not stop_requested:
                    if not args.quiet:
                        print("\nTCP disconnected; reconnecting...", flush=True)
                    time.sleep(0.5)
    finally:
        write_metadata(
            metadata_path,
            host=args.host,
            port=args.port,
            started_at=started_at,
            sample_count=sample_count,
            parser=parser,
        )

    if not args.quiet:
        print()
    print(f"Đã lưu {sample_count} mẫu ({sample_count * PACKET_SIZE} byte).")
    print(f"Metadata: {metadata_path}")
    return 0


def main() -> int:
    try:
        return run(parse_args())
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"Lỗi: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
