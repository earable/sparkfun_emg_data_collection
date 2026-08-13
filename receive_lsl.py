#!/usr/bin/env python3
"""Receive SparkFun MyoWare EMG samples from LSL and save them to binary."""

from __future__ import annotations

import argparse
import json
import signal
import struct
import sys
import time
from pathlib import Path

try:
    from pylsl import StreamInlet, resolve_byprop
except ImportError as exc:
    raise SystemExit(
        "Thiếu pylsl. Cài dependency bằng: python -m pip install -r requirements.txt"
    ) from exc


PACKET_HEADER = 0xAA55
PACKET_VERSION = 2
PACKET_BODY_STRUCT = struct.Struct("<HBIIHQ")
PACKET_STRUCT = struct.Struct("<HBIIHQH")
PACKET_SIZE = PACKET_STRUCT.size


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Nhận SparkFun MyoWare EMG từ LSL và lưu vào file binary."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("lsl_raw.bin"),
        help="File binary đầu ra (mặc định: lsl_raw.bin)",
    )
    parser.add_argument(
        "--stream-name",
        default="SparkFun_MyoWare_EMG",
        help="Tên LSL stream cần nhận",
    )
    parser.add_argument(
        "--source-id",
        help="Chỉ nhận stream có source_id này, ví dụ sparkfun-myo-001",
    )
    parser.add_argument(
        "--resolve-timeout",
        type=float,
        default=10.0,
        help="Số giây chờ tìm stream",
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


def resolve_stream(args: argparse.Namespace):
    property_name = "source_id" if args.source_id else "name"
    property_value = args.source_id or args.stream_name
    print(
        f"Đang tìm LSL stream có {property_name}={property_value!r} "
        f"(timeout {args.resolve_timeout:g}s)..."
    )
    streams = resolve_byprop(
        property_name,
        property_value,
        minimum=1,
        timeout=args.resolve_timeout,
    )
    if not streams:
        raise RuntimeError(
            "Không tìm thấy LSL stream. Hãy chạy collect_emg.py trước và "
            "kiểm tra firewall/mạng."
        )
    if len(streams) > 1:
        print(
            f"Cảnh báo: tìm thấy {len(streams)} stream; sử dụng stream đầu tiên.",
            file=sys.stderr,
        )
    return streams[0]


def write_metadata(
    path: Path,
    *,
    stream_info,
    started_at: str,
    sample_count: int,
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
        "lsl": {
            "name": stream_info.name(),
            "type": stream_info.type(),
            "source_id": stream_info.source_id(),
            "channel_count": stream_info.channel_count(),
            "nominal_srate": stream_info.nominal_srate(),
        },
        "session": {
            "started_at": started_at,
            "ended_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "samples": sample_count,
        },
    }
    path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> int:
    if args.resolve_timeout <= 0:
        raise ValueError("--resolve-timeout phải lớn hơn 0")
    if args.duration is not None and args.duration <= 0:
        raise ValueError("--duration phải lớn hơn 0")
    if args.flush_interval <= 0:
        raise ValueError("--flush-interval phải lớn hơn 0")

    stream_info = resolve_stream(args)
    if stream_info.channel_count() != 4:
        raise RuntimeError(
            f"Stream có {stream_info.channel_count()} kênh; "
            "tool yêu cầu packet v2 gồm 4 kênh."
        )

    inlet = StreamInlet(stream_info, max_buflen=60, recover=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    metadata_path = args.output.with_suffix(args.output.suffix + ".json")
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    started_monotonic = time.monotonic()
    last_flush = started_monotonic
    last_report = started_monotonic
    sample_count = 0
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    print(
        f"Đã kết nối: {stream_info.name()} "
        f"(source_id={stream_info.source_id() or 'N/A'})"
    )
    print(f"Data: {args.output}")
    print("Đang nhận; nhấn Ctrl+C để dừng.")

    try:
        with args.output.open("wb") as output:
            while not stop_requested:
                now = time.monotonic()
                if args.duration is not None and now - started_monotonic >= args.duration:
                    break

                samples, _timestamps = inlet.pull_chunk(
                    timeout=0.2, max_samples=1024
                )
                for sample in samples:
                    if len(sample) != 4:
                        continue
                    packet_id = int(sample[0])
                    device_timestamp_us = int(sample[1])
                    emg = int(sample[2])
                    utc_timestamp_s = int(sample[3])
                    if not 0 <= packet_id <= 0xFFFFFFFF:
                        raise RuntimeError(f"packet_id ngoài miền uint32: {packet_id}")
                    if not 0 <= device_timestamp_us <= 0xFFFFFFFF:
                        raise RuntimeError(
                            "device_timestamp_us ngoài miền uint32: "
                            f"{device_timestamp_us}"
                        )
                    if not 0 <= emg <= 0xFFFF:
                        raise RuntimeError(f"EMG ngoài miền uint16: {emg}")
                    if not 0 <= utc_timestamp_s <= 0xFFFFFFFFFFFFFFFF:
                        raise RuntimeError(
                            f"UTC timestamp ngoài miền uint64: {utc_timestamp_s}"
                        )

                    body = PACKET_BODY_STRUCT.pack(
                        PACKET_HEADER,
                        PACKET_VERSION,
                        packet_id,
                        device_timestamp_us,
                        emg,
                        utc_timestamp_s,
                    )
                    output.write(body + struct.pack("<H", crc16_ccitt(body)))
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
        write_metadata(
            metadata_path,
            stream_info=stream_info,
            started_at=started_at,
            sample_count=sample_count,
        )
        inlet.close_stream()

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
