#!/usr/bin/env python3
"""Print the first and last N seconds of a SparkFun raw.bin recording."""

from __future__ import annotations

import argparse
import struct
import sys
from collections import deque
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import TextIO


PACKET_HEADER = 0xAA55
V1_PACKET_STRUCT = struct.Struct("<HBIIHH")
V2_PACKET_STRUCT = struct.Struct("<HBIIHQH")
TIMESTAMP_WRAP = 1 << 32


@dataclass(frozen=True)
class Row:
    relative_time_s: float
    packet_id: int
    device_timestamp_us: int
    emg: int
    utc_timestamp_s: int | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="In dữ liệu text trong N giây đầu và N giây cuối của raw.bin."
    )
    parser.add_argument("input", type=Path, help="File raw.bin cần đọc")
    parser.add_argument(
        "--seconds",
        type=float,
        default=10.0,
        help="Số giây ở mỗi đầu file (mặc định: 10)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Ghi CSV vào file này thay vì in ra terminal",
    )
    return parser.parse_args()


def detect_packet_struct(source) -> tuple[int, struct.Struct]:
    prefix = source.read(3)
    source.seek(0)
    if len(prefix) != 3:
        raise ValueError("File không chứa packet hoàn chỉnh.")
    header, version = struct.unpack("<HB", prefix)
    if header != PACKET_HEADER:
        raise ValueError(f"Header đầu file không hợp lệ: 0x{header:04X}")
    if version == 1:
        return version, V1_PACKET_STRUCT
    if version == 2:
        return version, V2_PACKET_STRUCT
    raise ValueError(f"Không hỗ trợ packet version {version}.")


def select_rows(
    path: Path, seconds: float
) -> tuple[list[Row], deque[Row], int, int]:
    first_rows: list[Row] = []
    last_rows: deque[Row] = deque()
    first_unwrapped_us: int | None = None
    previous_timestamp_us: int | None = None
    wrap_count = 0
    packet_count = 0

    with path.open("rb") as source:
        version, packet_struct = detect_packet_struct(source)
        packet_size = packet_struct.size
        while raw := source.read(packet_size):
            if len(raw) != packet_size:
                raise ValueError(
                    f"File có {len(raw)} byte dư ở cuối; packet phải dài "
                    f"{packet_size} byte."
                )

            values = packet_struct.unpack(raw)
            header, packet_version, packet_id, timestamp_us, emg = values[:5]
            utc_timestamp_s = values[5] if packet_version == 2 else None
            if header != PACKET_HEADER:
                raise ValueError(
                    f"Header không hợp lệ tại packet {packet_count}: 0x{header:04X}"
                )
            if packet_version != version:
                raise ValueError(
                    f"Packet {packet_count} có version {packet_version}, "
                    f"mong đợi version {version}."
                )

            if (
                previous_timestamp_us is not None
                and previous_timestamp_us - timestamp_us > TIMESTAMP_WRAP // 2
            ):
                wrap_count += 1
            previous_timestamp_us = timestamp_us

            unwrapped_us = timestamp_us + wrap_count * TIMESTAMP_WRAP
            if first_unwrapped_us is None:
                first_unwrapped_us = unwrapped_us
            relative_time_s = (unwrapped_us - first_unwrapped_us) / 1_000_000
            row = Row(
                relative_time_s,
                packet_id,
                timestamp_us,
                emg,
                utc_timestamp_s,
            )

            if relative_time_s <= seconds:
                first_rows.append(row)

            last_rows.append(row)
            while (
                last_rows
                and relative_time_s - last_rows[0].relative_time_s > seconds
            ):
                last_rows.popleft()

            packet_count += 1

    return first_rows, last_rows, packet_count, version


def format_utc(timestamp_s: int | None) -> str:
    if timestamp_s is None:
        return ""
    return datetime.fromtimestamp(timestamp_s, UTC).strftime("%Y-%m-%dT%H:%M:%SZ")


def print_section(title: str, rows: list[Row] | deque[Row], output: TextIO) -> None:
    print(f"# {title}", file=output)
    print(
        "relative_time_s,packet_id,device_timestamp_us,emg,"
        "utc_timestamp_s,utc_iso8601",
        file=output,
    )
    for row in rows:
        utc_s = "" if row.utc_timestamp_s is None else str(row.utc_timestamp_s)
        print(
            f"{row.relative_time_s:.6f},{row.packet_id},"
            f"{row.device_timestamp_us},{row.emg},{utc_s},"
            f"{format_utc(row.utc_timestamp_s)}",
            file=output,
        )


def run(args: argparse.Namespace) -> int:
    if args.seconds <= 0:
        raise ValueError("--seconds phải lớn hơn 0")
    if not args.input.is_file():
        raise ValueError(f"Không tìm thấy file: {args.input}")

    first_rows, last_rows, packet_count, version = select_rows(
        args.input, args.seconds
    )

    output: TextIO = sys.stdout
    output_file: TextIO | None = None
    try:
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            output_file = args.output.open("w", encoding="utf-8", newline="")
            output = output_file

        print_section(f"{args.seconds:g} seconds đầu", first_rows, output)
        print(file=output)
        print_section(f"{args.seconds:g} seconds cuối", last_rows, output)
    finally:
        if output_file:
            output_file.close()

    summary = (
        f"Đã đọc {packet_count} packet v{version}; chọn {len(first_rows)} "
        f"packet đầu và {len(last_rows)} packet cuối."
    )
    print(summary, file=sys.stderr if not args.output else sys.stdout)
    if args.output:
        print(f"Đã ghi: {args.output}")
    return 0


def main() -> int:
    try:
        return run(parse_args())
    except (OSError, ValueError) as exc:
        print(f"Lỗi: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
