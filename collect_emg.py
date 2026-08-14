#!/usr/bin/env python3
"""Collect SparkFun MyoWare EMG packets from USB, LSL, and a LAN TCP hub."""

from __future__ import annotations

import argparse
import json
import queue
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterator

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit(
        "Thiếu pyserial. Cài dependency bằng: python -m pip install -r requirements.txt"
    ) from exc

DEFAULT_TCP_HOST = "0.0.0.0"
DEFAULT_TCP_PORT = 8765


BAUDRATE = 230_400
PACKET_HEADER = 0xAA55
WIRE_PACKET_VERSION = 1
WIRE_PACKET_STRUCT = struct.Struct("<HBIIHH")
WIRE_PACKET_SIZE = WIRE_PACKET_STRUCT.size
STORED_PACKET_VERSION = 2
STORED_PACKET_BODY_STRUCT = struct.Struct("<HBIIHQ")
STORED_PACKET_STRUCT = struct.Struct("<HBIIHQH")
STORED_PACKET_SIZE = STORED_PACKET_STRUCT.size
HEADER_BYTES = struct.pack("<H", PACKET_HEADER)
SERIAL_READ_CHUNK = 8192
WRITE_BUFFER_PACKETS = 128
LSL_CHUNK_PACKETS = 32
SERIAL_RECONNECT_S = 0.5


def _crc16_table() -> tuple[int, ...]:
    table = []
    for index in range(256):
        crc = index << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return tuple(table)


CRC16_TABLE = _crc16_table()


def crc16_ccitt(data: bytes) -> int:
    """Match the CRC-16/CCITT-FALSE implementation used by the firmware."""
    crc = 0xFFFF
    for byte in data:
        crc = ((crc << 8) ^ CRC16_TABLE[((crc >> 8) ^ byte) & 0xFF]) & 0xFFFF
    return crc


@dataclass(frozen=True)
class EMGPacket:
    raw: bytes
    packet_id: int
    device_timestamp_us: int
    emg: int


class PacketParser:
    """Incrementally parse packets and recover after corrupt or missing bytes."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad_crc = 0
        self.bad_version = 0
        self.discarded_bytes = 0
        self._in_sync = False

    def reset(self) -> None:
        self.buffer.clear()
        self._in_sync = False

    def _reject_candidate(self) -> None:
        if self._in_sync and len(self.buffer) >= WIRE_PACKET_SIZE:
            del self.buffer[:WIRE_PACKET_SIZE]
            self.discarded_bytes += WIRE_PACKET_SIZE
        else:
            del self.buffer[0]
            self.discarded_bytes += 1
        self._in_sync = False

    def feed(self, data: bytes) -> Iterator[EMGPacket]:
        self.buffer.extend(data)

        while True:
            header_index = self.buffer.find(HEADER_BYTES)
            if header_index < 0:
                # Keep one byte in case it is the start of a split header.
                discard = max(0, len(self.buffer) - 1)
                self.discarded_bytes += discard
                del self.buffer[:discard]
                self._in_sync = False
                return

            if header_index:
                self.discarded_bytes += header_index
                del self.buffer[:header_index]
                self._in_sync = False

            if len(self.buffer) < WIRE_PACKET_SIZE:
                return

            raw = bytes(self.buffer[:WIRE_PACKET_SIZE])
            header, version, packet_id, timestamp_us, emg, expected_crc = (
                WIRE_PACKET_STRUCT.unpack(raw)
            )
            actual_crc = crc16_ccitt(raw[:-2])

            if header != PACKET_HEADER:
                # Defensive only: find() above already guarantees this.
                del self.buffer[0]
                self.discarded_bytes += 1
                self._in_sync = False
                continue
            if version != WIRE_PACKET_VERSION:
                self.bad_version += 1
                self._reject_candidate()
                continue
            if actual_crc != expected_crc:
                self.bad_crc += 1
                self._reject_candidate()
                continue

            del self.buffer[:WIRE_PACKET_SIZE]
            self._in_sync = True
            yield EMGPacket(raw, packet_id, timestamp_us, emg)


def open_serial(port: str, baudrate: int) -> serial.Serial:
    kwargs = {
        "port": port,
        "baudrate": baudrate,
        "bytesize": serial.EIGHTBITS,
        "parity": serial.PARITY_NONE,
        "stopbits": serial.STOPBITS_ONE,
        "timeout": 0.05,
        "write_timeout": 0,
        "xonxoff": False,
        "rtscts": False,
        "dsrdtr": False,
    }
    try:
        ser = serial.Serial(**kwargs, exclusive=True)
    except TypeError:
        ser = serial.Serial(**kwargs)
    time.sleep(0.4)
    ser.reset_input_buffer()
    return ser


def find_serial_port(preferred: str) -> str | None:
    ports = available_ports()
    if preferred in ports:
        return preferred
    usb_ports = [
        port
        for port in ports
        if "usb" in port.lower() or "acm" in port.lower()
    ]
    if len(usb_ports) == 1:
        return usb_ports[0]
    return None


def serial_reader_loop(
    preferred_port: str,
    baudrate: int,
    parser: PacketParser,
    packet_queue: queue.SimpleQueue[EMGPacket],
    stop: threading.Event,
    usb_state: dict[str, object],
) -> None:
    """Read USB continuously; reopen the port if the cable is unplugged."""
    ser: serial.Serial | None = None
    last_wait_log = 0.0
    try:
        while not stop.is_set():
            if ser is None:
                target = find_serial_port(preferred_port)
                if target is None:
                    usb_state["connected"] = False
                    now = time.monotonic()
                    if now - last_wait_log >= 2.0:
                        print(
                            f"\nUSB disconnected; waiting for {preferred_port}",
                            flush=True,
                        )
                        last_wait_log = now
                    stop.wait(SERIAL_RECONNECT_S)
                    continue
                try:
                    ser = open_serial(target, baudrate)
                except (serial.SerialException, OSError) as exc:
                    usb_state["connected"] = False
                    now = time.monotonic()
                    if now - last_wait_log >= 2.0:
                        print(f"\nUSB waiting ({target}): {exc}", flush=True)
                        last_wait_log = now
                    stop.wait(SERIAL_RECONNECT_S)
                    continue
                parser.reset()
                usb_state["port"] = target
                usb_state["generation"] = int(usb_state["generation"]) + 1
                usb_state["connected"] = True
                print(f"\nUSB connected: {target}", flush=True)
            try:
                waiting = ser.in_waiting
                chunk = ser.read(waiting if waiting else SERIAL_READ_CHUNK)
            except (serial.SerialException, OSError):
                print("\nUSB disconnected; waiting to reconnect...", flush=True)
                usb_state["connected"] = False
                parser.reset()
                try:
                    ser.close()
                except OSError:
                    pass
                ser = None
                last_wait_log = time.monotonic()
                continue
            if not chunk:
                continue
            for packet in parser.feed(chunk):
                packet_queue.put(packet)
    finally:
        if ser is not None:
            try:
                ser.close()
            except OSError:
                pass


class TcpHub:
    """Accept LAN clients and push stored v2 packets without blocking USB I/O."""

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.server: socket.socket | None = None
        self._clients: dict[socket.socket, tuple[str, int]] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def client_count(self) -> int:
        with self._lock:
            return len(self._clients)

    def start(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(16)
        server.settimeout(0.5)
        self.server = server
        self._thread = threading.Thread(
            target=self._accept_loop, name="tcp-hub", daemon=True
        )
        self._thread.start()

    def _accept_loop(self) -> None:
        assert self.server is not None
        while not self._stop.is_set():
            try:
                conn, addr = self.server.accept()
            except (TimeoutError, socket.timeout):
                continue
            except OSError:
                break
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.setblocking(False)
            with self._lock:
                self._clients[conn] = addr
            print(f"\nTCP connected: {addr[0]}:{addr[1]}", flush=True)

    def broadcast(self, payload: bytes) -> None:
        with self._lock:
            clients = list(self._clients.items())
        dropped: list[socket.socket] = []
        for conn, addr in clients:
            try:
                sent = 0
                while sent < len(payload):
                    n = conn.send(payload[sent:])
                    if n == 0:
                        raise OSError("TCP send returned 0")
                    sent += n
            except (BlockingIOError, InterruptedError):
                # The iOS client may not have started reading yet. Keep it.
                continue
            except OSError:
                dropped.append(conn)
                print(f"\nTCP dropped: {addr[0]}:{addr[1]}", flush=True)
        if dropped:
            self._drop(dropped)

    def _drop(self, sockets: list[socket.socket]) -> None:
        with self._lock:
            for conn in sockets:
                self._clients.pop(conn, None)
                try:
                    conn.close()
                except OSError:
                    pass

    def close(self) -> None:
        self._stop.set()
        if self.server is not None:
            try:
                self.server.close()
            except OSError:
                pass
            self.server = None
        with self._lock:
            clients = list(self._clients)
            self._clients.clear()
        for conn in clients:
            try:
                conn.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None


class BonjourAdvertiser:
    """Advertise the TCP hub so iOS can discover it via Bonjour."""

    service_type = "_sparkfun-emg._tcp"
    service_name = "SparkFun MyoWare EMG"

    def __init__(self, port: int) -> None:
        self.port = port
        self._process: subprocess.Popen[bytes] | None = None
        self._zeroconf = None
        self._info = None

    def start(self) -> str:
        lan_ip = local_ipv4()
        if sys.platform == "darwin" and self._start_dns_sd(lan_ip):
            return f"{self.service_name}.{self.service_type} @ {lan_ip}:{self.port}"
        if self._start_zeroconf(lan_ip):
            return f"{self.service_name}.{self.service_type} @ {lan_ip}:{self.port}"
        return (
            f"unavailable; iOS can still connect to {lan_ip}:{self.port} "
            "if Local Network permission is allowed"
        )

    def _start_dns_sd(self, lan_ip: str) -> bool:
        try:
            self._process = subprocess.Popen(
                [
                    "dns-sd",
                    "-P",
                    self.service_name,
                    self.service_type,
                    "local.",
                    str(self.port),
                    "sparkfun-emg.local",
                    lan_ip,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return self._process.poll() is None
        except OSError:
            self._process = None
            return False

    def _start_zeroconf(self, lan_ip: str) -> bool:
        try:
            from zeroconf import ServiceInfo, Zeroconf
        except ImportError:
            return False
        try:
            info = ServiceInfo(
                f"{self.service_type}.local.",
                f"{self.service_name}.{self.service_type}.local.",
                addresses=[socket.inet_aton(lan_ip)],
                port=self.port,
                properties={"path": "/", "source": "sparkfun-myo"},
            )
            zeroconf = Zeroconf()
            zeroconf.register_service(info)
            self._info = info
            self._zeroconf = zeroconf
            return True
        except OSError:
            return False

    def close(self) -> None:
        if self._process is not None:
            self._process.terminate()
            try:
                self._process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None
        if self._zeroconf is not None:
            try:
                if self._info is not None:
                    self._zeroconf.unregister_service(self._info)
                self._zeroconf.close()
            except OSError:
                pass
            self._zeroconf = None
            self._info = None


def local_ipv4() -> str:
    candidates: list[str] = []
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        ip = probe.getsockname()[0]
        if _is_lan_ipv4(ip):
            candidates.append(ip)
    except OSError:
        pass
    finally:
        probe.close()

    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if _is_lan_ipv4(ip) and ip not in candidates:
                candidates.append(ip)
    except OSError:
        pass

    return candidates[0] if candidates else "127.0.0.1"


def _is_lan_ipv4(ip: str) -> bool:
    return not (
        ip.startswith("127.")
        or ip.startswith("169.254.")
        or ip.startswith("0.")
    )


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


def choose_port(requested_port: str | None) -> str:
    if requested_port:
        return requested_port

    ports = available_ports()
    usb_ports = [
        port
        for port in ports
        if "usb" in port.lower() or "acm" in port.lower()
    ]
    candidates = usb_ports or ports
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise RuntimeError(
            "Không tìm thấy cổng serial. Kiểm tra cáp USB hoặc truyền --port."
        )
    raise RuntimeError(
        "Có nhiều cổng serial; hãy chọn bằng --port:\n  " + "\n  ".join(candidates)
    )


def create_lsl_outlet(source_id: str):
    try:
        from pylsl import StreamInfo, StreamOutlet
    except ImportError as exc:
        raise RuntimeError(
            "Thiếu pylsl. Cài dependency bằng: python -m pip install -r requirements.txt"
        ) from exc
    info = StreamInfo(
        name="SparkFun_MyoWare_EMG",
        type="EMG",
        channel_count=4,
        nominal_srate=0,
        channel_format="int64",
        source_id=source_id,
    )
    channels = info.desc().append_child("channels")
    channel_definitions = [
        ("packet_id", "count", "PacketID"),
        ("device_timestamp_us", "microseconds", "DeviceTimestamp"),
        ("emg", "ADC_count", "EMG"),
        ("utc_timestamp_s", "seconds_since_unix_epoch", "UTC"),
    ]
    for label, unit, channel_type in channel_definitions:
        channel = channels.append_child("channel")
        channel.append_child_value("label", label)
        channel.append_child_value("unit", unit)
        channel.append_child_value("type", channel_type)
    acquisition = info.desc().append_child("acquisition")
    acquisition.append_child_value("manufacturer", "SparkFun")
    acquisition.append_child_value("model", "MyoWare")
    acquisition.append_child_value("packet_version", str(STORED_PACKET_VERSION))
    return StreamOutlet(info)


def utc_timestamp_text(unix_s: float) -> str:
    moment = datetime.fromtimestamp(unix_s).astimezone()
    return moment.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + moment.strftime("%z")


def make_lost_interval(
    *,
    start_unix: float,
    end_unix: float,
    reason: str,
    lost_packets: int | None,
    start_packet_id: int | None,
    end_packet_id: int | None,
) -> dict[str, object]:
    return {
        "start_timestamp": utc_timestamp_text(start_unix),
        "end_timestamp": utc_timestamp_text(end_unix),
        "lost_packets": lost_packets,
        "reason": reason,
        "start_packet_id": start_packet_id,
        "end_packet_id": end_packet_id,
    }


def write_metadata(
    path: Path,
    *,
    port: str,
    baudrate: int,
    source_id: str | None,
    started_at: str,
    packets: int,
    lost_packets: int,
    lost_intervals: list[dict[str, object]],
    parser: PacketParser,
    tcp_endpoint: str | None,
) -> None:
    metadata = {
        "format": {
            "endianness": "little",
            "version": STORED_PACKET_VERSION,
            "struct": "<HBIIHQH",
            "packet_size_bytes": STORED_PACKET_SIZE,
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
        "wire_format": {
            "version": WIRE_PACKET_VERSION,
            "struct": "<HBIIHH",
            "packet_size_bytes": WIRE_PACKET_SIZE,
        },
        "serial": {"port": port, "baudrate": baudrate},
        "lsl": None
        if source_id is None
        else {
            "name": "SparkFun_MyoWare_EMG",
            "type": "EMG",
            "source_id": source_id,
        },
        "tcp": None
        if tcp_endpoint is None
        else {
            "endpoint": tcp_endpoint,
            "packet_struct": "<HBIIHQH",
            "packet_size_bytes": STORED_PACKET_SIZE,
        },
        "session": {
            "started_at": started_at,
            "ended_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "valid_packets": packets,
            "estimated_lost_packets": lost_packets,
            "lost_intervals": lost_intervals,
            "bad_crc": parser.bad_crc,
            "bad_version": parser.bad_version,
            "discarded_bytes": parser.discarded_bytes,
        },
    }
    path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Thu thập MyoWare EMG từ USB, lưu raw.bin, stream LSL và/hoặc TCP LAN."
        )
    )
    parser.add_argument("--port", help="Cổng serial, ví dụ /dev/cu.usbserial-1130")
    parser.add_argument("--baud", type=int, default=BAUDRATE, help="Baudrate serial")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("raw.bin"),
        help="File binary đầu ra (mặc định: raw.bin)",
    )
    parser.add_argument(
        "--source-id", default="sparkfun-myo-001", help="LSL source_id ổn định"
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="Tự dừng sau số giây này; mặc định chạy đến Ctrl+C",
    )
    parser.add_argument(
        "--flush-interval",
        type=float,
        default=1.0,
        help="Chu kỳ flush file theo giây",
    )
    parser.add_argument(
        "--quiet", action="store_true", help="Không in dữ liệu trạng thái định kỳ"
    )
    parser.add_argument(
        "--list-ports", action="store_true", help="Liệt kê cổng serial rồi thoát"
    )
    parser.add_argument(
        "--no-lsl",
        action="store_true",
        help="Không tạo LSL outlet; chỉ ghi file và/hoặc TCP",
    )
    parser.add_argument(
        "--tcp-host",
        default=DEFAULT_TCP_HOST,
        help="Địa chỉ bind TCP (mặc định: 0.0.0.0, toàn bộ LAN)",
    )
    parser.add_argument(
        "--tcp-port",
        type=int,
        default=DEFAULT_TCP_PORT,
        help=f"Cổng TCP (mặc định: {DEFAULT_TCP_PORT})",
    )
    parser.add_argument(
        "--no-tcp",
        action="store_true",
        help="Không mở TCP server",
    )
    return parser.parse_args()


def run(args: argparse.Namespace) -> int:
    if args.list_ports:
        ports = available_ports()
        print("\n".join(ports) if ports else "Không tìm thấy cổng serial.")
        return 0
    if args.duration is not None and args.duration <= 0:
        raise ValueError("--duration phải lớn hơn 0")
    if args.flush_interval <= 0:
        raise ValueError("--flush-interval phải lớn hơn 0")
    if not args.no_tcp and not (1 <= args.tcp_port <= 65535):
        raise ValueError("--tcp-port phải nằm trong 1..65535")

    port = choose_port(args.port)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    metadata_path = args.output.with_suffix(args.output.suffix + ".json")
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    parser = PacketParser()
    lsl_clock = None
    outlet = None
    if not args.no_lsl:
        from pylsl import local_clock as lsl_clock

        outlet = create_lsl_outlet(args.source_id)
    tcp_hub = None if args.no_tcp else TcpHub(args.tcp_host, args.tcp_port)
    bonjour = None
    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    packets = 0
    lost_packets = 0
    lost_intervals: list[dict[str, object]] = []
    previous_packet_id: int | None = None
    last_packet_unix: float | None = None
    last_packet_id: int | None = None
    pending_usb_start_unix: float | None = None
    pending_usb_start_packet_id: int | None = None
    started_monotonic = time.monotonic()
    last_report = started_monotonic
    last_flush = started_monotonic
    packet_queue: queue.SimpleQueue[EMGPacket] = queue.SimpleQueue()
    write_buf = bytearray()
    lsl_samples: list[list[int]] = []
    lsl_stamps: list[float] = []
    usb_state: dict[str, object] = {
        "connected": False,
        "port": port,
        "generation": 0,
    }
    last_usb_generation = 0

    print(f"Serial: {port} @ {args.baud}")
    print(f"Data:   {args.output}")
    if outlet is None:
        print("LSL:    tắt")
    else:
        print(f"LSL:    SparkFun_MyoWare_EMG ({args.source_id})")
    if tcp_hub is None:
        print("TCP:    tắt")
    else:
        lan_ip = local_ipv4()
        print(f"TCP:    {args.tcp_host}:{args.tcp_port} (LAN {lan_ip}:{args.tcp_port})")
        tcp_hub.start()
        bonjour = BonjourAdvertiser(args.tcp_port)
        print(f"Bonjour:{bonjour.start()}")
    print("Đang thu thập; nhấn Ctrl+C để dừng.")

    def flush_outputs(output, force: bool = False) -> None:
        if write_buf and (
            force or len(write_buf) >= STORED_PACKET_SIZE * WRITE_BUFFER_PACKETS
        ):
            output.write(write_buf)
            write_buf.clear()
        if outlet is not None and lsl_samples and (force or len(lsl_samples) >= LSL_CHUNK_PACKETS):
            outlet.push_chunk(lsl_samples, lsl_stamps)
            lsl_samples.clear()
            lsl_stamps.clear()

    def handle_packet(packet: EMGPacket) -> None:
        nonlocal packets, lost_packets, previous_packet_id
        nonlocal last_packet_unix, last_packet_id
        nonlocal pending_usb_start_unix, pending_usb_start_packet_id
        utc_now = time.time()
        utc_timestamp_s = int(utc_now)
        stored_body = STORED_PACKET_BODY_STRUCT.pack(
            PACKET_HEADER,
            STORED_PACKET_VERSION,
            packet.packet_id,
            packet.device_timestamp_us,
            packet.emg,
            utc_timestamp_s,
        )
        stored_packet = stored_body + struct.pack("<H", crc16_ccitt(stored_body))
        write_buf.extend(stored_packet)
        if tcp_hub is not None:
            tcp_hub.broadcast(stored_packet)
        if outlet is not None and lsl_clock is not None:
            lsl_samples.append(
                [
                    packet.packet_id,
                    packet.device_timestamp_us,
                    packet.emg,
                    utc_timestamp_s,
                ]
            )
            lsl_stamps.append(lsl_clock())
        packets += 1
        if pending_usb_start_unix is not None:
            lost_intervals.append(
                make_lost_interval(
                    start_unix=pending_usb_start_unix,
                    end_unix=utc_now,
                    reason="usb_disconnect",
                    lost_packets=None,
                    start_packet_id=pending_usb_start_packet_id,
                    end_packet_id=packet.packet_id,
                )
            )
            pending_usb_start_unix = None
            pending_usb_start_packet_id = None
        if previous_packet_id is not None:
            gap = (packet.packet_id - previous_packet_id) & 0xFFFFFFFF
            if 1 < gap < 0x80000000:
                lost = gap - 1
                lost_packets += lost
                lost_intervals.append(
                    make_lost_interval(
                        start_unix=(
                            last_packet_unix if last_packet_unix is not None else utc_now
                        ),
                        end_unix=utc_now,
                        reason="packet_gap",
                        lost_packets=lost,
                        start_packet_id=previous_packet_id,
                        end_packet_id=packet.packet_id,
                    )
                )
        previous_packet_id = packet.packet_id
        last_packet_unix = utc_now
        last_packet_id = packet.packet_id

    try:
        with args.output.open("wb") as output:
            reader = threading.Thread(
                target=serial_reader_loop,
                args=(
                    port,
                    args.baud,
                    parser,
                    packet_queue,
                    stop_event,
                    usb_state,
                ),
                name="serial-reader",
                daemon=True,
            )
            reader.start()
            while not stop_event.is_set():
                now = time.monotonic()
                if args.duration is not None and now - started_monotonic >= args.duration:
                    stop_event.set()
                    break

                usb_generation = int(usb_state["generation"])
                if usb_generation != last_usb_generation:
                    if last_usb_generation != 0 and pending_usb_start_unix is None:
                        pending_usb_start_unix = (
                            last_packet_unix
                            if last_packet_unix is not None
                            else time.time()
                        )
                        pending_usb_start_packet_id = last_packet_id
                    previous_packet_id = None
                    last_usb_generation = usb_generation
                elif (
                    last_usb_generation != 0
                    and not usb_state["connected"]
                    and pending_usb_start_unix is None
                ):
                    pending_usb_start_unix = (
                        last_packet_unix if last_packet_unix is not None else time.time()
                    )
                    pending_usb_start_packet_id = last_packet_id

                drained = 0
                while drained < 256:
                    try:
                        packet = (
                            packet_queue.get(timeout=0.05)
                            if drained == 0
                            else packet_queue.get_nowait()
                        )
                    except queue.Empty:
                        break
                    handle_packet(packet)
                    drained += 1

                flush_outputs(output)
                now = time.monotonic()
                if now - last_flush >= args.flush_interval:
                    flush_outputs(output, force=True)
                    output.flush()
                    last_flush = now
                if not args.quiet and now - last_report >= 1.0:
                    elapsed = max(now - started_monotonic, 1e-9)
                    tcp_clients = 0 if tcp_hub is None else tcp_hub.client_count
                    seen = packets + lost_packets
                    loss_pct = (100.0 * lost_packets / seen) if seen else 0.0
                    usb_text = "ok" if usb_state["connected"] else "disconnected"
                    print(
                        f"\rPackets: {packets} | Rate: {packets / elapsed:.1f} Hz "
                        f"| Lost: {lost_packets} ({loss_pct:.3f}%) "
                        f"| CRC errors: {parser.bad_crc} "
                        f"| USB: {usb_text} "
                        f"| TCP clients: {tcp_clients}",
                        end="",
                        flush=True,
                    )
                    last_report = now
            stop_event.set()
            while True:
                try:
                    handle_packet(packet_queue.get_nowait())
                except queue.Empty:
                    break
            flush_outputs(output, force=True)
            output.flush()
            reader.join(timeout=1.0)
    finally:
        if pending_usb_start_unix is not None:
            lost_intervals.append(
                make_lost_interval(
                    start_unix=pending_usb_start_unix,
                    end_unix=time.time(),
                    reason="usb_disconnect",
                    lost_packets=None,
                    start_packet_id=pending_usb_start_packet_id,
                    end_packet_id=None,
                )
            )
            pending_usb_start_unix = None
        if bonjour is not None:
            bonjour.close()
        if tcp_hub is not None:
            tcp_hub.close()
        write_metadata(
            metadata_path,
            port=port,
            baudrate=args.baud,
            source_id=None if args.no_lsl else args.source_id,
            started_at=started_at,
            packets=packets,
            lost_packets=lost_packets,
            lost_intervals=lost_intervals,
            parser=parser,
            tcp_endpoint=None
            if args.no_tcp
            else f"{args.tcp_host}:{args.tcp_port}",
        )

    if not args.quiet:
        print()
    print(
        f"Đã lưu {packets} packet ({packets * STORED_PACKET_SIZE} byte); "
        f"ước tính mất {lost_packets} packet "
        f"({len(lost_intervals)} khoảng)."
    )
    print(f"Metadata: {metadata_path}")
    return 0


def main() -> int:
    try:
        return run(parse_args())
    except (RuntimeError, ValueError, serial.SerialException, OSError) as exc:
        print(f"Lỗi: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
