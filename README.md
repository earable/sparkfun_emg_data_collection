# SparkFun MyoWare EMG Collector

Thu thập EMG từ MyoWare trên macOS: ghi `raw.bin`, phát LSL và/hoặc TCP LAN,
vẽ realtime.

```
MyoWare -> RedBoard -> USB -> collect_emg.py
                           |-> raw.bin + raw.bin.json
                           |-> LSL Outlet          (tuỳ chọn)
                           |-> TCP 0.0.0.0:8765    (mặc định bật)
```

Chi tiết firmware, packet và các cờ CLI nằm trong
[SparkFun_MyoWare_EMG_Tutorial.md](SparkFun_MyoWare_EMG_Tutorial.md).

## Phần cứng

- SparkFun RedBoard Plus
- MyoWare Muscle Sensor + Shield
- macOS, USB-C

Nạp firmware packet v1 (15 byte, `230400` baud) như trong tutorial. Đóng
Arduino Serial Monitor trước khi chạy collector.

## Cài đặt

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## Quick start

```bash
python collect_emg.py --list-ports

python collect_emg.py \
  --port /dev/cu.usbserial-1130 \
  --output data/session01/raw.bin
```

Terminal khác, cùng máy:

```bash
python plot_emg.py
```

Chỉ TCP, không LSL:

```bash
python collect_emg.py --port /dev/cu.usbserial-1130 --no-lsl
```

Máy khác trên LAN:

```bash
python receive_tcp.py --host <IP_MAC> --output data/session01/tcp_raw.bin
python plot_emg.py --host <IP_MAC>
```

Nhận LSL:

```bash
python receive_lsl.py --output data/session01/lsl_raw.bin
```

In 10 giây đầu/cuối:

```bash
python print_raw.py data/session01/raw.bin --seconds 10
```

## Tool

| File             | Vai trò                                 |
| ---------------- | --------------------------------------- |
| `collect_emg.py` | USB → file + LSL + TCP server           |
| `receive_tcp.py` | Client TCP LAN, ghi `tcp_raw.bin`       |
| `receive_lsl.py` | Client LSL, ghi `lsl_raw.bin`           |
| `plot_emg.py`    | Biểu đồ realtime qua TCP                |
| `print_raw.py`   | Xuất text/CSV từ file binary            |

TCP mặc định `0.0.0.0:8765`. Plotter là process riêng, refresh ~30 FPS, cửa sổ
5 giây — không vẽ trong collector để tránh drop USB.

## Định dạng data

USB firmware gửi packet **v1** 15 byte (`<HBIIHH`).

Collector gắn UTC Unix **theo giây** rồi lưu/phát packet **v2** 23 byte
(`<HBIIHQH`):

| Field             | Type   |
| ----------------- | ------ |
| Header `0xAA55`   | uint16 |
| Version `2`       | uint8  |
| PacketID          | uint32 |
| Timestamp (us)    | uint32 |
| EMG               | uint16 |
| UTC timestamp (s) | uint64 |
| CRC16             | uint16 |

`raw.bin`, TCP, `tcp_raw.bin` và `lsl_raw.bin` đều dùng v2.

**EMG không phải Volt.** Đó là ADC count 10-bit (`0..1023`) từ
`analogRead(A0)`. Quy đổi:

```text
V = emg * Vref / 1023
```

`Vref` là 5 V hoặc 3.3 V tùy công tắc I/O trên RedBoard. Plot Volt:

```bash
python plot_emg.py --vref 5
```

LSL stream `SparkFun_MyoWare_EMG` có 4 kênh `int64`: `packet_id`,
`device_timestamp_us`, `emg`, `utc_timestamp_s`.

## Troubleshooting

| Hiện tượng                         | Cách xử lý                                      |
| ---------------------------------- | ----------------------------------------------- |
| `Resource busy` trên cổng serial   | Đóng Serial Monitor / process đang giữ cổng     |
| Không thấy LSL stream              | Chạy collector trước, không dùng `--no-lsl`     |
| TCP client không kết nối được      | Kiểm tra IP LAN, firewall, `--tcp-port`         |
| Plot trống                         | Collector phải đang chạy và TCP không bị `--no-tcp` |
