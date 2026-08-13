# SparkFun MyoWare EMG Collector Tutorial

## Mục tiêu

Thu thập EMG từ MyoWare trên macOS, lưu `raw.bin`, và phát ra LAN theo hai
cách: LSL hoặc TCP.

```
MyoWare -> RedBoard -> USB -> collect_emg.py
                           |-> raw.bin + raw.bin.json
                           |-> LSL Outlet          (tuỳ chọn)
                           |-> TCP 0.0.0.0:8765    (tuỳ chọn, mặc định bật)
```

Thiết bị cùng mạng LAN có thể nhận data qua TCP, không cần cài LSL.

## Phần cứng

- SparkFun RedBoard Plus
- MyoWare Muscle Sensor + Shield
- macOS
- USB-C

## Arduino Firmware (packet dây v1, 15 byte)

Firmware gửi liên tục trên Serial `230400` baud:

```cpp
#include <Arduino.h>

#define EMG_PIN A0
#define BAUDRATE 230400
#define PACKET_HEADER 0xAA55
#define PACKET_VERSION 1

#pragma pack(push,1)
struct EMGPacket{
    uint16_t header;
    uint8_t version;
    uint32_t packetID;
    uint32_t timestamp;
    uint16_t emg;
    uint16_t crc;
};
#pragma pack(pop)

EMGPacket packet;
uint32_t packetCounter=0;

uint16_t crc16(const uint8_t *data,uint16_t len){
    uint16_t crc=0xFFFF;
    while(len--){
        crc ^= (*data++)<<8;
        for(uint8_t i=0;i<8;i++)
            crc = (crc & 0x8000)?((crc<<1)^0x1021):(crc<<1);
    }
    return crc;
}

void setup(){
    Serial.begin(BAUDRATE);
    analogReference(DEFAULT);
    pinMode(EMG_PIN,INPUT);
    packet.header=PACKET_HEADER;
    packet.version=PACKET_VERSION;
}

void loop(){
    packet.packetID=packetCounter++;
    packet.timestamp=micros();
    packet.emg=analogRead(EMG_PIN);
    packet.crc=crc16((uint8_t*)&packet,sizeof(packet)-sizeof(packet.crc));
    Serial.write((uint8_t*)&packet,sizeof(packet));
}
```

### Packet dây (USB, version 1)

| Field           | Type   |
| --------------- | ------ |
| Header `0xAA55` | uint16 |
| Version `1`     | uint8  |
| PacketID        | uint32 |
| Timestamp (us)  | uint32 |
| EMG             | uint16 |
| CRC16           | uint16 |

Struct little-endian: `<HBIIHH`, 15 byte.

## Packet lưu / phát (version 2, 23 byte)

Python nhận packet v1 từ USB, gắn UTC Unix **theo giây** lúc Mac nhận, rồi
ghi/phát packet v2:

| Field              | Type   |
| ------------------ | ------ |
| Header `0xAA55`    | uint16 |
| Version `2`        | uint8  |
| PacketID           | uint32 |
| Timestamp (us)     | uint32 |
| EMG                | uint16 |
| UTC timestamp (s)  | uint64 |
| CRC16              | uint16 |

Struct little-endian: `<HBIIHQH`, 23 byte. CRC tính trên toàn packet trừ
trường CRC.

File `raw.bin`, stream TCP, và file `tcp_raw.bin` / `lsl_raw.bin` đều dùng
định dạng v2 này.

## Cài Python

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Tool:

| File              | Vai trò                                      |
| ----------------- | -------------------------------------------- |
| `collect_emg.py`  | Đọc USB, ghi `raw.bin`, LSL, TCP server      |
| `receive_lsl.py`  | Client LSL, ghi `lsl_raw.bin`                |
| `receive_tcp.py`  | Client TCP LAN, ghi `tcp_raw.bin`            |
| `plot_emg.py`     | Vẽ EMG realtime qua TCP                      |
| `print_raw.py`    | In text N giây đầu/cuối của file binary      |

## Collector

Liệt kê cổng serial:

```bash
python collect_emg.py --list-ports
```

Thu thập (ghi file + LSL + TCP):

```bash
python collect_emg.py \
  --port /dev/cu.usbserial-1130 \
  --output data/session01/raw.bin
```

Chỉ ghi file và phát TCP, không dùng LSL:

```bash
python collect_emg.py \
  --port /dev/cu.usbserial-1130 \
  --output data/session01/raw.bin \
  --no-lsl
```

Tự dừng sau 60 giây:

```bash
python collect_emg.py --port /dev/cu.usbserial-1130 --duration 60
```

Tùy chọn chính:

| Cờ            | Mặc định            | Ý nghĩa                          |
| ------------- | ------------------- | -------------------------------- |
| `--port`      | tự dò nếu chỉ 1 cổng | Cổng USB serial                 |
| `--output`    | `raw.bin`           | File binary v2                   |
| `--source-id` | `sparkfun-myo-001`  | LSL source_id                    |
| `--no-lsl`    | tắt                 | Không tạo LSL outlet             |
| `--tcp-host`  | `0.0.0.0`           | Bind TCP cho LAN                 |
| `--tcp-port`  | `8765`              | Cổng TCP                         |
| `--no-tcp`    | tắt                 | Không mở TCP server              |
| `--duration`  | chạy đến Ctrl+C     | Số giây thu                      |

Collector kiểm CRC, đồng bộ lại khi mất byte, thống kê packet loss. Metadata
được ghi cạnh file data, ví dụ `raw.bin.json`.

Nếu gặp `Resource busy`, đóng Arduino Serial Monitor rồi chạy lại.

## LSL

Collector tạo stream `SparkFun_MyoWare_EMG` với 4 kênh `int64`:

- `packet_id`
- `device_timestamp_us`
- `emg`
- `utc_timestamp_s`

Terminal thứ hai, cùng virtual environment:

```bash
python receive_lsl.py --output data/session01/lsl_raw.bin
```

Chọn đúng outlet và tự dừng sau 60 giây:

```bash
python receive_lsl.py \
  --source-id sparkfun-myo-001 \
  --duration 60 \
  --output data/session01/lsl_raw.bin
```

Receiver tái tạo packet v2 23 byte vào `lsl_raw.bin`.

## TCP LAN

TCP là cách nhận data trên LAN **không cần LSL**. Collector lắng nghe
`0.0.0.0:8765` và đẩy nguyên packet v2 tới mọi client đã kết nối. Client chậm
bị drop để không làm chậm vòng USB.

Trên máy thu (Mac), lấy IP LAN rồi chạy collector:

```bash
ipconfig getifaddr en0
python collect_emg.py --port /dev/cu.usbserial-1130 --no-lsl
```

Trên máy nhận cùng LAN:

```bash
python receive_tcp.py \
  --host 192.168.1.10 \
  --output data/session01/tcp_raw.bin
```

Đổi cổng:

```bash
python collect_emg.py --port /dev/cu.usbserial-1130 --tcp-port 9000 --no-lsl
python receive_tcp.py --host 192.168.1.10 --port 9000
```

Nếu macOS hỏi quyền mạng, cho phép Python nhận kết nối đến.

## Realtime plot

Không vẽ trong `collect_emg.py` để vòng USB không bị chậm. `plot_emg.py` là
client TCP riêng: nhận packet v2, giữ cửa sổ trượt 5 giây, refresh ~30 FPS.

Cùng máy với collector:

```bash
python plot_emg.py
```

Máy khác trên LAN:

```bash
python plot_emg.py --host 192.168.1.10 --window 5
```

Quy đổi sang Volt nếu biết Vref của RedBoard (5 hoặc 3.3):

```bash
python plot_emg.py --vref 5
```

Trục Y mặc định là ADC count `0..1023`. Trục X là thời gian tương đối (giây),
0 ở mép phải (mẫu mới nhất).

## In dữ liệu text

In 10 giây đầu và 10 giây cuối của `raw.bin` (cũng dùng được với
`tcp_raw.bin` / `lsl_raw.bin` v2):

```bash
python print_raw.py data/session01/raw.bin --seconds 10
```

Xuất CSV:

```bash
python print_raw.py data/session01/raw.bin \
  --seconds 10 \
  --output data/session01/raw_first_last_10s.csv
```

Cột: `relative_time_s,packet_id,device_timestamp_us,emg,utc_timestamp_s,utc_iso8601`.
UTC ISO có dạng `2026-08-06T02:52:15Z`. Tool đọc được cả file v1 cũ (cột UTC
trống) và file v2 mới.

## Kiến trúc đề xuất

```
Frenz SDK ----\
Polar SDK -----+--> Session Manager --> raw.bin + metadata.json
MindRove ------/
SparkFun ------/
```

Định dạng `raw.bin` thống nhất giúp replay, AI pipeline và đồng bộ nhiều
thiết bị.
