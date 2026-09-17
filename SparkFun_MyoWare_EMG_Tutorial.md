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

## Định nghĩa

### EMG

Điện cơ (electromyography): điện thế khi cơ co. Surface EMG thường mạnh nhất khoảng
10–500 Hz, biên độ thô trước khuếch đại cỡ µV–mV.

### ADC count

Giá trị `analogRead(A0)` trên RedBoard, ADC 10-bit nên miền **0…1023**. Đây là
đơn vị mặc định trong `raw.bin` và trục Y của `plot_emg.py`. Không phải Volt.

```text
V = emg * Vref / 1023
```

`Vref` là 5 V hoặc 3.3 V tùy công tắc I/O. Plot Volt: `python plot_emg.py --vref 5`.

### RAW, RECT, ENV (chân MyoWare)

MyoWare 2.0 có ba lối analog. Firmware tutorial đọc **một** chân qua `EMG_PIN`
(thường A0 / shield SIG):

| Chân | Tín hiệu | Băng thông gần đúng | Ghi chú |
| ---- | -------- | ------------------- | ------- |
| RAW  | EMG hai cực, khuếch đại ~200× | ~20–500 Hz | Nghỉ quanh `Vref/2` (ADC ~512) |
| RECT | RAW đã chỉnh lưu toàn sóng | vẫn nhanh | Chưa làm mượt |
| ENV  | Envelope: chỉnh lưu + lọc ~3.6 Hz | rất chậm | 0…Vcc; nghỉ gần 0, co cơ tăng |

ENV là envelope **phần cứng**. Không nhầm với đường RMS trên plot (xem dưới).

### Nyquist và 1000 Hz

Tần số lấy mẫu phải ≥ 2× tần số cao nhất của tín hiệu. RAW tới ~500 Hz nên
**1000 Hz là mức tối thiểu**. ENV đã lọc 3.6 Hz nên 100–200 Hz đã đủ; firmware
vẫn khóa 1000 Hz cho an toàn cả hai chế độ và còn headroom UART 230400 baud.

### RMS (đường đỏ trên `plot_emg.py`)

**Root Mean Square** — biên độ hiệu dụng, tính **trên máy plot**, không gửi từ
board, không ghi vào `raw.bin`.

Với cửa sổ trượt khoảng **80 ms** (`N ≈ sample_rate × 0.08`):

```text
RMS[i] = sqrt( (1/N) * sum_{k=i-N+1..i}  x[k]^2 )
```

`x[k]` là mẫu đang vẽ (ADC count, hoặc Volt nếu `--vref`).

| Đường trên plot | Ý nghĩa |
| --------------- | ------- |
| raw (xanh nhạt) | Từng mẫu ADC; dao động nhanh nên dễ thành “khối” |
| RMS (đỏ)        | Biên độ trung bình trượt; nghỉ thấp / co cơ nhô |

RMS **không** phải chân ENV. Khác biệt:

- ENV: analog trên MyoWare, lọc ~3.6 Hz, đã có trong từng sample nếu A0 = ENV.
- RMS: overlay ~80 ms trong `plot_emg.py` (`rolling_rms`), chỉ để nhìn realtime.

Nếu A0 là ENV, RMS gần với biên độ đã làm mượt. Nếu A0 là RAW (quanh 512), RMS
vẫn mang DC offset nên lúc nghỉ không về 0.

### CRC16

Checksum CRC-16/CCITT-FALSE trên packet trừ trường CRC. Sai CRC → collector
bỏ packet, đếm `CRC errors`, rồi resync.

### Packet loss

`packet_id` trên firmware tăng 1 mỗi mẫu. Collector thấy khoảng nhảy ID thì
cộng `Lost` và ghi `lost_intervals` vào `raw.bin.json` (`start_timestamp` /
`end_timestamp`). Rút USB cũng là một khoảng `usb_disconnect`.

## Arduino Firmware (packet dây v1, 15 byte)

Firmware lấy mẫu **cố định 1000 Hz** rồi gửi trên Serial `230400` baud.
Không chạy `loop()` hết tốc lực: packet 15 byte chiếm ~150 bit, trần lý thuyết
khoảng 1536 Hz; ~1480 Hz trước đây sát trần nên CH340 dễ overrun.

MyoWare RAW có băng thông ~20–500 Hz nên Nyquist tối thiểu là 1000 Hz. ENV
đã lọc 3.6 Hz, 100–200 Hz là đủ, nhưng 1000 Hz vẫn an toàn cho cả hai và
còn ~35% headroom UART. Đổi `SAMPLE_HZ` nếu cần.

```cpp
#include <Arduino.h>

#define EMG_PIN A0
#define BAUDRATE 230400
#define SAMPLE_HZ 1000
#define PACKET_HEADER 0xAA55
#define PACKET_VERSION 1

#pragma pack(push, 1)
struct EMGPacket {
    uint16_t header;
    uint8_t version;
    uint32_t packetID;
    uint32_t timestamp;
    uint16_t emg;
    uint16_t crc;
};
#pragma pack(pop)

static EMGPacket packet;
static uint32_t packetCounter = 0;
static uint32_t nextSampleUs = 0;
static const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_HZ;

uint16_t crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (*data++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void setup() {
    Serial.begin(BAUDRATE);
    analogReference(DEFAULT);
    pinMode(EMG_PIN, INPUT);
    packet.header = PACKET_HEADER;
    packet.version = PACKET_VERSION;
    nextSampleUs = micros();
}

void loop() {
    uint32_t now = micros();
    if ((int32_t)(now - nextSampleUs) < 0) {
        return;
    }
    if ((int32_t)(now - nextSampleUs) > (int32_t)SAMPLE_PERIOD_US) {
        nextSampleUs = now;
    }
    nextSampleUs += SAMPLE_PERIOD_US;

    packet.packetID = packetCounter++;
    packet.timestamp = now;
    packet.emg = analogRead(EMG_PIN);
    packet.crc = crc16((uint8_t *)&packet, sizeof(packet) - sizeof(packet.crc));
    Serial.write((uint8_t *)&packet, sizeof(packet));
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

Hai đường: **raw** (xanh, từng mẫu ADC) và **RMS** (đỏ, biên độ trượt ~80 ms).
Định nghĩa RMS, ADC, RAW/ENV nằm ở mục [Định nghĩa](#định-nghĩa). Máy khác trên
LAN phải truyền `--host <IP_collector>` (mặc định là `127.0.0.1`).

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

## Tín hiệu không đổi khi co cơ

Collector/plot chạy được nghĩa là USB ổn. Nếu co cơ mà đường vẽ gần như đứng yên
thì gần như chắc là **cảm biến / gain / điện cực / chân đọc**, không phải TCP.

Session `data/session01/raw.bin` điển hình của lỗi này: hầu hết mẫu nằm
**930–932 ADC**, `max |delta|` giữa hai mẫu liên tiếp chỉ **1**. Đó là đường
thẳng, không phải EMG.

### ENV tốt trông như thế nào

Muốn “thấy co cơ” thì jumper/output shield phải là **ENV** (mặc định MyoWare):

| Trạng thái | ADC (ENV) khoảng |
| ---------- | ---------------- |
| Nghỉ, cơ thả | thấp, ~50–250 |
| Co vừa | tăng rõ, vài trăm |
| Co mạnh | cao nhưng **không dính trần 1023** |

Nếu nghỉ đã ~900–1023: **gain quá lớn**, co hay thả đều bão hòa nên plot không đổi.

### Việc nên làm theo thứ tự

1. **LED trên MyoWare 2.0**  
   - LED **VIN**: sáng cố định khi công tắc ON (có nguồn).  
   - LED **ENV** (vàng/amber): sáng khi chân ENV đủ cao, tức co cơ.  
   Co cơ mà ENV lúc nháy lúc không thường là **điện cực/da/REF tiếp xúc kém**,
   hoặc biên độ đúng ngưỡng LED — chỉnh pad trước, chưa vặn GAIN.
   ENV nháy lúc không dán pad là bình thường (input floating).
2. **Ba điện cực**  
   MID và END dọc thớ cơ (cách nhau vài cm), **REF trên xương** (khuỷu, mắt cá),
   không đặt REF trên cùng bụng cơ. Dùng điện cực gel mới, lau da.
3. **Chiết áp GAIN** (chỉ chỉnh ENV; RAW/RECT cố định ×200)  
   Nằm trên **board cảm biến** (không phải RedBoard), góc trên-trái, chữ
   **GAIN**. Stack shield (Cable/Link/Power) che mất chiết áp — tháo shield
   trên cùng mới vặn được. Tua vít Phillips nhỏ, nhẹ tay, không vặn quá cứng.  
   - **Ngược chiều kim đồng hồ** = giảm gain.  
   - **Cùng chiều kim đồng hồ** = tăng gain.  
   Giảm đến khi nghỉ thấp (~50–250 ADC); co cơ RMS đỏ nhô, raw không dính 1023.
4. **Đúng chân analog**  
   Firmware đọc `A0`. Shield/Link: jumper **ENV** (không phải RAW) nếu mục tiêu
   là thấy mức co. RAW dao động quanh ~512, biên độ nhỏ, RMS không về 0 — trông
   như “không đổi” nếu nhìn cả thang 0–1023.
5. **Nguồn và công tắc**  
   MyoWare 2.0 cần Power Shield / nguồn đúng; công tắc board ON. RedBoard I/O
   5 V hoặc 3.3 V phải khớp `Vref` nếu plot `--vref`.
6. **Đối chiếu số**  
   `python print_raw.py data/session01/raw.bin --seconds 5`  
   Cột `emg` phải thay đổi hàng chục–hàng trăm khi xen kẽ nghỉ/co. Đứng ~930
   như file cũ là chưa có EMG.

RAW chỉ cần khi phân tích phổ; quan sát co cơ realtime dùng ENV + RMS trên plot.

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
