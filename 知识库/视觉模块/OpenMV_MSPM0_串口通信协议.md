# OpenMV - MSPM0 串口通信协议

## 硬件连接

```
OpenMV P4 (TX) ──→ MSPM0 RX (PA10)
OpenMV P5 (RX) ──→ MSPM0 TX (PA9)
OpenMV GND     ──→ MSPM0 GND
```

## 通信参数

| 参数 | 值 |
|------|-----|
| 波特率 | 115200 bps |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |

## 二进制帧协议 (推荐)

```
帧格式:
| 帧头(0x2C) | 数据长(0x04) | cx_lo | cy_lo | cx_hi | cy_hi | 帧尾(0x5B) |
|   1 byte   |    1 byte    |  1B  |  1B  |  1B  |  1B  |   1 byte    |
总长: 7 bytes
```

### OpenMV 发送端

```python
from pyb import UART
uart = UART(3, 115200)

def send_target(cx, cy):
    data = bytearray([0x2C, 4,
                      cx & 0xFF, cy & 0xFF,
                      cx >> 8,   cy >> 8,
                      0x5B])
    uart.write(data)
```

### MSPM0 接收端 (状态机)

```c
typedef struct {
    uint16_t cx, cy;
} TargetPos;

TargetPos target = {0};

void parse_openmv(uint8_t byte)
{
    static uint8_t buf[7], state = 0, idx = 0;

    switch (state) {
    case 0:  // 等待帧头
        if (byte == 0x2C) { buf[idx++] = byte; state = 1; }
        break;
    case 1:  // 数据长度
        if (byte == 4) { buf[idx++] = byte; state = 2; }
        else { state = 0; idx = 0; }
        break;
    case 2:  // 接收数据
        buf[idx++] = byte;
        if (idx >= 6) state = 3;
        break;
    case 3:  // 帧尾
        if (byte == 0x5B && idx == 6) {
            target.cx = buf[2] | (buf[4] << 8);
            target.cy = buf[3] | (buf[5] << 8);
        }
        state = 0; idx = 0;
        break;
    }
}

// 在 UART RX ISR 中调用
void UART_IRQHandler(void) {
    uint8_t byte;
    if (DL_UART_receiveDataCheck(UART_INST, &byte)) {
        parse_openmv(byte);
    }
}
```

## 请求-响应模式

减少中断频率，MCU 主动请求数据:

### OpenMV 端

```python
while True:
    if uart.any():
        cmd = uart.readchar()
        if cmd == ord('X'):  # 请求坐标
            img = sensor.snapshot()
            target = find_target(img)
            if target:
                send_target(target.cx(), target.cy())
            else:
                uart.write(bytearray([0x2C, 0, 0x5B]))  # 空帧=未找到
```

### MSPM0 端

```c
// 定时请求 (如 50Hz)
void timer_callback(void) {
    DL_UART_transmitData(UART_INST, 'X');  // 发送请求
}
// 然后在 RX ISR 中解析响应帧
```

## JSON 文本协议 (备选)

```python
# 简单, 但效率低
import json
data = json.dumps({'cx': cx, 'cy': cy})
uart.write(data + '
')
```

MCU 端需用 sscanf 或简单字符串解析。

> 推荐使用二进制帧协议: 7字节/帧, 效率高, 解析快, 适合电赛4天开发周期。
