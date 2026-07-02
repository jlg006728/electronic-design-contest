# OpenMV - MSPM0 串口通信协议

## 硬件连接

```text
MSPM0 PA10 (UART0 TX) -> OpenMV RX
MSPM0 PA11 (UART0 RX) <- OpenMV TX
MSPM0 GND             -> OpenMV GND
```

OpenMV H7 使用 `UART(3, 115200)`。官方文档中 `UART3 TX=P4`、`UART3 RX=P5`，因此常用接法是：

```text
MSPM0 PA10 (UART0 TX) -> OpenMV P5 (UART3 RX)
MSPM0 PA11 (UART0 RX) <- OpenMV P4 (UART3 TX)
```

不同扩展板丝印可能写 P4/P5 或 TX/RX，最终按“TX 接对方 RX、RX 接对方 TX”核对。

## 通信参数

| 参数 | 值 |
|------|-----|
| 波特率 | 115200 bps |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |

## 请求-响应模式

MSPM0 定时发送一个字节：

```text
'X' = 0x58
```

OpenMV 收到 `X` 后拍一帧图像，识别红色靶心或红色圆弧，并回传一帧二进制数据。

## 二进制帧格式

当前 `ccs_project/e_car/openmv.c` 与 `openmv/openmv_e_target.py` 使用 8 字节固定帧：

```text
byte0  0x2C                  帧头
byte1  detected              1=识别到目标, 0=未识别
byte2  cx low
byte3  cx high               小端 int16
byte4  cy low
byte5  cy high               小端 int16
byte6  checksum              byte1~byte5 异或
byte7  0x5B                  帧尾
```

未识别时：

```text
detected = 0
cx = 0xFFFF
cy = 0xFFFF
```

## OpenMV 发送端示例

完整脚本见：

```text
ccs_project/e_car/openmv/openmv_e_target.py
```

核心发送函数：

```python
def send_target(detected, cx, cy):
    if not detected:
        cx = 0xFFFF
        cy = 0xFFFF
    payload = bytearray([
        1 if detected else 0,
        cx & 0xFF,
        (cx >> 8) & 0xFF,
        cy & 0xFF,
        (cy >> 8) & 0xFF,
    ])
    chk = 0
    for b in payload:
        chk ^= b
    uart.write(bytearray([0x2C]) + payload + bytearray([chk, 0x5B]))
```

## MSPM0 接收端

当前车端代码在 `UART0_IRQHandler()` 中读 UART RX FIFO，并逐字节调用：

```c
openmv_uart_isr(byte);
```

解析成功后：

```c
if (openmv_data_ready()) {
    openmv_data_t target = openmv_get_data();
    gimbal_aiming_update(&target);
}
```

## 调试注意

- 使用 UART0 接 OpenMV 时，不要同时用 UART0 做 printf。
- LaunchPad 的 XDS110 虚拟串口可能占用 PA10/PA11，联调 OpenMV 前建议拔掉 J21/J22 跳线帽。
- 如果坐标一直不更新，先只让 OpenMV 用 USB 运行脚本，在 IDE 里确认阈值能识别红色靶心。
