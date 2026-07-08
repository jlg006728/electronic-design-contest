# 20260703_215935 OpenMV 主动串流排查版

## 文件

- OpenMV 主动串流脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_215935_openmv_uart_stream_rot180.py`
- 当前 MSPM0 烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- MSPM0 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_215935_mspm0_openmv_track_uart_stream.c`

## 为什么要改

现场现象：

- `g_omv_tx_requests` 一直增加：MSPM0 正在发请求。
- `g_omv_rx_bytes` 一直为 0：MSPM0 完全没有收到 OpenMV 回来的字节。
- OpenMV 灯遇到红色物体会变化：只能证明 OpenMV 本地视觉在运行，不能证明 UART 回传链路是通的。

原脚本只有在收到 MSPM0 发来的 `'C'` 命令后才回包。如果 `MSPM0 PA10 -> OpenMV P5` 这条命令线不通，OpenMV 就不会回包，MSPM0 的 `Rx` 会一直是 0。

新脚本改成：OpenMV 即使没收到命令，也会每隔约 `100ms` 主动从 UART3 发送一帧颜色识别结果。

## 使用步骤

只有一根 USB 时按这个顺序：

1. USB 插 OpenMV。
2. OpenMV IDE 打开：
   `C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_215935_openmv_uart_stream_rot180.py`
3. 点运行，确认终端出现：
   `OpenMV UART stream ready: UART3=115200, QVGA RGB565, rot180=1`
4. 在 OpenMV IDE 执行 `Tools -> Save Script to OpenMV Cam`，保存为开机自动运行脚本。
5. 拔掉 OpenMV USB。
6. 给 OpenMV 继续供电。
7. USB 插回 MSPM0。
8. CCS 烧录/运行当前 `empty.c`，看 Watch。

## 接线

至少必须有：

- `OpenMV P4 TX -> MSPM0 PA11 RX`
- `OpenMV GND -> MSPM0 GND`

如果还要保留请求命令线：

- `MSPM0 PA10 TX -> OpenMV P5 RX`

主动串流版下，先只要 P4->PA11 这条回传线通，MSPM0 的 `g_omv_rx_bytes` 就应该增加。

## Watch 判断

- `g_omv_rx_bytes` 增加：MSPM0 已收到 OpenMV 字节，P4->PA11 回传链路通。
- `g_omv_color_frame_count` 增加：MSPM0 成功解析颜色帧。
- `g_omv_target_detected = 1`：MSPM0 收到的帧确认检测到目标。
- `g_servo_x_us/g_servo_y_us` 变化：追踪控制已经开始改舵机命令。

如果使用主动串流版后 `g_omv_rx_bytes` 仍然为 0：

1. 查 `OpenMV P4 TX -> MSPM0 PA11 RX` 是否接错。
2. 查 OpenMV GND 和 MSPM0 GND 是否共地。
3. 查 PA11 是否被 LaunchPad 板载 backchannel UART 跳线帽干扰。
4. 查是否真的保存并运行了主动串流版 `main.py`。
