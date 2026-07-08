# 20260705_145746 OpenMV 抗卡死主动串流测试

## 文件

- MSPM0 当前烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- MSPM0 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260705_145444_mspm0_openmv_joint_home_850_1500_link_diag.c`
- OpenMV 新脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260705_145746_openmv_uart_stream_watchdog_no_print_rot180.py`

## 这版解决的问题

现场现象是：OpenMV 灯还会随红色目标变化，但舵机有时不动，随后 OpenMV 可能卡死或断续停止。

这版 OpenMV 脚本只改视觉端，不改 MSPM0 舵机端，便于隔离原因：

- OpenMV 主动每约 25ms 发送一帧颜色识别结果，不依赖 MSPM0 请求。
- 脱机运行时不 `print`，避免无 USB 控制台时输出阻塞。
- UART RX 每轮最多清 16 字节，避免 RX 噪声或异常输入让视觉循环一直耗在 `uart.any()`。
- 摄像头初始化失败时红灯闪烁并重试，不让脚本直接退出。
- 捕获 Python 层异常，红灯短亮并重新初始化摄像头。
- 如果 OpenMV 固件支持 WDT，则启用约 3s 看门狗；若底层 `sensor.snapshot()` 等硬卡死，看门狗会自动复位 OpenMV。
- 每 80 帧做一次 `gc.collect()`，降低长时间运行的内存碎片风险。

## 接线保持

- OpenMV P4 / UART3_TX -> MSPM0 PA11 / UART0_RX
- OpenMV P5 / UART3_RX <- MSPM0 PA10 / UART0_TX
- OpenMV GND -> MSPM0 GND
- PA14 -> 水平舵机信号
- PA17 -> 俯仰舵机信号
- 舵机红线 -> 稳定 5V
- 舵机黑/棕线 -> 公共 GND

## 测试步骤

1. OpenMV IDE 打开并运行 `20260705_145746_openmv_uart_stream_watchdog_no_print_rot180.py`。
2. 保存脚本到 OpenMV 主板，让 OpenMV 脱离电脑后也自动运行。
3. CCS 手动烧录当前 `C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`。
4. 先静止测试红色目标追踪，再长时间运行观察是否还出现“灯变但舵机不动/随后卡死”。

## 如果再次出现问题

连接电脑 Watch MSPM0 变量，重点看：

- `g_omv_rx_bytes`
- `g_omv_frame_count`
- `g_omv_color_frame_count`
- `g_omv_target_detected`
- `g_omv_pixels`
- `g_omv_timeout_frames`
- `g_aim_state`
- `g_aim_update_count`
- `g_aim_x_step_us`
- `g_aim_y_step_us`
- `g_servo_x_us`
- `g_servo_y_us`
- `g_diag_stage`
- `g_diag_no_link_count`
- `g_diag_no_target_count`
- `g_diag_small_target_count`
- `g_diag_confirm_wait_count`
- `g_diag_deadband_count`
- `g_diag_servo_update_count`
- `g_diag_limited_count`

`g_diag_stage` 含义：

- `0`：MSPM0 没收到有效 OpenMV 帧或超时
- `1`：收到有效颜色帧
- `2`：收到帧但 OpenMV 说没有目标
- `3`：目标像素太小，被 MSPM0 拒收
- `4`：目标确认等待
- `5`：目标在死区内，舵机本来就不动
- `6`：MSPM0 已更新舵机命令
- `7`：舵机命令撞到软件限幅
