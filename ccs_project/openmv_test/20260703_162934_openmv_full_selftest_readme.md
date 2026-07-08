# 20260703 OpenMV 完整自检步骤

## 1. 先只测 OpenMV 本体

1. OpenMV 用 USB 连接电脑。
2. 在 OpenMV IDE 打开并运行：
   `C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_162934_openmv_full_selftest.py`
3. 正常现象：
   - IDE 画面窗口持续刷新。
   - 终端每秒打印 `OMV selftest fps=... l=... target=...`。
   - OpenMV 蓝灯周期闪烁。
   - 拿红色物体/红色标记进入画面时，绿色框和十字会跟随目标，`target=1`。

没有红色目标时 `target=0` 是正常的；只要画面刷新、FPS 和亮度在变，摄像头本体就是通的。

## 2. 再测 MSPM0 和 OpenMV 串口

断电后接线：

```text
OpenMV P4 / UART3_TX -> MSPM0 PA11 / UART0_RX
OpenMV P5 / UART3_RX <- MSPM0 PA10 / UART0_TX
OpenMV GND           -> MSPM0 GND
```

注意：

- OpenMV 可以先继续用 USB 供电，不要同时再接外部 VIN。
- MSPM0 也可以继续用 USB 烧录/调试。
- 必须拔掉 LaunchPad 的 `J21/J22` backchannel UART 跳线帽，避免 XDS110 虚拟串口占用 PA10/PA11。

MSPM0 端烧录：

```text
C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_162934_mspm0_openmv_full_selftest.c
```

当前活动工程会被替换为同一份代码：

```text
C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c
```

## 3. CCS Watch 变量

重点看：

```text
g_omv_link_ok
g_omv_frame_count
g_omv_last_cmd
g_omv_frame_type
g_omv_camera_ok
g_omv_target_detected
g_omv_cx
g_omv_cy
g_omv_pixels
g_omv_fps_x10
g_omv_l_mean
g_omv_bad_checksum
g_omv_bad_footer
g_omv_timeout_count
```

正常现象：

```text
g_omv_frame_count 持续增加
g_omv_link_ok = 1
g_omv_camera_ok = 1
g_omv_bad_checksum 基本不增加
g_omv_bad_footer 基本不增加
PA15 LED 随收到 OpenMV 帧而翻转
```

拿红色目标进入画面时：

```text
g_omv_target_detected = 1
g_omv_cx / g_omv_cy 随目标位置变化
g_omv_pixels 随目标面积变化
```

如果 `g_omv_frame_count` 不增加，优先查：

1. OpenMV 脚本是否正在运行，而不是只打开文件。
2. `P4 TX -> PA11 RX`、`P5 RX <- PA10 TX` 是否交叉接反。
3. OpenMV GND 是否和 MSPM0 GND 共地。
4. LaunchPad `J21/J22` 是否已拔掉。
5. OpenMV 是否只用一种 5V/VIN/USB 供电方式，没有互相反灌。
