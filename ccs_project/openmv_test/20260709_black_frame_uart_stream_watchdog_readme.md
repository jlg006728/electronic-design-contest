# OpenMV 黑框中心 UART 上报版说明

## 适用场景

本版本用于替代早期“红色目标/红色圆环识别”方案。现场测试表明红色圆环颜色淡、线条细，颜色阈值容易漏检或被环境红色干扰；比赛目标外侧黑框稳定存在，因此改为识别黑色外框，并把黑框四角平均值作为目标中心。

## 当前文件

- 主脚本：`ccs_project/openmv_test/20260709_black_frame_uart_stream_watchdog.py`
- 当前烧录目标：OpenMV 盘符 `E:\main.py`
- 烧录前备份：`ccs_project/openmv_test/openmv_main_backup_20260709_before_black_frame_generalized.py`
- 纯黑框测试版：`ccs_project/openmv_test/20260709_black_frame_center_test.py`
- 红色识别测试版：`ccs_project/openmv_test/20260709_simple_red_color_sensitivity_test.py`

## 接线

```text
OpenMV P4 / UART3_TX -> MSPM0 PA11 / UART0_RX
OpenMV P5 / UART3_RX <- MSPM0 PA10 / UART0_TX
OpenMV GND           -> MSPM0 GND
```

OpenMV 可以用 USB 或稳定 5V/VIN 供电。调试时必须保证 OpenMV、MSPM0、舵机供电共地。

## UART 协议保持不变

仍使用原 17 字节帧：

```text
0:  0xAA
1:  0x55
2:  type      1=P, 2=S, 3=C
3:  seq low
4:  seq high
5:  flags     bit0=camera_ok, bit1=target_detected, bit2=uart_rx_seen
6:  cx low    int16, -1 when no target
7:  cx high
8:  cy low
9:  cy high
10: pixels low
11: pixels high
12: fps_x10 low
13: fps_x10 high
14: l_mean
15: checksum XOR of bytes 2..14
16: 0x5B
```

MSPM0 端原则上可以继续沿用原 OpenMV 目标帧解析逻辑：

- `FLAG_TARGET_DETECTED=1`：识别到黑框中心。
- `cx/cy`：黑框四角平均中心。
- `cx=-1, cy=-1`：当前没有目标。
- `pixels`：blob 路线下为黑色像素数，rect 路线下为候选框面积；MSPM0 当前主要使用 `cx/cy` 即可。

## 检测流程

1. 保持 QVGA/RGB565，图像按当前倒装安装做 `hmirror + vflip`。
2. 优先调用 `find_rects()` 检测矩形黑框。
3. 若矩形检测不稳定，则使用黑色 blob 备用路线。
4. 若普通 blob 没合并成框，则启用 `GROUP` 兜底，把多个非贴边黑块组合成一个候选框。
5. 对候选进行贴边、尺寸、宽高比、黑色密度过滤。
6. 成功后取四角平均值，作为 UART 上报的 `cx/cy`。
7. 若当前帧漏检，最多用最近一次目标保持 `TARGET_HOLD_FRAMES=8` 帧，减少 UART 目标闪断。

## 终端反馈

当前 `ENABLE_DEBUG_PRINT=True`，OpenMV IDE 串行终端会看到：

```text
OMV black-frame fps=57.0 l=120 target=1 cx=141 cy=132 pixels=2365 hold=8 uart_seen=0 exc=0
```

字段含义：

- `target=1`：当前输出有效目标。
- `cx/cy`：回传给 MSPM0 的黑框中心。
- `pixels`：候选强度参考。
- `hold=8` 附近：当前帧真实识别到目标。
- `hold=7/6/5...`：当前可能短时漏检，正在沿用上一帧目标。
- `target=0`：连续超过保持帧数仍未识别。
- `exc`：异常恢复次数。

正式上车若担心 USB 打印带来开销，可把 `ENABLE_DEBUG_PRINT` 改为 `False`。

## 关键调参项

| 参数 | 作用 | 调整建议 |
|---|---|---|
| `BLACK_THRESHOLDS` | 黑色/灰黑阈值 | 漏检黑框时放宽 L 上限；误检暗色背景时收紧 |
| `RECT_THRESHOLD` | `find_rects()` 灵敏度 | 矩形路线抓不到时降低；误检矩形时升高 |
| `BLACK_MERGE_MARGIN` | blob 合并距离 | 黑框断裂时增大；误合并背景时减小 |
| `MIN_BOX_W/H` | 最小目标尺寸 | 远距离小目标漏检时减小 |
| `MAX_BOX_W/H` | 最大目标尺寸 | 近距离目标被 size 拒绝时增大 |
| `MIN/MAX_ASPECT_X100` | 宽高比范围 | 透视角度大时放宽 |
| `MIN/MAX_DENSITY_PERCENT` | 黑色密度范围 | 线框太细时降低下限，实心干扰多时降低上限 |
| `TARGET_HOLD_FRAMES` | 漏检保持帧数 | 目标闪断时增大，快速移动时减小 |

## 当前结论

现场验证中，`find_rects()` 不一定稳定，常见成功路线为 `best=blob` 或集成版中的 blob/GROUP 备用路线。成功中心曾稳定在 `cx=131~141`、`cy=131~132`，帧率约 `57fps`。后续与 MSPM0 云台集成时，应优先观察 `FLAG_TARGET_DETECTED`、`cx/cy` 和舵机响应是否稳定。
