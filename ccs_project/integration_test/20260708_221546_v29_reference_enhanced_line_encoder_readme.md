# 20260708_221546 V29 参考代码增强版：红外多采样 + 黑线锁存 + 编码器里程

## 基底

- 基于 V28：`20260708_220234_v28_state_switch_debug_cleanup.c`。
- 当前 CCS 活动文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`。
- 本版只编译验证，未自动烧录。

## 保持不变的接口

- TB6612：`PB0/PB1/PB2/PB3/PB4`，`PA12/PA13` PWM。
- 红外：`S1/S2/S3/S4/S5/S6/S7 -> PB5/PB6/PB7/PA7/PA8/PB9/PB10`。
- 编码器：左 `PB12/PB13`，右 `PB14/PB15`。
- OpenMV UART：`PA10/PA11`。
- 舵机：`PA14/PA17`。
- 蜂鸣器：`PB11`。
- 当前烧录入口仍是 `C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`。

## 参考代码学习点

25E 参考代码不是单次读传感器，也不是纯 PWM：

1. `GraySensorDataUpdate()` 每通道多次采样，丢掉第一次，后几次平均。
2. `EncoderDataUpdate()` 把编码器脉冲换算为速度和累计距离 `X`。
3. `Task()` 通过灰度触发 + 编码器距离 + 强制转向状态机完成直角弯。
4. `MotorPidCtrl()` 用平均速度 + 转向差速做电机速度闭环。

本版暂不移植四电机 PID，因为当前硬件是双电机 TB6612，接口不同；先移植“多采样、锁存、编码器里程”三个低风险增强。

## V29 关键改动

### 1. 红外多次采样

新增：

```c
#define LINE_SAMPLE_COUNT           5U
#define LINE_SAMPLE_DELAY_CYCLES    80U
```

每次 `read_line_sensors()` 连续读取 5 次，记录：

```c
g_line_raw_high_last_mask
g_line_raw_high_or_mask
g_line_raw_high_and_mask
```

当前 `LINE_BLACK_IS_HIGH=1`，所以逻辑黑线使用 5 次采样的 OR 结果。只要 5 次里有一次读到黑线，就认为该路看到黑线。

### 2. 黑线锁存

新增：

```c
#define LINE_SEEN_LATCH_FRAMES      8U
```

只要任意红外看到黑线，就锁存约 8 个主循环帧：

```c
g_line_seen_latched
g_line_seen_latch_frames
g_line_seen_event_count
```

桥接阶段触发条件改为：

```c
g_line_valid != 0 || g_line_seen_latched != 0
```

目的：解决车运动时黑线一闪而过、单帧漏采样的问题。

### 3. 编码器里程 Watch

新增：

```c
g_encoder_left_forward_total
g_encoder_right_forward_total
g_encoder_avg_forward_total
g_encoder_diff_forward_total
```

沿用当前方向约定：

```text
左轮前进量 = -g_left_count
右轮前进量 =  g_right_count
平均前进量 = (左 + 右) / 2
左右差值   = 左 - 右
```

这版先只用于 Watch 和现场标定。后续可以用 `g_encoder_avg_forward_total` 替代 `g_run_sample` 做 B/C/D/A 距离触发。

## 现场测试重点

### 白底 A 点启动

应看到：

```text
停 2 秒 + 2 声
```

说明进入桥接。

### 桥接经过黑线

如果任意红外采到黑线，应看到：

```text
停 2 秒 + 3 声
```

如果 V29 仍然没有 3 声：

- 说明不是单帧采样漏检。
- 优先怀疑传感器阵列实际没有压到黑线，或行驶中传感器高度/角度不对。

### 黑线启动

应看到：

```text
停 2 秒 + 4 声
```

如果黑线启动后仍无法循迹，重点看：

```c
g_line_raw_mask
g_line_error
g_line_correction
g_left_pwm_ticks
g_right_pwm_ticks
```

若红外读数在变但 PWM 差速不明显，下一步应恢复/提高普通循迹 PWM 参数，而不是继续改重捕逻辑。

## 后续建议

1. 先用 V29 判断“桥接时是否能采到黑线”。
2. 若能采到但循迹失败，下一版恢复接近稳定 baseline 的循迹 PWM。
3. 若仍采不到，先处理机械位置：红外高度、传感器前伸位置、A 点摆车角度。
4. 编码器里程稳定后，用 `g_encoder_avg_forward_total` 标定 `edges_per_cm`，再把 B/C/D/A 任务点从 `g_run_sample` 切到编码器距离。
