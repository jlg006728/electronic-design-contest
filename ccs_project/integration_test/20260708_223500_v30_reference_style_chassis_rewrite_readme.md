# 20260708_223500 V30 参考代码架构底盘重写版

## 基底

- 不再延续 V29 的集成版思路。
- 以 25E 参考代码的底盘架构为思路重写当前 `empty.c`。
- 当前活动文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`。
- 本版只编译验证，未自动烧录。

## 保持不变的接口

```text
TB6612：
PB0/PB1/PB2/PB3/PB4
PA12/PA13 PWM

红外：
S1 PB5
S2 PB6
S3 PB7
S4 PA7
S5 PA8
S6 PB9
S7 PB10

编码器：
左 PB12/PB13
右 PB14/PB15

蜂鸣器：
PB11

LED：
PA15
```

OpenMV、PA14/PA17 舵机本版不控制，避免底盘调试被视觉/舵机逻辑干扰。

## 和之前版本的区别

V30 是底盘重写，不是补丁：

```text
hardware_init()
line_sensor_update()
line_position_update()
encoder_distance_update()
line_follow_drive()
bridge_drive()
force_turn_drive()
```

主循环结构接近参考代码：

```text
更新编码器
更新红外
更新路线状态
输出电机 PWM
```

不再使用 OpenMV 串口、舵机软件 PWM、任务点预瞄等复杂逻辑。

## 参考代码吸收点

来自 25E 参考代码：

1. 传感器周期更新，而不是状态里临时乱读。
2. 传感器多次采样。
3. 编码器累计前进量。
4. 状态机控制运动阶段。
5. 丢线后不是立刻停车，而是进入强制转向找线。

## 当前状态机

```text
PHASE_START_GAP_BRIDGE = 8
白底启动，进入 A 点无线桥接。

PHASE_LINE_RUN = 2
黑线启动或桥接重捕线后进入普通循迹。

PHASE_FORCE_TURN = 10
普通循迹丢线后按最后误差方向强制转向找线。

PHASE_STOP = 4
超时或长时间丢线停车。
```

## 蜂鸣/停车提示

```text
上电 1 声：程序启动
停顿 + 2 声：白底启动，进入桥接
停顿 + 4 声：黑线启动，直接进入循迹
停顿 + 3 声：桥接时重捕黑线，进入循迹
长响：停车保护
```

## 红外策略

```c
#define LINE_SAMPLE_COUNT 7U
#define LINE_SEEN_LATCH_FRAMES 10U
```

每次连续读 7 次红外。当前黑线高电平，所以 7 次里任意一次读高，就认为该传感器看到黑线。

## 编码器里程

```c
g_encoder_left_forward_total
g_encoder_right_forward_total
g_encoder_avg_forward_total
g_encoder_diff_forward_total
```

方向约定：

```text
左轮前进量 = -g_left_count
右轮前进量 =  g_right_count
平均前进量 = (左 + 右) / 2
左右差值   = 左 - 右
```

这版先用于 Watch，后续可以用平均前进量触发 B/C/D/A。

## 第一轮测试

### 白底 A 点启动

期望：

```text
停顿 + 2 声
车直行桥接
碰到黑线后停顿 + 3 声
进入循迹
```

如果没有 3 声：

```text
红外阵列实际仍没采到黑线
```

重点查高度、位置、是否真的压线。

### 黑线启动

期望：

```text
停顿 + 4 声
直接循迹
```

如果仍直走：

重点看：

```c
g_line_raw_mask
g_line_error
g_line_correction
g_left_pwm_ticks
g_right_pwm_ticks
```

如果 `g_line_error/g_line_correction` 在变，但车不转，说明电机 PWM/方向/扭矩参数要调。

## 当前关键 PWM

```c
PWM_LEFT_BASE_TICKS    520
PWM_RIGHT_BASE_TICKS   500
PWM_BRIDGE_LEFT_TICKS  500
PWM_BRIDGE_RIGHT_TICKS 520
PWM_MIN_TICKS          330
PWM_MAX_TICKS          760
```

如果太快，优先整体降低这些 PWM。

如果能识别线但转不动，优先提高 `PWM_MAX_TICKS` 和 `LINE_SHARP_CORR_LIMIT`。
