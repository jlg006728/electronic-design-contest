# 20260708_210707 V26 A 点桥接任意传感器见线即循迹

## 基底

- 基于 V25：`20260708_205429_v25_fixed_course_start_gap_capture.c`。
- 当前 CCS 活动文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`。
- 本版只编译验证，未自动烧录。

## 为什么从 V25 简化

V25 的捕线锁中心逻辑原本用于防止“擦到黑线边缘就误切循迹”，但现场表现说明这一段不适合这么保守：

- 小车桥接后已经走到弯道入口。
- 只要没有满足中心锁定条件，它就继续桥接直走。
- 结果是看到了弯道线，但没有及时交给普通循迹，车继续直走冲过弯道。

因此 V26 改为更直接的策略：

```text
桥接阶段只要任意一个红外传感器识别到黑线，立即进入普通循迹。
```

## 关键改动

```c
#define START_GAP_REACQUIRE_FRAMES  1U
```

并将 `start_gap_bridge_update()` 中的重捕逻辑改为：

```text
g_line_valid != 0
-> 立即清桥接状态
-> line_follow_update()
-> g_phase = PHASE_LINE_RUN
-> 蜂鸣 3 声
```

V25 的 `PHASE_START_GAP_CAPTURE` 捕线函数仍保留在代码中，但桥接入口不再进入该状态。当前有效路径是：

```text
PHASE_START_GAP_BRIDGE -> PHASE_LINE_RUN
```

## 当前保留参数

桥接右漂修正仍保留：

```c
#define START_GAP_LEFT_TRIM_TICKS   -8
#define START_GAP_RIGHT_TRIM_TICKS  8
```

任务点仍关闭：

```c
#define START_GAP_TEST_ONLY         1U
```

## 蜂鸣提示

```text
1 声：程序启动
2 声：进入 A 点无线桥接
3 声：任意传感器看到黑线，已经切入普通循迹
长响：桥接超时仍未看到黑线
```

## 首测重点

1. A 点摆正起步。
2. 车大致直行桥接。
3. 到弯道入口时，只要任意红外碰到黑线，应立刻听到 3 声。
4. 听到 3 声后，观察是否能按普通循迹进入弯道。

## 调参规则

如果桥接仍向右偏：

```c
#define START_GAP_LEFT_TRIM_TICKS   -12
#define START_GAP_RIGHT_TRIM_TICKS  12
```

如果 3 声响了但仍进不了弯：

- 问题已经不是“重捕没切换”，而是普通循迹进入弯道时的速度/急弯参数问题。
- 下一步优先降低桥接末端速度或恢复急弯策略，不再增加重捕条件。

如果始终没有 3 声：

- 说明代码没有读到任何有效黑线。
- 检查 7 路红外供电、DO 输出和 `LINE_BLACK_IS_HIGH`。

## Watch 变量

```c
g_phase
g_turn_mode
g_line_valid
g_line_raw_mask
g_line_error
g_start_gap_bridge_frames
g_start_gap_reacquire_count
g_start_gap_reacquired_event_count
g_left_pwm_ticks
g_right_pwm_ticks
```

状态含义：

```text
g_phase = 8：A 点无线桥接
g_phase = 2：普通循迹
g_turn_mode = 4：桥接直行
```
