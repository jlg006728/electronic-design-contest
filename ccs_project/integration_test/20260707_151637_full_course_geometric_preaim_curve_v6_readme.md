# 20260707_151637 全程几何预偏转 V6

## 文件

- 当前 CCS 活动文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_151637_full_course_geometric_preaim_curve_v6.c`
- OpenMV 脚本保持：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260707_144221_openmv_uart_stream_qvga_far_stable_v2.py`

## 核心逻辑

这版不再使用 AB/BC/CD/DA 离散跳变预偏转。

云台水平预偏转按一整圈连续变化：

- 从 A 点出发后，云台向左偏转。
- A 到 B，再到 BC 中点之前，保持左偏目标。
- 过 BC 中点后，云台目标开始线性向右回正。
- 到下一次 A 点前，目标回到正前方 HOME。

OpenMV 和预偏转的关系：

- OpenMV 稳定识别到红色目标时，舵机按图像误差闭环追踪。
- OpenMV 没识别到目标或目标不可用时，舵机按赛道几何预偏转继续找目标。
- 没有把 OpenMV 修正量和预偏转量硬相加，避免两个控制量互相打架。

## 当前关键参数

```c
#define COURSE_PREAIM_ENABLE        1U
#define COURSE_LAP_SAMPLES          1280U
#define COURSE_BC_MID_SAMPLE        462U
#define COURSE_LEFT_TARGET_US       2550U
#define COURSE_HOME_TARGET_US       SERVO_X_HOME_US
#define COURSE_PREAIM_STEP_US       35
```

`COURSE_BC_MID_SAMPLE = 462` 的来源：

- 图中 AB 长度约 `100cm`
- 半圆半径约 `40cm`
- A 到 BC 中点距离约 `100 + pi*40/2 = 162.8cm`
- 一圈长度约 `2*100 + 2*pi*40 = 451.3cm`
- 比例约 `36.1%`
- `1280 * 36.1% = 462`

## Watch 建议

```c
g_course_lap_sample
g_course_preaim_phase
g_course_preaim_x_us
g_course_aim_offset_us
g_servo_x_us
g_servo_y_us
g_omv_target_detected
g_aim_state
```

`g_course_preaim_phase`：

- `1`：A 到 BC 中点，左偏保持
- `2`：BC 中点到下一次 A，逐渐回正

## 现场调参

如果 B 点或刚进 BC 时还看不到靶：

```c
COURSE_LEFT_TARGET_US 2550U -> 2600U
```

如果云台左偏太大、扫过目标：

```c
COURSE_LEFT_TARGET_US 2550U -> 2480U
```

如果开始回正太早：

```c
COURSE_BC_MID_SAMPLE 462U -> 500U
```

如果开始回正太晚：

```c
COURSE_BC_MID_SAMPLE 462U -> 430U
```

如果一圈实际时间和程序不同，先调：

```c
COURSE_LAP_SAMPLES
```

当前仍然是用 `g_run_sample` 估计圈进度，不是编码器闭环距离。因此速度变化明显时，`COURSE_LAP_SAMPLES` 也要跟着调。

## 验证

- CCS `gmake all` 编译通过。
- 按规则未自动烧录。
