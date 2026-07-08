# 20260707_152852 关键点停车强制校正 V7

## 文件

- 当前 CCS 活动文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_152852_course_point_stop_forced_aim_v7.c`
- OpenMV 脚本保持：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260707_144221_openmv_uart_stream_qvga_far_stable_v2.py`

## 这版解决什么

V6 只有连续预偏转，太软。V7 增加 ABCD 关键点强制修正：

- 到 B 点停车 2s，云台强制打到左偏位置。
- 到 C 点停车 2s，云台强制打到按几何回正曲线计算出的 C 点位置。
- 到 D 点停车 2s，云台强制打到按几何回正曲线计算出的 D 点位置。
- 到下一次 A 点停车 2s，云台强制回 HOME。
- 停车期间 OpenMV 仍然运行；如果识别到目标，视觉闭环可继续微调。
- 停车期间 `g_run_sample` 不增加，点位进度不会被 2s 停车时间冲乱。

上电起始 A 点的 HOME 保持时间也改为 2s：

```c
#define SERVO_STARTUP_HOME_FRAMES   100U
```

## 当前关键参数

```c
#define MISSION_ENABLE_POINT_STOPS  1U
#define COURSE_POINT_HOLD_FRAMES    100U

#define COURSE_LAP_SAMPLES          1280U
#define COURSE_POINT_B_SAMPLE       284U
#define COURSE_BC_MID_SAMPLE        462U
#define COURSE_POINT_C_SAMPLE       640U
#define COURSE_POINT_D_SAMPLE       924U
#define COURSE_POINT_A_SAMPLE       COURSE_LAP_SAMPLES

#define COURSE_LEFT_TARGET_US       2550U
#define COURSE_PREAIM_STEP_US       35
```

`COURSE_POINT_HOLD_FRAMES=100` 表示约 2s，因为舵机软件 PWM 一帧约 20ms。

## 点位 sample 来源

按图估算：

- AB 直线：`100cm`
- 半圆半径：`40cm`
- 一圈约：`2*100 + 2*pi*40 = 451.3cm`

换算到 `COURSE_LAP_SAMPLES=1280`：

- B 点：`100/451.3*1280 ≈ 284`
- BC 中点：`(100 + pi*40/2)/451.3*1280 ≈ 462`
- C 点：`(100 + pi*40)/451.3*1280 ≈ 640`
- D 点：`(200 + pi*40)/451.3*1280 ≈ 924`
- 下一次 A 点：`1280`

## Watch 建议

```c
g_phase
g_run_sample
g_point_stop_next_index
g_point_hold_id
g_point_hold_frames_left
g_point_stop_target_sample
g_point_forced_x_us
g_course_preaim_x_us
g_servo_x_us
g_omv_target_detected
g_aim_state
```

`g_phase = 7` 表示关键点停车。

`g_point_hold_id`：

- `1` = A
- `2` = B
- `3` = C
- `4` = D

## 现场调参

如果停车点整体来得太早或太晚，优先调：

```c
COURSE_LAP_SAMPLES
```

如果只有某一个点早/晚，单独调：

```c
COURSE_POINT_B_SAMPLE
COURSE_POINT_C_SAMPLE
COURSE_POINT_D_SAMPLE
COURSE_POINT_A_SAMPLE
```

如果 B 点左转幅度仍不够：

```c
COURSE_LEFT_TARGET_US 2550U -> 2600U
```

如果 B 点扫得过头：

```c
COURSE_LEFT_TARGET_US 2550U -> 2480U
```

## 关于电机行驶距离

当前代码仍然没有使用编码器闭环距离。所有点位都是用 `g_run_sample` 估计距离，本质是时间估算，不是真实里程。

所以电池电压升高、负重增加、轮胎打滑、速度参数变化，都会让 B/C/D/A 的实际停车位置漂移。

真正要把 ABCD 点停准，下一步应接入编码器计数，用轮子实际脉冲数替代 `g_run_sample`。V7 先保留当前硬件/代码结构，用显式点位 sample 做可调版本。

## 验证

- CCS `gmake all` 编译通过。
- 未自动烧录。
