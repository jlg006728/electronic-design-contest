# 20260707_160803 V11 - 5/7 非编码器，左偏保持到 CD 中点

## 文件

- 当前 CCS 活动文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_160803_scaled_5over7_left_until_cd_mid_servo_wide_v11.c`

## 核心改动

仍然使用 V9 的 5/7 非编码器距离版本，不启用编码器。

预偏转策略改为：

- A -> B -> C -> CD 中点：持续左偏。
- CD 中点之后：开始向右回正。
- 到 A 点前：回到正前方。

当前 5/7 点位：

```c
#define COURSE_LAP_SAMPLES          914U
#define COURSE_POINT_B_SAMPLE       203U
#define COURSE_POINT_C_SAMPLE       457U
#define COURSE_POINT_D_SAMPLE       660U
#define COURSE_LEFT_HOLD_SAMPLE     559U
```

`COURSE_LEFT_HOLD_SAMPLE=559` 来自：

```text
(C点457 + D点660) / 2 ≈ 559
```

## 舵机范围

水平舵机软件限幅放宽为：

```c
#define SERVO_X_MIN_US              300U
#define SERVO_X_MAX_US              2800U
#define COURSE_LEFT_TARGET_US       2750U
```

不要把 `SERVO_X_MIN_US` 改成 `0U`。`0us` 不是有效舵机角度，而是非法/丢信号级别的脉宽，容易导致舵机持续转、抖动或失控。

如果 `2800us` 出现顶死、抖动、发热或持续嗡鸣，应立刻降回：

```c
#define SERVO_X_MAX_US              2650U
#define COURSE_LEFT_TARGET_US       2550U
```

## 验证

- CCS `gmake all` 编译通过。
- 未自动烧录。
