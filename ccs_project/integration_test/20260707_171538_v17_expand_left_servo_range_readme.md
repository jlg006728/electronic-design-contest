# V17：扩大水平舵机左偏软件范围

时间：2026-07-07 17:15

## 文件

- 当前 CCS 活动源码：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_171538_v17_expand_left_servo_range.c`
- 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_171538_before_v17_current_v16_backup.c`

## 基底

本版基于 V16：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_171101_v16_faster_right_return_no_left_limit_push.c`

## 现场纠正

用户确认：水平舵机本体支持 270 度，之前左转不到 270 度不是机械本体不支持，而是代码给的最大偏转范围限制了。

## 改动

扩大水平舵机左偏方向的软件上限：

```c
#define SERVO_X_MAX_US              3050U
#define COURSE_LEFT_TARGET_US       3000U
```

保持不变：

```c
#define SERVO_X_HOME_US             1600U
#define SERVO_X_MIN_US              350U
#define COURSE_PREAIM_STEP_US       35
#define COURSE_PREAIM_RETURN_STEP_US 140
```

说明：

- 左偏目标从 `2550us` 提高到 `3000us`，解除 V16/V15 的左偏截断。
- 最大限幅从 `2650us` 提高到 `3050us`，给左偏目标留少量余量。
- 下限不改，避免再次接近之前 `0us` 导致舵机乱转的问题。
- 右回速度继续沿用 V16 的 `140us/frame`。

## 验证

CCS `gmake all` 编译通过。按当前规则：不自动烧录，由用户手动烧录。
