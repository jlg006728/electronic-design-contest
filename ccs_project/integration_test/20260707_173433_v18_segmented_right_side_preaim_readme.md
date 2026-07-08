# V18：分段右侧预瞄曲线

时间：2026-07-07 17:34

## 文件

- 当前 CCS 活动源码：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_173433_v18_segmented_right_side_preaim.c`
- 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_173433_before_v18_current_restore_v16_backup.c`

## 基底

本版基于回退后的 V16 安全边界：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_172422_restore_v16_for_servo_strategy_review.c`

## 设计目的

用户提出的新策略：

- 到 B 点时，云台在左侧约 180 度。
- B 点停车结束、车继续走后，云台快速向右转到接近右侧 180 度。
- B 后到 CD 中点，云台慢慢向左恢复，到 CD 中点时约右侧 90 度。
- CD 中点到 D 点，云台继续向右，接近右侧 180 度。
- D 到 A，云台向左回正，到 A 点回到 0 度/HOME。

这个策略不是“软件重置舵机角度”，而是按赛道进度给 OpenMV 一个分段预瞄方向。

## 新增/修改参数

```c
#define COURSE_PREAIM_FAST_RIGHT    2U
#define COURSE_PREAIM_RIGHT_DEEPEN  3U
#define COURSE_PREAIM_RETURN_HOME   4U

#define COURSE_RIGHT_NEAR_TARGET_US 700U
#define COURSE_RIGHT_MID_TARGET_US  1100U
```

保留安全边界：

```c
#define SERVO_X_MIN_US              350U
#define SERVO_X_MAX_US              2650U
#define COURSE_LEFT_TARGET_US       2550U
```

## 分段曲线

当前 sample 点：

```c
B = 162
CD 中点 = 447
D = 528
A = 731
```

预瞄目标：

- `0 ~ B`：左侧目标 `2550us`。
- `B+1 ~ CD中点`：从右接近 180 度 `700us` 线性回到右 90 度 `1100us`。
- `CD中点 ~ D`：从右 90 度 `1100us` 线性转回右接近 180 度 `700us`。
- `D ~ A`：从右接近 180 度 `700us` 线性回 HOME `1600us`。

## 关键修正

原来的 `clamp_course_preaim_x()` 只允许 HOME 到 LEFT 范围，会把右侧目标夹回 HOME。V18 已改为允许 `COURSE_RIGHT_NEAR_TARGET_US ~ COURSE_LEFT_TARGET_US`，所以右侧预瞄会真正生效。

## 验证

CCS `gmake all` 编译通过。按当前规则：不自动烧录，由用户手动烧录。
