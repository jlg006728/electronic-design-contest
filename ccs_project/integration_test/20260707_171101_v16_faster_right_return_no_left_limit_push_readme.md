# V16：右回速度继续加快，不再硬推左偏极限

时间：2026-07-07 17:11

## 文件

- 当前 CCS 活动源码：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_171101_v16_faster_right_return_no_left_limit_push.c`
- 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_171101_before_v16_current_v15_backup.c`

## 基底

本版基于 V15：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_165804_v15_shorter_distance_fast_return.c`

## 现场反馈

- 舵机向左始终只能到约 180 度，没有达到 270 度，并且表现为卡住。
- 向右回正速度仍然不够快。

## 判断

左转卡住属于机械/舵机有效行程限制，不应继续靠增大脉宽硬顶。继续增大 `COURSE_LEFT_TARGET_US` 或 `SERVO_X_MAX_US` 可能导致舵机堵转、发热、电源压降，甚至影响 OpenMV 和主控。

因此 V16 不扩大左偏极限，只加快向右回正。

## 改动

```c
#define COURSE_PREAIM_RETURN_STEP_US 140
```

V15 为 `70`，V16 提到 `140`。

保持不变：

```c
#define COURSE_PREAIM_STEP_US       35
#define COURSE_LEFT_TARGET_US       2550U
#define SERVO_X_MIN_US              350U
#define SERVO_X_MAX_US              2650U
```

含义：

- AB 左偏速度仍然是 `35us/frame`。
- CD 中点后和 DA 段向右回正速度为 `140us/frame`。
- 不再通过软件强行追求左转 270 度。

## 后续如果仍需要 270 度

需要从机械和器件侧处理：

- 重新安装水平舵机摆臂，让目标角度落在舵机有效机械范围内。
- 确认舵机本体是否真支持 270 度。
- 若本体只支持 180 度，必须更换 270 度舵机或改变传动结构，代码不能把 180 度舵机变成 270 度。

## 验证

CCS `gmake all` 编译通过。按当前规则：不自动烧录，由用户手动烧录。
