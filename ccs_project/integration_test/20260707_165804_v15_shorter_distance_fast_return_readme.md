# V15：点位距离 4/5 + 右回速度翻倍

时间：2026-07-07 16:58

## 文件

- 当前 CCS 活动源码：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_165804_v15_shorter_distance_fast_return.c`
- 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_165804_before_v15_current_v14_backup.c`

## 基底

本版基于 V14：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_164523_v9_trigger_left_until_cd_mid_v14.c`

## 修改目的

现场反馈：

- A 到 B 段舵机左偏能力良好，需要保留。
- BC 段继续左偏已经受到舵机/机械范围限制，继续增加左偏意义不大。
- CD 中点以及 DA 段向右回正速度需要变为 2 倍。
- ABCD 点位距离整体需要变为当前的 `4/5`。

## 关键改动

距离点位按当前 V14 乘 `4/5`：

```c
#define COURSE_LAP_SAMPLES          731U
#define COURSE_LEFT_HOLD_SAMPLE     447U
#define COURSE_POINT_B_SAMPLE       162U
#define COURSE_POINT_C_SAMPLE       366U
#define COURSE_POINT_D_SAMPLE       528U
#define COURSE_POINT_A_SAMPLE       COURSE_LAP_SAMPLES
```

旧任务提示点也同步缩放：

```c
#define MISSION_B_STOP_SAMPLE       144U
#define MISSION_C_PROMPT_SAMPLE     288U
#define MISSION_D_PROMPT_SAMPLE     432U
#define MISSION_A_FINISH_SAMPLE     576U
```

舵机预偏转速度分离：

```c
#define COURSE_PREAIM_STEP_US        35
#define COURSE_PREAIM_RETURN_STEP_US 70
```

- 向左偏转：仍然每帧最多 `35us`，保留 AB 段手感。
- 向右回正：每帧最多 `70us`，用于 CD 中点后和 DA 段更快回正。

## 保持不变

- 不扩大 `COURSE_LEFT_TARGET_US`，避免继续撞舵机/机械限制。
- 不引入 V13 的 `course_preaim_update_when_unlocked()`。
- 仍沿用 V9/V14 的无目标触发预偏转方式。

## 验证

CCS `gmake all` 编译通过。按当前规则：不自动烧录，由用户手动烧录。
