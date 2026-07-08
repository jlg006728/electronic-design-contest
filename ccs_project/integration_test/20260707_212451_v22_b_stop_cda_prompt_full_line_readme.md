# V22：全程有线赛道，B 停车，C/D/A 只声光提示

时间：2026-07-07 21:24

## 文件

- 当前 CCS 活动源码：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_212451_v22_b_stop_cda_prompt_full_line.c`

## 基底

本版从 V19 恢复并修改：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_190540_v19_restore_dynamic_tracking_with_buzzer.c`

## 假设

先假定赛道全程都有黑线，不考虑中途断线/桥接。

## 任务行为

- B 点：停车，进入 `PHASE_POINT_HOLD`，保留瞄准/蜂鸣逻辑。
- C 点：不停车，只闪灯并蜂鸣 4 声。
- D 点：不停车，只闪灯并蜂鸣 5 声。
- A 点：不停车，只闪灯并蜂鸣 1 声。

## 关键改动

打开提示逻辑：

```c
#define MISSION_ENABLE_POINT_PROMPTS 1U
```

C/D/A 提示点改为当前 COURSE 点位：

```c
#define MISSION_C_PROMPT_SAMPLE     COURSE_POINT_C_SAMPLE
#define MISSION_D_PROMPT_SAMPLE     COURSE_POINT_D_SAMPLE
#define MISSION_A_FINISH_SAMPLE     COURSE_POINT_A_SAMPLE
```

停车状态机只允许 B 点进入：

```c
if (g_point_stop_next_index != 0U) {
    return false;
}
```

B 点停车结束后：

```c
g_mission_b_done = 1U;
g_point_stop_next_index = 1U;
```

这样 C/D/A 不再进入停车状态。

## 验证

已执行 clean build 并编译通过。按当前规则：不自动烧录，由用户手动烧录。
