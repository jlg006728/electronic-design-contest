# 20260707_162256 V13 - 主动预偏转修正版

## 文件

- 当前 CCS 活动文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_162256_scaled_5over7_left_until_cd_mid_active_preaim_v13.c`

## 修正的问题

V12 的宏配置看起来是“左偏保持到 CD 中点”，但主循环里只调用了：

```c
course_preaim_target_x_us();
```

这只会更新 `g_course_preaim_x_us`，不会主动改变 `g_servo_x_us`。

真正让舵机动的旧逻辑在 `aim_search_for_lost_target()` 里，因此 V12 在 AB 段可能没有左偏。

V13 抽出了旧版的步进逻辑：

```c
course_preaim_step_to_target()
```

并在主循环里增加：

```c
course_preaim_update_when_unlocked();
```

行为：

- OpenMV 尚未稳定锁定目标时，舵机会每帧主动向课程预瞄角移动。
- OpenMV 稳定锁定目标后，视觉闭环接管，避免预偏转和视觉控制互相拉扯。
- 点位停车时仍使用强制点位角。

## 当前关键参数

```c
#define COURSE_LAP_SAMPLES          914U
#define COURSE_LEFT_HOLD_SAMPLE     559U
#define COURSE_LEFT_TARGET_US       2550U
#define SERVO_X_MIN_US              350U
#define SERVO_X_MAX_US              2650U
```

`COURSE_LEFT_HOLD_SAMPLE=559`：左偏保持到 CD 中点。

## 验证

- CCS `gmake all` 编译通过。
- 未自动烧录。
