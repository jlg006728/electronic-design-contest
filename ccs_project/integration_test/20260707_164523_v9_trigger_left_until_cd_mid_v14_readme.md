# V14：V9 触发方式 + 左偏保持到 CD 中点

时间：2026-07-07 16:45

## 文件

- 当前 CCS 活动源码：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_164523_v9_trigger_left_until_cd_mid_v14.c`
- 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_164523_before_v14_current_empty_backup.c`

## 基底

本版从 V9 复制生成：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_153950_course_point_stop_buzzer_scaled_5over7_v9.c`

## 只做的改动

保留 V9 的预偏转触发方式，不引入 V13 的主循环主动预偏转。

将原本的：

```c
#define COURSE_BC_MID_SAMPLE        330U
```

改为：

```c
#define COURSE_LEFT_HOLD_SAMPLE     559U
```

并把 `course_preaim_phase_from_sample()`、`course_preaim_target_from_lap_sample()` 中的回正起点统一改为 `COURSE_LEFT_HOLD_SAMPLE`。

## 行为

- `0 ~ 559`：无可用目标时，云台水平目标保持左偏。
- `559 ~ 913`：无可用目标时，云台水平目标逐渐回正，到下一次 A 点附近回到 HOME。
- `COURSE_PREAIM_STEP_US` 保持 V9 的 `35`，也就是保留 V9 的 AB 段左偏速度。
- `SERVO_X_MIN_US`、`SERVO_X_MAX_US`、`COURSE_LEFT_TARGET_US` 不扩大，避免再次出现过转。

## 触发逻辑

V14 仍然沿用 V9：

```text
OpenMV 新帧无目标/目标不可用
  -> aim_handle_unusable_target()
  -> aim_search_for_lost_target()
  -> 按 course_preaim_target_x_us() 分帧移动 g_servo_x_us
```

所以它不是 V13 那种“只要未锁定就在主循环主动推舵机”的版本。

## 验证

CCS `gmake all` 编译通过。按当前规则：不自动烧录，由用户手动烧录。
