# 20260706_193630 V10 initial speed

## 目的

电池电压提高后，小车速度过快。本版只恢复早期速度档，不回退 OpenMV 丢目标搜索逻辑，也不修改循迹中心参数。

## 文件

当前 CCS 活动文件：

`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`

时间戳源码：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_193630_line_follow_openmv_servo_v10_initial_speed.c`

备份：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_193630_before_restore_initial_speed_empty.c`

## 改动

从高扭矩速度档：

- `PWM_LEFT_BASE_TICKS 705 -> 605`
- `PWM_RIGHT_BASE_TICKS 705 -> 605`
- `PWM_LEFT_BOOST_TICKS 880 -> 605`
- `PWM_RIGHT_BOOST_TICKS 880 -> 605`
- `PWM_MIN_TICKS 520 -> 430`
- `PWM_MAX_TICKS 1000 -> 850`
- `START_BOOST_SAMPLES 20 -> 0`

保留：

- `LINE_CENTER_OFFSET = 500`
- V9 OpenMV 丢目标搜索
- V9 舵机参数
- V8/V9 急弯策略

## 现场判断

- 如果速度合适但直线偏：再微调左右基础 PWM，不要先动 OpenMV。
- 如果直角弯变慢过不去：优先只提高 `PWM_SHARP_LEFT_BASE_TICKS/PWM_SHARP_RIGHT_BASE_TICKS` 或 `PWM_SHARP_MAX_TICKS`。
- 如果起步无力：先把 `START_BOOST_SAMPLES` 从 `0` 改到 `8`，不要直接恢复到 `20`。

本次只编译验证，未自动烧录。
