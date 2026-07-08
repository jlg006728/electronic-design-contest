# 20260706_194607 V12 slow 3/5

## 目的

用户要求车速“减慢 2/5”。本版按字面执行：保留当前 V11 速度的 `3/5`，即整体 PWM 降低 40%。

## 文件

当前 CCS 活动文件：

`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`

时间戳源码：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_194607_line_follow_openmv_servo_v12_slow_3of5.c`

备份：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_194607_before_v12_slow_3of5_empty.c`

## 改动

从 V11：

- `PWM_LEFT_BASE_TICKS 595 -> 357`
- `PWM_RIGHT_BASE_TICKS 615 -> 369`
- `PWM_LEFT_BOOST_TICKS 595 -> 357`
- `PWM_RIGHT_BOOST_TICKS 615 -> 369`
- `PWM_MIN_TICKS 430 -> 258`
- `PWM_MAX_TICKS 850 -> 510`
- `PWM_SHARP_LEFT_BASE_TICKS 605 -> 363`
- `PWM_SHARP_RIGHT_BASE_TICKS 605 -> 363`
- `PWM_SHARP_MAX_TICKS 850 -> 510`

保留：

- V9 OpenMV 丢目标搜索逻辑
- V11 左右偏右修正比例
- `LINE_CENTER_OFFSET = 500`
- 舵机参数

## 现场判断

- 如果地上起步不动：先把 `START_BOOST_SAMPLES` 从 `0` 改到 `8`，并把 `PWM_LEFT_BOOST_TICKS/PWM_RIGHT_BOOST_TICKS` 临时设为 `500/515`。
- 如果直线能走但直角弯过不去：只提高 `PWM_SHARP_LEFT_BASE_TICKS/PWM_SHARP_RIGHT_BASE_TICKS` 到 `420/420`。
- 如果速度仍快：再按 10% 一档降低，不要一次大跳。

本次只编译验证，未自动烧录。
