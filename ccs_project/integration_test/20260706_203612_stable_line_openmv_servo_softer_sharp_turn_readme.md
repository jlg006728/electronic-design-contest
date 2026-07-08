# 20260706_203612 softer sharp turn

## 目的

用户反馈：小车偏离黑线向右后，朝左边的修正太过度。

复查当前代码后，左右基础 PWM 和普通修正都是对称的；更可能的问题是当前低速档下急弯修正过猛：

- 基础速度约 `403`
- 原急弯修正上限 `380`
- 一侧轮可能被压到接近停转，导致回拉过头

## 文件

当前 CCS 活动文件：

`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`

时间戳源码：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_203612_stable_line_openmv_servo_softer_sharp_turn.c`

备份：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_203612_before_softer_sharp_turn_empty.c`

## 改动

只改急弯模式：

- `LINE_SHARP_ERROR 1800 -> 2200`
- `LINE_SHARP_KP_DIVISOR 8 -> 10`
- `LINE_SHARP_CORR_LIMIT 380 -> 260`

含义：

- 中等偏差先继续用普通循迹，不要过早急弯。
- 真正大偏差才进急弯。
- 急弯时一侧轮不会被压得太低，减少朝左回拉过头。

## 保留

- `PWM_LEFT_BASE_TICKS = 403`
- `PWM_RIGHT_BASE_TICKS = 403`
- `LINE_KP_DIVISOR = 20`
- `LINE_CORR_LIMIT = 150`
- `LINE_CENTER_OFFSET = 0`
- OpenMV/舵机逻辑

## 现场判断

- 如果右偏后回正不再过头：保留。
- 如果直角弯过不去：优先把 `LINE_SHARP_CORR_LIMIT 260 -> 300`，不要先改普通循迹。
- 如果仍回正过猛：继续把 `LINE_SHARP_CORR_LIMIT 260 -> 220`。

本次只编译验证，未自动烧录。
