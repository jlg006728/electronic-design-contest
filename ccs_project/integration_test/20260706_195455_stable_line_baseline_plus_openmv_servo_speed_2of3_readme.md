# 20260706_195455 stable baseline plus OpenMV servo, speed 2/3

## 目的

在 `20260706_195125_stable_line_baseline_plus_openmv_servo.c` 的基础上，将电机速度降低为原来的 `2/3`。

## 文件

当前 CCS 活动文件：

`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`

时间戳源码：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_195455_stable_line_baseline_plus_openmv_servo_speed_2of3.c`

备份：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_195455_before_baseline_openmv_speed_2of3_empty.c`

## 改动

- `PWM_LEFT_BASE_TICKS 620 -> 413`
- `PWM_RIGHT_BASE_TICKS 590 -> 393`
- `PWM_LEFT_BOOST_TICKS 620 -> 413`
- `PWM_RIGHT_BOOST_TICKS 590 -> 393`
- `PWM_MIN_TICKS 430 -> 287`
- `PWM_MAX_TICKS 850 -> 567`
- `PWM_SHARP_LEFT_BASE_TICKS 620 -> 413`
- `PWM_SHARP_RIGHT_BASE_TICKS 590 -> 393`
- `PWM_SHARP_MAX_TICKS 850 -> 567`

保留：

- 稳定 baseline 循迹参数：`LINE_KP_DIVISOR=20`、`LINE_CORR_LIMIT=150`、急弯 `1800/8/380`
- `LINE_CENTER_OFFSET=0`
- OpenMV/舵机逻辑
- V9 丢目标搜索与边缘保护

## 现场判断

- 如果直线稳定但直角弯变弱：只提高 `PWM_SHARP_*`，不要先提高普通直线速度。
- 如果地面起步无力：加短启动助推，而不是整体恢复高速。

本次只编译验证，未自动烧录。
