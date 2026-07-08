# 20260706_200241 speed 2/3 right bias trim

## 目的

用户反馈当前 `2/3` 速度版仍有一点默认偏右。本版只减小左右基础 PWM 差值，不改变整体速度。

## 文件

当前 CCS 活动文件：

`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`

时间戳源码：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_200241_stable_line_openmv_servo_speed_2of3_right_bias_trim.c`

备份：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_200241_before_right_bias_trim_empty.c`

## 改动

- `PWM_LEFT_BASE_TICKS 413 -> 408`
- `PWM_RIGHT_BASE_TICKS 393 -> 398`
- `PWM_LEFT_BOOST_TICKS 413 -> 408`
- `PWM_RIGHT_BOOST_TICKS 393 -> 398`
- `PWM_SHARP_LEFT_BASE_TICKS 413 -> 408`
- `PWM_SHARP_RIGHT_BASE_TICKS 393 -> 398`

平均速度仍为 `403`，只把左右差值从 `20` 降到 `10`。

## 继续微调

- 如果仍偏右：下一步改成 `403/403`。
- 如果变偏左：回到 `410/396` 左右。

本次只编译验证，未自动烧录。
