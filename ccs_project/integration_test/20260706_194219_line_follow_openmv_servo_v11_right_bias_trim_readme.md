# 20260706_194219 V11 right bias trim

## 目的

V10 降速后，用户反馈小车有一点固定偏右。本版只微调左右基础 PWM，不修改视觉、舵机、循迹中心和急弯策略。

## 文件

当前 CCS 活动文件：

`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`

时间戳源码：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_194219_line_follow_openmv_servo_v11_right_bias_trim.c`

备份：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_194219_before_v11_right_bias_empty.c`

## 改动

- `PWM_LEFT_BASE_TICKS 605 -> 595`
- `PWM_RIGHT_BASE_TICKS 605 -> 615`
- `PWM_LEFT_BOOST_TICKS 605 -> 595`
- `PWM_RIGHT_BOOST_TICKS 605 -> 615`

平均速度仍为 605，只改变左右平衡。

## 继续微调

- 如果仍偏右：继续左减右增，例如 `590/620`。
- 如果变成偏左：回退一点，例如 `600/610`。
- 如果只在急弯后偏，先不要改这里，应单独看急弯恢复逻辑。

本次只编译验证，未自动烧录。
