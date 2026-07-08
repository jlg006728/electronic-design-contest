# 20260706_204209 revert to open loop balance fix

## 目的

用户要求回退。已撤销上一版 `20260706_203612_stable_line_openmv_servo_softer_sharp_turn.c` 的急弯温和化改动。

## 恢复来源

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_201926_stable_line_openmv_servo_open_loop_balance_fix.c`

## 当前文件

当前 CCS 活动文件：

`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`

回退后时间戳源码：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_204209_revert_to_open_loop_balance_fix.c`

回退前备份：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_204209_before_revert_softer_sharp_turn_empty.c`

## 恢复后的关键参数

```c
PWM_LEFT_BASE_TICKS      403
PWM_RIGHT_BASE_TICKS     403
PWM_SHARP_LEFT_BASE_TICKS 403
PWM_SHARP_RIGHT_BASE_TICKS 403

LINE_SHARP_ERROR         1800
LINE_SHARP_KP_DIVISOR    8
LINE_SHARP_CORR_LIMIT    380
```

本次只编译验证，未自动烧录。
