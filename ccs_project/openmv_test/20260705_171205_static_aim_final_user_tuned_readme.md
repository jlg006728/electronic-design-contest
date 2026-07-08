# 20260705_171205 OpenMV 双舵机静态瞄准最终调参版

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260705_171205_mspm0_openmv_static_aim_final_user_tuned.c`
- OpenMV 推荐脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260705_145746_openmv_uart_stream_watchdog_no_print_rot180.py`

## 现场最终调定参数

```c
#define AIM_X_PULSE_SIGN            (-1)
#define AIM_Y_PULSE_SIGN            (-1)

#define AIM_DEAD_X_PIXELS           20
#define AIM_DEAD_Y_PIXELS           16
#define AIM_X_DIVISOR               3
#define AIM_Y_DIVISOR               3
#define AIM_MAX_STEP_US             45
#define AIM_MIN_PIXELS              30U
#define AIM_FILTER_DIVISOR          2
#define AIM_TARGET_CONFIRM_FRAMES   2U

#define SERVO_X_HOME_US             1600U
#define SERVO_Y_HOME_US             1500U
#define SERVO_X_MIN_US              800U
#define SERVO_X_MAX_US              2200U
#define SERVO_Y_MIN_US              500U
#define SERVO_Y_MAX_US              2500U
```

## 行为

- 电机禁用，`PB4/STBY` 保持低电平。
- 上电后云台先回 HOME。
- OpenMV 通过 UART0 回传红色目标坐标。
- PA14 水平舵机、PA17 俯仰舵机跟踪红色目标。
- 丢失目标后保持最后舵机位置，不再慢慢自动回 HOME。

## 验证

CCS `gmake all` 编译通过。按用户要求，未自动烧录。

