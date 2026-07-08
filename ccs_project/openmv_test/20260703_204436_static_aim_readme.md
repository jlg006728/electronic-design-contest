# 20260703 OpenMV + 双舵机静态瞄准测试

## 目的

验证：

```text
OpenMV 识别红色目标 -> MSPM0 接收 cx/cy -> PA14/PA17 云台自动追到画面中心
```

本版本不驱动电机，`PB4/STBY` 保持低电平。

## 需要先运行的 OpenMV 脚本

```text
C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_162934_openmv_full_selftest.py
```

OpenMV 端必须运行这个脚本，因为 MSPM0 会持续发送 `C` 命令，请求 OpenMV 回传颜色目标帧。

## MSPM0 端代码

```text
C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_204436_mspm0_openmv_servo_static_aim.c
```

当前活动工程：

```text
C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c
```

## 接线

```text
OpenMV P4 / UART3_TX -> MSPM0 PA11 / UART0_RX
OpenMV P5 / UART3_RX <- MSPM0 PA10 / UART0_TX
OpenMV GND           -> MSPM0 GND

PA14 -> 水平/左右舵机信号
PA17 -> 俯仰/上下舵机信号
舵机红线 -> 5V
舵机黑/棕线 -> 公共 GND
```

联调前拔掉 LaunchPad 的 `J21/J22` backchannel UART 跳线帽，避免 XDS110 干扰 PA10/PA11。

## Watch 变量

```text
g_omv_link_ok
g_omv_color_frame_count
g_omv_target_detected
g_omv_cx
g_omv_cy
g_aim_dx
g_aim_dy
g_aim_x_step_us
g_aim_y_step_us
g_servo_x_us
g_servo_y_us
g_aim_state
```

`g_aim_state`：

```text
0 = 串口未连接/超时，慢慢回中
1 = OpenMV 正常但没看到目标
2 = 正在追踪
3 = 已进入中心死区
```

## 如果方向反了

如果红色目标在画面右边，云台却往左追，改：

```c
#define AIM_X_PULSE_SIGN 1
```

把 `1` 改成 `(-1)`。

如果红色目标在画面下方，云台却往上追，改：

```c
#define AIM_Y_PULSE_SIGN (-1)
```

把 `(-1)` 改成 `1`。

## 第一版参数偏保守

舵机范围限制：

```text
1250us 到 1750us
```

每次最多改：

```text
8us
```

这样不容易突然撞机械极限。确认方向正确后，再加快追踪速度。
