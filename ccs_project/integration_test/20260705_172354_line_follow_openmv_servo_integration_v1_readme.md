# 20260705_172354 循迹 + OpenMV 双舵机合并 V1

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260705_172354_line_follow_openmv_servo_integration_v1.c`
- OpenMV 推荐脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260705_145746_openmv_uart_stream_watchdog_no_print_rot180.py`

## 合并内容

保留已验证的循迹参数：

```c
#define PWM_LEFT_BASE_TICKS    620U
#define PWM_RIGHT_BASE_TICKS   590U
#define LINE_KP_DIVISOR        20
#define LINE_CORR_LIMIT        150
#define LINE_SHARP_ERROR       1800
#define LINE_SHARP_KP_DIVISOR  8
#define LINE_SHARP_CORR_LIMIT  380
```

保留用户最终调定的云台参数：

```c
#define SERVO_X_HOME_US        1600U
#define SERVO_Y_HOME_US        1500U
#define SERVO_X_MIN_US         800U
#define SERVO_X_MAX_US         2200U
#define SERVO_Y_MIN_US         500U
#define SERVO_Y_MAX_US         2500U
```

## 当前行为

- 上电后先闪灯并让 PA14/PA17 云台回 HOME。
- 随后电机启动，按红外循迹运行。
- OpenMV 通过 `PA10/PA11` UART 同时回传目标坐标。
- 云台在小车循迹过程中继续追踪红色目标。
- 运行约 90 秒后停车。
- 如果已经见过线后又连续丢线约 2 秒，则停车。
- 如果启动时暂时没看到线，先直走找线。

## Watch 建议

循迹：

- `g_phase`
- `g_stop_reason`
- `g_line_raw_mask`
- `g_line_error`
- `g_turn_mode`
- `g_left_pwm_ticks`
- `g_right_pwm_ticks`

OpenMV/云台：

- `g_omv_color_frame_count`
- `g_omv_target_detected`
- `g_omv_cx`
- `g_omv_cy`
- `g_servo_x_us`
- `g_servo_y_us`
- `g_aim_state`
- `g_diag_stage`

状态含义：

- `g_phase=2`：正在循迹运行
- `g_phase=3`：90 秒到时停车
- `g_phase=4`：丢线停车
- `g_phase=5`：舵机校准模式
- `g_stop_reason=1`：运行时间到
- `g_stop_reason=2`：丢线停车
- `g_turn_mode=0`：普通循迹
- `g_turn_mode=1`：急弯
- `g_turn_mode=2`：丢线按最后方向找回
- `g_turn_mode=3`：启动或完全无线时先直走找线

## 首次测试建议

1. 先架空小车烧录运行，确认电机、舵机、OpenMV 都同时工作且不掉电。
2. 再放到赛道上短距离测试，红色目标先固定不动。
3. 如果循迹明显变差，优先怀疑软件舵机 PWM 的 20ms 阻塞影响循迹节奏，下一版应把舵机 PWM 改成硬件定时器或降低云台更新频率。

## 验证

CCS `gmake all` 编译通过。按用户要求，未自动烧录。

