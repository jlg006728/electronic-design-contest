# 20260704_162850 OpenMV 掉电诊断版：水平舵机慢扫

## 文件

- 当前烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_162850_mspm0_openmv_diag_yaw_slow_sweep.c`
- OpenMV 脚本继续使用：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_160413_openmv_uart_stream_smoother_fast_rot180.py`

## 背景

冻结水平舵机版本没有断电，说明 OpenMV 自身、UART、俯仰舵机基本不是主因；断电与水平舵机运动强相关。

本版用于进一步判断：水平舵机只要慢速运动，是否也会导致 OpenMV 掉电。

## 代码状态

```c
#define DIAG_FREEZE_YAW_SERVO 0U
#define DIAG_YAW_SWEEP_MODE   1U
```

水平舵机在 `1300..1700us` 之间慢速扫描：

```c
#define DIAG_YAW_SWEEP_MIN_US  1300U
#define DIAG_YAW_SWEEP_MAX_US  1700U
#define DIAG_YAW_SWEEP_STEP_US 3U
```

俯仰舵机锁定 HOME，OpenMV 继续运行，但视觉识别结果不参与控制。

## 判据

- 如果慢扫时仍然断电：问题基本在水平舵机运动链路，包括舵机本体、云台机构卡滞、线束拉扯、舵机电源/GND 回路、舵机噪声干扰 OpenMV。
- 如果慢扫时不再断电：水平舵机本身可动，问题更可能来自视觉闭环追踪过快/频繁反向/持续顶某个方向，下一步做“限速限幅正式追踪版”。

## Watch

- `g_diag_yaw_sweep_turn_count`
- `g_diag_yaw_sweep_direction`
- `g_servo_x_goal_us`
- `g_servo_x_us`
- `g_omv_frame_count`
- `g_omv_timeout_frames`
