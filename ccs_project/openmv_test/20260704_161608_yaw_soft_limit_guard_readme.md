# 20260704_161608 水平舵机软限位保护版

## 文件

- 当前烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_161608_mspm0_openmv_yaw_soft_limit_guard.c`
- OpenMV 脚本继续使用：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_160413_openmv_uart_stream_smoother_fast_rot180.py`

## 背景

用户反馈：OpenMV 掉电不是随机发生，而是常在水平舵机持续向左/右追踪、距离初始位置不到 90 度时出现。舵机会先一卡一卡，然后 OpenMV 停止/关机。

判断：这不像单纯静态 5V 不够，更像水平舵机被闭环推到某个高负载/内部电气边界区域，产生电流尖峰或地线噪声，导致 OpenMV 复位。

## 改动

- 水平轴运行范围从 `900..2100us` 收窄到 `1050..1950us`。
- 新增水平边界保护：
  - `SERVO_X_LIMIT_MARGIN_US = 35`
  - `SERVO_X_LIMIT_BACKOFF_US = 70`
- 如果 `g_servo_x_goal_us` 已经接近水平安全边界，并且算法还继续往外推：
  - 不再硬顶边界
  - 目标位置自动退回一点
  - `g_tracking_stop_reason = 4`
  - `g_servo_x_limit_guard_count++`

## Watch

- `g_servo_x_goal_us`
- `g_servo_x_us`
- `g_tracking_stop_reason`
- `g_servo_x_limit_guard_count`
- `g_omv_frame_count`
- `g_omv_color_frame_count`
- `g_omv_timeout_frames`

## 结果判定

- 如果这版不再导致 OpenMV 关机：之前问题主要是水平舵机被推入危险工作区。
- 如果这版仍会关机：重点检查地线走法和舵机瞬态干扰，尤其不要让舵机电流回路经过 OpenMV/MSPM0 的细 GND 线。
