# 20260704_160413 OpenMV 双舵机平滑快速追踪版

## 文件

- MSPM0 当前烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- MSPM0 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_160413_mspm0_openmv_smooth_fast_track_hold_loss.c`
- OpenMV 新脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_160413_openmv_uart_stream_smoother_fast_rot180.py`

## 这版解决的问题

- OpenMV 串流周期从 `50ms` 改为 `25ms`，目标运动稍快时坐标更新更及时。
- MSPM0 新增 `g_servo_x_goal_us/g_servo_y_goal_us`，视觉算法只更新目标脉宽。
- 实际输出 `g_servo_x_us/g_servo_y_us` 每个 20ms 舵机周期按 `SERVO_SLEW_STEP_US=32us` 平滑追向目标，减少一卡一卡的阶跃感。
- 短时丢目标时进入 `g_aim_state=8`，沿最后追踪方向做有限预测；长时间丢目标保持当前位置，不再自动回中。
- `g_tracking_stop_reason` 用于判断为什么不继续追：
  - `0` 正常追踪
  - `1` MSPM0 超时未收到 OpenMV 帧
  - `2` OpenMV 正常发帧但没有检测到目标
  - `3` 检测到目标太小，被过滤

## Watch 建议

- `g_omv_frame_count`
- `g_omv_color_frame_count`
- `g_omv_timeout_frames`
- `g_tracking_stop_reason`
- `g_aim_state`
- `g_omv_cx`
- `g_omv_cy`
- `g_servo_x_goal_us`
- `g_servo_y_goal_us`
- `g_servo_x_us`
- `g_servo_y_us`

## 关于 360 度

普通三线 PWM 舵机不能靠代码实现“向左 360 度、向右 360 度”的绝对位置控制。当前代码只能扩大 PWM 脉宽范围，让舵机在自身机械允许范围内运动。

如果要真正左右大角度瞄准，需要换成以下之一：

- 360 度连续旋转舵机：能持续转，但通常不能直接定位到某个角度。
- 带位置反馈的多圈舵机/串口舵机：能做大角度位置控制。
- 小电机 + 编码器 + 云台结构：软件闭环控制角度。
