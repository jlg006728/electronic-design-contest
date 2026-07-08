# 20260704_135927 OpenMV 快速稳定追踪测试版

## 文件

- 当前 MSPM0 烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- MSPM0 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_135927_mspm0_openmv_fast_stable_track.c`
- OpenMV 脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_135927_openmv_uart_stream_fast_rot180.py`

## 修改目标

现场现象：

- OpenMV 与 MSPM0 串口已经打通，舵机可以随视觉数据动作。
- 但固定红色目标时舵机仍持续移动，甚至偏出边界。
- 舵机响应速度明显太慢，不满足后续行进间打靶需求。

本版修改：

1. OpenMV 主动串流周期从 `100ms` 改为 `50ms`。
2. OpenMV 最小红色目标面积从 `12` 提高到 `30`，减少红色噪声。
3. MSPM0 不再主动发送 `'C'` 请求，只接收 OpenMV 主动串流。
4. MSPM0 追踪参数从慢速小步改为快速分段：
   - `AIM_MAX_STEP_US = 45`
   - `AIM_X_DIVISOR = 3`
   - `AIM_Y_DIVISOR = 3`
5. 中心死区加大：
   - `AIM_DEAD_X_PIXELS = 20`
   - `AIM_DEAD_Y_PIXELS = 16`
6. 增加坐标低通滤波：
   - `g_aim_filtered_cx`
   - `g_aim_filtered_cy`
7. 增加目标确认：
   - 连续 `2` 帧有效目标后才开始动舵机。
8. 小目标拒收：
   - `g_omv_pixels < 30` 时不追踪。

## 使用步骤

1. 用 OpenMV IDE 打开并保存到 OpenMV Cam：
   `20260704_135927_openmv_uart_stream_fast_rot180.py`
2. CCS 烧录当前 `empty.c`。
3. 运行后先固定红色目标在画面边缘附近，不要一开始放在中心。
4. 看云台是否朝目标方向追。

## Watch 变量

- `g_omv_rx_bytes`
- `g_omv_color_frame_count`
- `g_omv_target_detected`
- `g_omv_cx`
- `g_omv_cy`
- `g_omv_pixels`
- `g_aim_raw_dx`
- `g_aim_raw_dy`
- `g_aim_dx`
- `g_aim_dy`
- `g_aim_filtered_cx`
- `g_aim_filtered_cy`
- `g_aim_x_step_us`
- `g_aim_y_step_us`
- `g_servo_x_us`
- `g_servo_y_us`
- `g_aim_state`

## g_aim_state

- `0`：OpenMV 串口超时/未连接
- `1`：OpenMV 正常但没看到目标
- `2`：正在追踪
- `3`：目标已进入中心死区，舵机不应继续动
- `4`：上电 HOME 初始化中
- `6`：目标太小，被拒收
- `7`：目标刚出现，正在等待连续确认

## 方向判断

如果目标固定在画面右侧，云台应该向右追；如果越追越偏，说明 X 轴方向反了，需要改：

```c
#define AIM_X_PULSE_SIGN 1
```

改成：

```c
#define AIM_X_PULSE_SIGN (-1)
```

如果上下越追越偏，改 `AIM_Y_PULSE_SIGN`。

不要用“继续加速度”解决方向反的问题；方向反时速度越快只会越快跑出边界。
