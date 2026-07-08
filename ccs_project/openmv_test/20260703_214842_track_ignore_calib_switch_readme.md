# 20260703_214842 OpenMV 追踪版：忽略校准开关

## 文件

- 当前烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_214842_mspm0_openmv_track_ignore_calib_switch.c`
- OpenMV 脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_213123_openmv_full_selftest_rot180.py`

## 修改原因

上一轮校准时 Watch 中 `g_servo_calibrate_enable=1`。旧代码在该值为 1 时会：

1. 每一帧强制 `servo_go_home_now()`。
2. 设置 `g_aim_state=5`。
3. 跳过 `aiming_update_from_latest_color_frame()`。

所以即使 OpenMV 检测到目标，舵机也会被锁在 HOME，看起来就是“不追踪”。

这版正式追踪代码保留 `g_servo_calibrate_enable` 变量以兼容 Watch，但如果它被误设为 1，程序会自动清回 0，不再阻断追踪。

## 固化参数

- `SERVO_X_HOME_US = 1500`
- `SERVO_Y_HOME_US = 580`
- `SERVO_X_MIN_US = 900`
- `SERVO_X_MAX_US = 2100`
- `SERVO_Y_MIN_US = 500`
- `SERVO_Y_MAX_US = 2100`

## 测试步骤

1. OpenMV IDE 运行 `20260703_213123_openmv_full_selftest_rot180.py`。
2. CCS 烧录当前 `empty.c`。
3. Resume 运行。
4. 拿红色目标放到画面边缘，先不要放在中心附近。

## Watch 判断表

- `g_omv_tx_requests` 一直增加：MSPM0 正在向 OpenMV 请求颜色识别。
- `g_omv_rx_bytes` 一直增加：MSPM0 收到了 OpenMV 串口数据。
- `g_omv_color_frame_count` 一直增加：MSPM0 成功解析到了颜色帧。
- `g_omv_target_detected = 1`：MSPM0 收到的帧里确认检测到目标。
- `g_omv_cx/g_omv_cy` 随目标位置变化：坐标链路正常。
- `g_aim_state = 2`：正在追踪。
- `g_aim_state = 3`：目标在中心死区内，舵机不动是正常的。
- `g_servo_x_us/g_servo_y_us` 变化：MSPM0 已经在改舵机命令。

如果 OpenMV IDE 显示 `target=1`，但 `g_omv_rx_bytes` 不增加，说明只是 OpenMV 自己识别到了，MSPM0 没收到数据，优先查 PA10/PA11 串口接线、共地、OpenMV 脚本是否在运行。
