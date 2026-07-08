# 20260706_220410 B 点左预瞄集成测试 V3

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_220410_line_follow_b_aim_left_preset_v3.c`
- OpenMV 脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260706_163540_openmv_uart_stream_edge_guard_rot180.py`

## 本版改动

1. B 点停车后，水平舵机先转到左侧预置位 `MISSION_B_AIM_X_PRESET_US = 2450`。
2. B 点静态瞄准阶段不再受 `g_turn_mode` 的“底盘转弯保持云台”逻辑限制。
3. B 点进入瞄准时清除旧目标滤波、确认计数、丢线计数和转弯状态。
4. OpenMV 丢失目标后的搜索步长从 `18us` 提到 `22us`。
5. B 点瞄准最长时间从 `250` 帧提高到 `350` 帧，约 7 秒。

## 测试重点

1. 小车到 B 点停车后，云台是否先明显向左预瞄。
2. 靶子放在 B 点左前方时，OpenMV 是否能找到并进入追踪。
3. 如果左转不够：把 `MISSION_B_AIM_X_PRESET_US` 从 `2450` 改到 `2550`。
4. 如果左转过头：把 `MISSION_B_AIM_X_PRESET_US` 从 `2450` 改到 `2300`。
5. 如果远处仍抓不稳，优先增大红色靶面或在 OpenMV IDE 里重新调红色阈值，不要直接大幅降低面积门槛。

## Watch 建议

- `g_phase`
- `g_mission_phase`
- `g_mission_last_point`
- `g_mission_b_aim_preset_applied`
- `g_mission_aim_frames`
- `g_mission_aim_success`
- `g_aim_state`
- `g_servo_x_us`
- `g_servo_y_us`
- `g_omv_target_detected`
- `g_omv_pixels`
- `g_omv_cx`
- `g_omv_cy`
- `g_diag_stage`

