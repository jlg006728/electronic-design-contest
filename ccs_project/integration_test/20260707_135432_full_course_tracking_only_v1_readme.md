# 20260707_135432 全程跟踪测试 V1

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_135432_full_course_tracking_only_v1.c`
- 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_135406_before_full_course_tracking_empty.c`
- OpenMV 脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260706_163540_openmv_uart_stream_edge_guard_rot180.py`

## 本版目标

先调试全程 OpenMV/双舵机追踪，不做 ABCD 任务点。

## 本版行为

1. 上电后云台回 HOME。
2. 小车直接开始循迹。
3. OpenMV 全程回传红色目标坐标。
4. PA14/PA17 双舵机全程追踪红色目标。
5. 不在 B 点停车。
6. 不做 C/D/A 提示。
7. 仍保留全局运行超时停车 `LINE_RUN_SAMPLES=4500U`。
8. 仍保留连续丢线停车保护。

## 关键开关

```c
#define MISSION_ENABLE_B_AIM        0U
#define MISSION_ENABLE_POINT_PROMPTS 0U
```

以后要恢复 B 点停车/ABCD 任务时，可打开这些开关，或回退到 B 点左预瞄版。

## 测试重点

1. 小车绕赛道运行时，OpenMV 是否全程尽量朝向红色目标。
2. 直道、圆弯、椭圆弯、急弯时云台是否会频繁丢目标。
3. 丢目标后云台是否能水平搜索找回。
4. 目标在画面边缘时是否会被边缘保护拒绝，随后进入搜索。
5. 云台大范围运动是否影响循迹稳定性。

## Watch 建议

- `g_omv_link_ok`
- `g_omv_color_frame_count`
- `g_omv_target_detected`
- `g_omv_pixels`
- `g_omv_cx`
- `g_omv_cy`
- `g_aim_state`
- `g_aim_search_active`
- `g_servo_x_us`
- `g_servo_y_us`
- `g_diag_stage`
- `g_turn_mode`
- `g_line_raw_mask`
- `g_stop_reason`

