# 20260706_214843 line follow B aim C/D/A prompt V2

## 目的

在 V1 “到 B 点停车静态瞄准”的基础上，补齐基本要求链路：

1. A 点启动循迹。
2. 到 B 点停车并静态瞄准。
3. 瞄准稳定或超时后继续循迹。
4. 经过 C、D 点时 LED 提示。
5. 回到 A 点停车，LED 常亮。

## 文件

当前 CCS 活动文件：

`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`

时间戳源码：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_214843_line_follow_b_aim_cda_prompt_v2.c`

备份：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_214843_before_keypoint_prompt_empty.c`

推荐 OpenMV 脚本：

`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260706_163540_openmv_uart_stream_edge_guard_rot180.py`

## 新增/关键参数

```c
#define MISSION_B_STOP_SAMPLE       180U
#define MISSION_C_PROMPT_SAMPLE     360U
#define MISSION_D_PROMPT_SAMPLE     540U
#define MISSION_A_FINISH_SAMPLE     720U
#define MISSION_AIM_MAX_FRAMES      250U
#define MISSION_AIM_STABLE_FRAMES   20U
#define MISSION_POINT_PROMPT_FRAMES 40U
```

当前按 20ms 控制周期估算：

- B：约 3.6s
- C：约 7.2s
- D：约 10.8s
- A：约 14.4s
- B 点瞄准最长：约 5s
- C/D 提示：约 0.8s

## 新增 Watch 变量

```c
g_mission_phase
g_mission_b_done
g_mission_c_done
g_mission_d_done
g_mission_a_done
g_mission_last_point
g_mission_aim_success
g_mission_aim_frames
g_mission_prompt_frames
```

`g_mission_last_point`：

- `1` = A
- `2` = B
- `3` = C
- `4` = D

## 现场调参

- B 停太早/太晚：调 `MISSION_B_STOP_SAMPLE`
- C 提示太早/太晚：调 `MISSION_C_PROMPT_SAMPLE`
- D 提示太早/太晚：调 `MISSION_D_PROMPT_SAMPLE`
- A 停太早/太晚：调 `MISSION_A_FINISH_SAMPLE`

调参时每次只改一个点，避免定位混乱。

本次只编译验证，未自动烧录。
