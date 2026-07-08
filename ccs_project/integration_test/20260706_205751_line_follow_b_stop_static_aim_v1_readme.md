# 20260706_205751 line follow B stop static aim V1

## 目的

进入基本要求的“巡迹到靶位联动”第一版：

1. 小车从 A 点出发循迹。
2. 到达估计的 B 点位置后停车。
3. 停车状态下 OpenMV + PA14/PA17 云台静态瞄准。
4. 瞄准稳定或超时后继续巡迹。

## 文件

当前 CCS 活动文件：

`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`

时间戳源码：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_205751_line_follow_b_stop_static_aim_v1.c`

备份：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_205751_before_mission_b_stop_aim_empty.c`

推荐 OpenMV 脚本：

`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260706_163540_openmv_uart_stream_edge_guard_rot180.py`

## 新增参数

```c
#define MISSION_B_STOP_SAMPLE       180U
#define MISSION_AIM_MAX_FRAMES      250U
#define MISSION_AIM_STABLE_FRAMES   20U
```

含义：

- `MISSION_B_STOP_SAMPLE`：从启动开始跑多少个 20ms 控制周期后停车，当前约 `3.6s`。
- `MISSION_AIM_MAX_FRAMES`：停车瞄准最长等待时间，当前约 `5s`。
- `MISSION_AIM_STABLE_FRAMES`：进入中心死区连续多少帧后认为瞄准稳定，当前约 `0.4s`。

## 新增 Watch 变量

```c
g_mission_phase
g_mission_b_done
g_mission_aim_success
g_mission_aim_frames
g_mission_aim_stable_frames
```

解释：

- `g_mission_phase = 0`：还没到 B。
- `g_mission_phase = 1`：B 点停车瞄准中。
- `g_mission_phase = 2`：B 点瞄准结束，继续巡迹。
- `g_mission_aim_success = 1`：瞄准进入死区并稳定。
- `g_mission_aim_success = 0`：瞄准超时后继续。

## 现场调试顺序

1. 先只看停车点是否接近 B 点。
2. 如果停车太早：增大 `MISSION_B_STOP_SAMPLE`，例如 `180 -> 210`。
3. 如果停车太晚：减小 `MISSION_B_STOP_SAMPLE`，例如 `180 -> 150`。
4. B 点位置调准后，再看停车后云台是否能在 5 秒内对准靶心。

## 保留内容

- 当前回退版循迹参数和速度。
- OpenMV/舵机追踪逻辑。
- V9 丢目标搜索和边缘目标保护。

本次只编译验证，未自动烧录。
