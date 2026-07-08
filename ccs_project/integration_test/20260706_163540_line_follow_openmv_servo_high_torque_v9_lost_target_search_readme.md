# 20260706_163540 V9 lost target search

## 现象判断

视频里底盘循迹已经能跑，但过弯后 OpenMV 明显会丢失红色目标。旧逻辑在移动底盘上有三个风险：

- 目标丢失后只保持最后姿态，不主动找回。
- 目标刚到画面边缘时仍按有效目标追踪，容易把水平舵机越推越偏。
- `OpenMV link lost` 分支仍保留慢慢回 HOME，P4/P5 瞬断时会和重新追踪叠加成摇摆。

## 本版改动

当前 CCS 活动文件：

`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`

时间戳源码：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_163540_line_follow_openmv_servo_high_torque_v9_lost_target_search.c`

OpenMV 脚本：

`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260706_163540_openmv_uart_stream_edge_guard_rot180.py`

保留底盘参数：

- `LINE_CENTER_OFFSET = 500`
- `LINE_KP_DIVISOR = 16`
- `LINE_CORR_LIMIT = 190`
- 急弯/高扭矩参数保持 V8

同步用户截图中的舵机参数：

- `SERVO_X_HOME_US = 1600`
- `SERVO_Y_HOME_US = 1500`
- `SERVO_X_MIN_US = 350`
- `SERVO_X_MAX_US = 2650`
- `SERVO_Y_MIN_US = 500`
- `SERVO_Y_MAX_US = 2500`

新增视觉策略：

- `g_aim_state = 8`：目标在画面边缘，拒绝直接追踪。
- `g_aim_state = 9`：丢目标后开始水平扫描找回。
- `g_aim_state = 10`：底盘正在急弯或丢线找回，云台先保持不大动。
- OpenMV 脚本拒绝贴边 blob，避免把半个红色目标当作稳定目标。
- 断链时不再自动回 HOME，只保持最后舵机位置并等待串口恢复。

## 现场测试方法

1. OpenMV IDE 运行并保存新脚本：
   `20260706_163540_openmv_uart_stream_edge_guard_rot180.py`
2. CCS 手动烧录当前 `empty.c`。
3. 先架空测试：确认电机、OpenMV、双舵机都在工作。
4. 再放到赛道上跑，看过弯后：
   - 丢目标时云台应短暂停住。
   - 底盘回到普通循迹后，水平舵机应有限扫描找回目标。
   - 不应一路追到边界后卡住。

## 可调参数

- 找目标太慢：`AIM_SEARCH_STEP_US 18 -> 24`
- 找目标太猛：`AIM_SEARCH_STEP_US 18 -> 12`
- 过弯期间仍乱动：`AIM_SEARCH_START_FRAMES 14 -> 20`
- 边缘目标拒绝太严格：`AIM_EDGE_MARGIN_X 18 -> 10`
- OpenMV 贴边过滤太严格：`BORDER_REJECT_MARGIN 3 -> 1`

本次只编译验证，未自动烧录。
