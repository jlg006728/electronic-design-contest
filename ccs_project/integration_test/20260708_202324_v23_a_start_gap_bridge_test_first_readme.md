# V23 A 点无线起步桥接测试优先版

时间：2026-07-08 20:23

## 文件

- 当前 CCS 活动源码：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260708_202324_v23_a_start_gap_bridge_test_first.c`
- 改前活动文件备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260708_202324_before_v23_current_empty_backup.c`

## 版本目标

适配 A 点起步时红外完全看不到黑线的场地。

本版默认：

```c
#define START_GAP_TEST_ONLY 1U
```

也就是优先测试 A 点无线起步桥接和重捕线，不触发 B 点停车、C/D/A 提示。桥接稳定后，把它改为 `0U` 再测完整任务。

## 保留功能

- V22 基础循迹参数。
- B 点停车、C/D/A 提示逻辑仍保留，只是在 `START_GAP_TEST_ONLY=1U` 时不触发。
- OpenMV UART、PA14/PA17 双舵机预瞄/追踪逻辑保留。
- PB11 蜂鸣器逻辑保留。

## 新增编码器接线

```text
PB12 -> 左编码器 A
PB13 -> 左编码器 B
PB14 -> 右编码器 A
PB15 -> 右编码器 B
```

沿用此前验证结论：

- 左轮前进计数软件取反。
- 右轮前进计数为正。

## A 点起步桥接逻辑

上电后：

1. 云台回 HOME。
2. 蜂鸣 1 声。
3. 读取红外。
4. 如果没有看到黑线，进入 `PHASE_START_GAP_BRIDGE = 8`。
5. 桥接期间用左右编码器前进量差值修正 PWM，保持直行。
6. 连续看到黑线 `4` 帧后退出桥接，进入普通循迹。
7. 超过 `220` 帧仍没看到线，停车，`g_stop_reason = 6`。

## 关键参数

```c
#define START_GAP_BRIDGE_ENABLE     1U
#define START_GAP_TEST_ONLY         1U
#define START_GAP_MAX_FRAMES        220U
#define START_GAP_REACQUIRE_FRAMES  4U
#define START_GAP_KP_DIVISOR        8
#define START_GAP_CORR_LIMIT        90
```

## Watch 变量

优先看：

```c
g_phase
g_stop_reason
g_turn_mode
g_line_valid
g_line_raw_mask
g_line_error
g_run_sample
```

编码器和桥接：

```c
g_left_count
g_right_count
g_left_edges
g_right_edges
g_start_gap_bridge_active
g_start_gap_bridge_frames
g_start_gap_bridge_event_count
g_start_gap_reacquire_count
g_start_gap_left_forward
g_start_gap_right_forward
g_start_gap_balance_error
g_start_gap_correction
g_start_gap_encoder_seen
```

状态含义：

```text
g_phase = 8  A 点无线起步桥接中
g_phase = 2  普通循迹
g_phase = 4  停车保护

g_turn_mode = 4  编码器桥接直行
g_stop_reason = 6  A 点桥接超时未重捕线
```

## 现场测试步骤

1. 先架空或手推轮子确认编码器：
   - `g_left_edges` 和 `g_right_edges` 都会变化。
   - 前进时 `g_start_gap_left_forward` 和 `g_start_gap_right_forward` 应为正。
2. 把车放在 A 点，车头人工摆正，朝向下一段应桥接到的黑线方向。
3. 烧录本版。
4. 观察：
   - 起步是否直行。
   - 是否不会原地找线。
   - 看到黑线后是否恢复普通循迹。

## 调参规则

- 桥接偏左/偏右：
  - 优先调 `START_GAP_KP_DIVISOR`。
  - `8 -> 6`：加强编码器纠偏。
  - `8 -> 10`：减弱编码器纠偏。
- 还没碰到线就停车：
  - 增大 `START_GAP_MAX_FRAMES`。
- 看到线但没退出桥接：
  - `START_GAP_REACQUIRE_FRAMES 4 -> 3`。
- 桥接抖动：
  - `START_GAP_CORR_LIMIT 90 -> 60`。

## 进入完整任务

桥接稳定后，把：

```c
#define START_GAP_TEST_ONLY 1U
```

改为：

```c
#define START_GAP_TEST_ONLY 0U
```

再重新生成时间戳版本并编译。那一版会恢复 B 点停车、C/D/A 声光提示。
