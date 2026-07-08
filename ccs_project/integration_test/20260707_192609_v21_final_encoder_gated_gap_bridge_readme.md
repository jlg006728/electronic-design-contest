# V21：编码器闭环 + 相位门控无线路段桥接

时间：2026-07-07 19:26

## 文件

- 当前 CCS 活动源码：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_192609_v21_final_encoder_gated_gap_bridge.c`
- 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_192609_before_v21_current_v19_backup.c`

## 基底

本版基于 V19：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_190540_v19_restore_dynamic_tracking_with_buzzer.c`

## 目标

适配比赛场地：黑线只有上下弧线，中间连接直线没有黑线。

## 核心策略

不是任意丢线都盲走，而是满足以下条件才进入桥接：

1. 当前赛道进度在预期断线窗口附近。
   - 默认窗口在 B 点和 D 点附近。
2. 丢线前最后误差足够靠边，说明确实是从弧线端点出线。
3. 进入桥接后，用左右编码器差闭环保持直线。
4. 红外重新连续看到黑线 `3` 帧后退出桥接，恢复普通循迹。
5. 桥接超过 `220` 帧仍未看到线，则退出桥接并交给原丢线停车保护。

## 新增编码器引脚

```c
PB12 -> 左编码器 A
PB13 -> 左编码器 B
PB14 -> 右编码器 A
PB15 -> 右编码器 B
```

沿用之前现场结论：

- 左轮前进计数软件取反。
- 右轮前进计数为正。

## 关键参数

```c
#define LINE_GAP_BRIDGE_ENABLE      1U
#define LINE_GAP_ENTER_ERROR        1800
#define LINE_GAP_MAX_FRAMES         220U
#define LINE_GAP_REACQUIRE_FRAMES   3U
#define LINE_GAP_KP_DIVISOR         8
#define LINE_GAP_CORR_LIMIT         90
#define LINE_GAP_WINDOW_MARGIN      70U
#define LINE_GAP_WINDOW_EXTRA       120U
```

桥接直线 PWM：

```c
#define LINE_GAP_LEFT_BASE_TICKS    PWM_LEFT_BASE_TICKS
#define LINE_GAP_RIGHT_BASE_TICKS   PWM_RIGHT_BASE_TICKS
```

## Watch 变量

```c
g_left_count
g_right_count
g_left_edges
g_right_edges
g_line_gap_bridge_active
g_line_gap_bridge_frames
g_line_gap_bridge_event_count
g_line_gap_reacquire_count
g_line_gap_entry_error
g_line_gap_left_forward
g_line_gap_right_forward
g_line_gap_balance_error
g_line_gap_correction
g_line_gap_window_ok
g_line_gap_edge_ok
g_line_gap_encoder_seen
g_turn_mode
```

`g_turn_mode = 4` 表示正在无线路段桥接。

## 调参建议

- 如果该进桥接时没进：
  - 降低 `LINE_GAP_ENTER_ERROR`，例如 `1800 -> 1400`。
  - 扩大窗口：增大 `LINE_GAP_WINDOW_MARGIN` 或 `LINE_GAP_WINDOW_EXTRA`。
- 如果不该桥接时误进：
  - 提高 `LINE_GAP_ENTER_ERROR`，例如 `1800 -> 2200`。
  - 缩小窗口。
- 如果桥接时仍偏：
  - 加强编码器纠偏：`LINE_GAP_KP_DIVISOR 8 -> 6`。
  - 若左右抖动，改回 `8` 或 `10`。
- 如果桥接还没碰到下一段线就停：
  - 增大 `LINE_GAP_MAX_FRAMES`。

## 验证

已执行 clean build 并重新编译通过。按当前规则：不自动烧录，由用户手动烧录。
