# 20260708_203751 V24 A 点无线起步桥接修正版

## 基底

- 基于 `20260708_202324_v23_a_start_gap_bridge_test_first.c` 继续修改。
- 当前 CCS 活动文件已替换为 `C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`。
- 本版只编译验证，不自动烧录。

## 这版解决的问题

现场现象：

1. A 点无线起步后，小车一直绕同一个顺时针圆圈，没有直行桥接。
2. 手动让红外连续几秒识别到黑线，小车仍没有明显进入正常循迹。

判断：

- V23 在编码器方向尚未复测前就默认启用了“编码器差速闭环桥接”。如果左右编码器方向、接线相序或计数符号任意一项与假设不一致，闭环会把直行修正变成持续转向。
- V23 从桥接直接跳回普通循迹，没有单独的重捕线稳定相位；如果车身姿态已经偏斜，刚看到线就可能又被带出线。

## V24 关键改动

```c
#define START_GAP_USE_ENCODER_BALANCE 0U
#define START_GAP_ALIGN_FRAMES      12U
#define START_GAP_LEFT_TRIM_TICKS   0
#define START_GAP_RIGHT_TRIM_TICKS  0
```

- 默认关闭编码器闭环修正，桥接阶段先按左右基准 PWM 直行。
- 编码器计数仍然更新，方便后续复测方向；只是暂时不参与控制。
- 新增 `PHASE_START_GAP_ALIGN = 9U`：重捕黑线后先用普通循迹稳定约 12 帧，再进入正式 `PHASE_LINE_RUN`。
- 进入桥接蜂鸣 2 声；成功重捕黑线蜂鸣 3 声。

## 首测步骤

1. 手动烧录当前 `empty.c`。
2. 把车放在 A 点无线起步位置，人工摆正车头，朝向下一段黑线。
3. 上电后听蜂鸣：
   - 1 声：程序启动。
   - 2 声：确认进入 A 点无线桥接。
   - 3 声：确认已经重捕黑线，进入稳定循迹阶段。
4. 第一轮只看三件事：
   - 是否还绕固定顺时针圆圈。
   - 是否能大体直行靠近下一段黑线。
   - 遇到黑线后是否蜂鸣 3 声并恢复循迹。

## 如果仍然绕顺时针圆圈

这时基本不是编码器闭环问题，而是左右轮实际驱动力不一致或方向/接线异常。先按下面调：

```c
#define START_GAP_LEFT_TRIM_TICKS   -20
#define START_GAP_RIGHT_TRIM_TICKS  20
```

含义：小车顺时针右转，通常说明左轮相对更快或右轮相对更慢，所以减左轮、加右轮。

如果变成逆时针，再把数值减半：

```c
#define START_GAP_LEFT_TRIM_TICKS   -10
#define START_GAP_RIGHT_TRIM_TICKS  10
```

## 如果看到黑线但没有蜂鸣 3 声

优先说明红外读数没有被代码判定为有效黑线，检查：

- 7 路红外是否通电。
- 黑线压到传感器时，对应 `DO` 是否从 0/1 正常变化。
- 当前代码仍假设 `LINE_BLACK_IS_HIGH = 1`，即黑线输出高电平。

可尝试：

```c
#define START_GAP_REACQUIRE_FRAMES  3U
```

## 如果蜂鸣 3 声后又偏出线

说明桥接退出已经成功，问题转移到普通循迹姿态或速度：

- 先把车摆得更正，确认不是入线角度过大。
- 可把 `START_GAP_ALIGN_FRAMES` 增加到 `20U`。
- 不要先动 B/C/D/A 任务点参数；当前 `START_GAP_TEST_ONLY = 1U`，任务点本来就是关闭的。

## Watch 变量

```c
g_phase
g_turn_mode
g_line_valid
g_line_raw_mask
g_start_gap_bridge_event_count
g_start_gap_reacquired_event_count
g_start_gap_align_frames_remaining
g_start_gap_left_forward
g_start_gap_right_forward
g_start_gap_encoder_seen
```

含义：

```text
g_phase = 8: A 点无线桥接
g_phase = 9: 重捕线后稳定
g_phase = 2: 正常循迹
g_turn_mode = 4: 桥接直行
g_turn_mode = 5: 重捕线稳定
```
