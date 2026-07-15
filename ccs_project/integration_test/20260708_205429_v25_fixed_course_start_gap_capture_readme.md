# 20260708_205429 V25 固定赛道混合控制：A 点无线桥接 + 捕线锁中心

## 基底

- 基于 V24：`20260708_203751_v24_a_start_gap_direct_bridge_reacquire.c`。
- 当前 CCS 活动文件已替换为：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`。
- 本版只编译验证，未自动烧录。

## 为什么改成“赛道定制混合方案”

当前比赛场地和任务固定，不需要继续追求通用赛道循迹。更合理的策略是：

- 无线段：按固定赛道定制桥接。
- 有线段：仍用红外循迹纠偏。
- 任务点：后续逐步从 `g_run_sample` 改为编码器里程触发。
- OpenMV：继续只负责靶面识别和云台追踪，不参与底盘导航。

但不建议纯脚本轨迹，因为电池电压、地面摩擦、负重和人工摆车角度都会带来累计误差。固定赛道信息应该用来决定“什么时候相信什么传感器”，而不是完全取消反馈。

## 本版解决的问题

现场反馈：

1. V24 不再顺时针画圆，但长期执行后车有轻微向右偏趋势。
2. 小车看到黑线后，没有稳定进入普通循迹。

V25 处理方式：

- A 点桥接加入轻微左修正：

```c
#define START_GAP_LEFT_TRIM_TICKS   -8
#define START_GAP_RIGHT_TRIM_TICKS  8
```

- 重捕黑线从“看到任意黑线就退出桥接”改为两级状态：
  - `PHASE_START_GAP_BRIDGE = 8`：无线直行桥接。
  - `PHASE_START_GAP_CAPTURE = 9`：低速捕线、锁中心。
  - `PHASE_LINE_RUN = 2`：正式普通循迹。

## 捕线锁中心条件

只有同时满足以下条件并连续 5 帧，才算重捕成功：

```text
g_line_valid = 1
abs(g_line_error) <= 1200
S3/S4/S5 至少一个压线
```

成功后蜂鸣 3 声，并进入 `PHASE_LINE_RUN`。

## 捕线阶段参数

```c
#define START_GAP_REACQUIRE_FRAMES  2U
#define START_GAP_CENTER_ERROR_LIMIT 1200
#define START_GAP_CENTER_LOCK_FRAMES 5U
#define START_GAP_CAPTURE_LOST_MAX_FRAMES 25U
#define START_GAP_CAPTURE_MAX_FRAMES 120U
#define START_GAP_CAPTURE_LEFT_BASE_TICKS 370U
#define START_GAP_CAPTURE_RIGHT_BASE_TICKS 370U
#define START_GAP_CAPTURE_KP_DIVISOR 10
#define START_GAP_CAPTURE_CORR_LIMIT 220
```

含义：

- 桥接阶段连续看到黑线 2 帧后，进入捕线阶段。
- 捕线阶段低速运行，避免擦线后冲出去。
- 捕线时允许短暂丢线，最多 25 帧。
- 120 帧内仍无法锁中心，则回到桥接继续找线。

## 蜂鸣提示

```text
1 声：程序启动
2 声：进入 A 点无线桥接
3 声：中心锁定成功，进入普通循迹
长响：桥接超时仍未捕线
```

## 首测步骤

1. 手动烧录当前 `empty.c`。
2. `START_GAP_TEST_ONLY` 保持 `1U`，第一轮只测 A 点起步。
3. 把车在 A 点摆正，车头朝向下一段黑线。
4. 观察：
   - 是否仍长期向右偏。
   - 是否进入捕线阶段后能慢速拉回中心。
   - 是否听到 3 声后再进入普通循迹。

## 调参规则

如果桥接仍向右偏：

```c
#define START_GAP_LEFT_TRIM_TICKS   -12
#define START_GAP_RIGHT_TRIM_TICKS  12
```

如果看到黑线但没有 3 声：

```c
#define START_GAP_CENTER_ERROR_LIMIT 1600
```

或者：

```c
#define START_GAP_CENTER_LOCK_FRAMES 3U
```

如果有 3 声但随后仍冲出线：

- 先把捕线 PWM 降低，例如 `370 -> 340`。
- 或增加捕线锁中心帧数，例如 `5 -> 8`。
- 暂时不要改普通循迹参数，因为问题发生在桥接到循迹的接入阶段。

## Watch 变量

```c
g_phase
g_turn_mode
g_line_valid
g_line_raw_mask
g_line_error
g_start_gap_bridge_frames
g_start_gap_capture_frames
g_start_gap_capture_event_count
g_start_gap_reacquired_event_count
g_start_gap_center_lock_count
g_start_gap_capture_lost_count
g_left_pwm_ticks
g_right_pwm_ticks
```

状态含义：

```text
g_phase = 8：A 点无线桥接
g_phase = 9：捕线锁中心
g_phase = 2：普通循迹
g_turn_mode = 4：桥接直行
g_turn_mode = 5：捕线阶段
```
