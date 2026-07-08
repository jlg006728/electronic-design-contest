# 20260707_153507 关键点停车 + 有源蜂鸣器 V8

## 文件

- 当前 CCS 活动文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_153507_course_point_stop_buzzer_v8.c`
- OpenMV 脚本保持：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260707_144221_openmv_uart_stream_qvga_far_stable_v2.py`

## 蜂鸣器接线

按 5V 有源蜂鸣器处理，推荐三极管/MOSFET 驱动：

```text
5V -> 蜂鸣器 +
蜂鸣器 - -> 三极管/MOSFET 输出端
三极管/MOSFET 另一端 -> GND
MSPM0 PB11 -> 1kΩ 电阻 -> 三极管基极/栅极
MSPM0 GND 与 5V 电源 GND 共地
```

代码使用：

```c
#define BUZZER_PORT GPIOB
#define BUZZER_PIN  DL_GPIO_PIN_11
```

PB11 对应 `IOMUX_PINCM28`。

## 声音逻辑

蜂鸣器是非阻塞状态机，按 20ms 舵机帧更新，不会用长 delay 卡住小车。

当前鸣叫规则：

- 上电 HOME：短响 1 声
- 到 B 点：短响 2 声
- OpenMV 瞄准稳定：短响 3 声
- 到 C 点：短响 4 声
- 到 D 点：短响 5 声
- 到 A 点：短响 1 声
- 关键点停车 2s 内没有锁定目标：长响约 1s

当前节奏：

```c
#define BUZZER_ON_FRAMES    5U   // 约100ms
#define BUZZER_OFF_FRAMES   5U   // 约100ms
#define BUZZER_LONG_FRAMES  50U  // 约1s
```

## 关键 Watch

```c
g_buzzer_mode
g_buzzer_output_on
g_buzzer_beeps_left
g_buzzer_frames_left
g_buzzer_event_count
g_point_hold_id
g_point_hold_target_locked
g_aim_state
g_omv_target_detected
```

`g_buzzer_mode`：

- `0` = 静音
- `1` = 短响序列
- `2` = 长响

`g_point_hold_target_locked = 1` 表示关键点停车期间已经出现过稳定瞄准。

## 验证

- CCS `gmake all` 编译通过。
- 未自动烧录。
