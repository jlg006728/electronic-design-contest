# 20260705_215010 循迹 + OpenMV 双舵机高扭矩合并 V2

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260705_215010_line_follow_openmv_servo_high_torque_v2.c`
- V2 前活动文件备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260705_215010_before_high_torque_v2_empty.c`
- OpenMV 推荐脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260705_145746_openmv_uart_stream_watchdog_no_print_rot180.py`

## 改动目的

小车增加 OpenMV、云台、舵机和供电线束后负重增加，原合并 V1 的电机基础 PWM 不足，可能出现落地走不动或电机滋滋响。

## 电机 PWM 调整

V1：

```c
#define PWM_LEFT_BASE_TICKS         620U
#define PWM_RIGHT_BASE_TICKS        590U
#define PWM_MIN_TICKS               430U
#define PWM_MAX_TICKS               850U
```

V2：

```c
#define PWM_LEFT_BASE_TICKS         720U
#define PWM_RIGHT_BASE_TICKS        690U
#define PWM_LEFT_BOOST_TICKS        900U
#define PWM_RIGHT_BOOST_TICKS       870U
#define PWM_MIN_TICKS               520U
#define PWM_MAX_TICKS               1000U
#define START_BOOST_SAMPLES         20U
```

`START_BOOST_SAMPLES=20` 表示前约 `20 * 20ms = 400ms` 使用启动助推基准 PWM，之后恢复到正常高扭矩基础 PWM。启动助推仍保留循迹修正，不是盲目直冲。

## 保留内容

- OpenMV 双舵机最终调参参数保持不变。
- `PA14/PA17` 云台继续追踪红色目标。
- 启动无线时先直走找线。
- 已经见线后连续丢线约 2 秒停车。
- 运行约 90 秒停车。

## Watch 新增建议

- `g_start_boost_active`
- `g_left_pwm_ticks`
- `g_right_pwm_ticks`
- `g_turn_mode`
- `g_stop_reason`

`g_start_boost_active=1` 表示正在启动助推阶段。

## 首次测试

1. 先架空测试，确认两侧电机能明显转起来。
2. 再落地测试起步是否不再滋滋响。
3. 如果起步可以但直角弯冲出，下一步降低 `PWM_LEFT/RIGHT_BASE_TICKS` 或加强急弯策略。
4. 如果仍然滋滋响不走，优先查电池电压、TB6612 VM、电机接线、车轮机械阻力和驱动电流能力。

## 验证

CCS `gmake all` 编译通过。按当前规则，未自动烧录。

