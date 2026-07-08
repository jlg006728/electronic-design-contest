# 20260705_221203 高扭矩合并 V4：急弯降速受控版

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260705_221203_line_follow_openmv_servo_high_torque_v4_controlled_turn.c`
- V4 前活动文件备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260705_221203_before_controlled_turn_v4_empty.c`

## 背景

V2 视频显示最后不是单纯“转弯太慢”，而是到弯道/底部区域后车头偏角过大，最终应该继续往右转，但控制方向变成了往左。V3 继续加强急弯后，把错误方向放大，导致放上去就容易原地打转。

因此 V4 不再继续加强急弯，而是回到 V2 急弯触发参数，并在急弯时主动降速和限制外侧轮速度，降低冲过头后读到错误侧传感器的概率。

## 保留 V2 参数

```c
#define LINE_SHARP_ERROR            1800
#define LINE_SHARP_KP_DIVISOR       8
#define LINE_SHARP_CORR_LIMIT       380
```

## 新增急弯低速参数

```c
#define PWM_SHARP_LEFT_BASE_TICKS   620U
#define PWM_SHARP_RIGHT_BASE_TICKS  590U
#define PWM_SHARP_MAX_TICKS         850U
```

普通直线/小弯仍使用高扭矩：

```c
#define PWM_LEFT_BASE_TICKS         720U
#define PWM_RIGHT_BASE_TICKS        690U
#define PWM_LEFT_BOOST_TICKS        900U
#define PWM_RIGHT_BOOST_TICKS       870U
#define PWM_MIN_TICKS               520U
#define PWM_MAX_TICKS               1000U
```

## 预期效果

- 直线和起步仍有足够动力。
- 急弯时不要让外侧轮过快，减少冲过弯和反向误判。
- 避免 V3 的原地打转。

## 如果仍然在同一位置转错方向

下一步不要再调强急弯，而要针对该弯道做“方向确认”逻辑：

- 连续多帧确认右侧传感器优先，才允许进入右急弯。
- 或者在丢线找回时限制方向反转，短时间内继续沿最后稳定弯道方向找线。

## 验证

CCS `gmake all` 编译通过且无警告。未自动烧录。

