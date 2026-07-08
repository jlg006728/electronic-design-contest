# 20260705_222246 高扭矩合并 V5：直线基础平衡

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260705_222246_line_follow_openmv_servo_high_torque_v5_straight_balance.c`
- 上一版：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260705_221203_line_follow_openmv_servo_high_torque_v4_controlled_turn.c`

## 现场现象

车辆放到直线段时，本应识别为直线，但上车后立即有右转倾向；远离黑线后又出现左偏补偿感。

判断：先修基础直线偏置。V4 中普通、启动助推和急弯低速基准都存在 `LEFT > RIGHT` 的固定差值，在当前负重和供电状态下会形成启动右偏。

## 改动

V4：

```c
#define PWM_LEFT_BASE_TICKS         720U
#define PWM_RIGHT_BASE_TICKS        690U
#define PWM_LEFT_BOOST_TICKS        900U
#define PWM_RIGHT_BOOST_TICKS       870U
#define PWM_SHARP_LEFT_BASE_TICKS   620U
#define PWM_SHARP_RIGHT_BASE_TICKS  590U
```

V5：

```c
#define PWM_LEFT_BASE_TICKS         705U
#define PWM_RIGHT_BASE_TICKS        705U
#define PWM_LEFT_BOOST_TICKS        880U
#define PWM_RIGHT_BOOST_TICKS       880U
#define PWM_SHARP_LEFT_BASE_TICKS   605U
#define PWM_SHARP_RIGHT_BASE_TICKS  605U
```

## 保留

- V4 的急弯降速受控策略保留。
- OpenMV 双舵机参数不变。
- 急弯触发参数不变。

## 测试重点

1. 放在直线黑线上，刚启动是否还明显右偏。
2. 如果仍右偏，下一版应让右侧基准略高于左侧。
3. 如果变成左偏，下一版恢复一点左侧基准。
4. 先只看直线起步，不要同时根据丢线后的补偿方向改参数。

## 验证

CCS `gmake all` 编译通过。未自动烧录。

