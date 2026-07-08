# 20260706_155859 高扭矩合并 V7：加强普通居中纠偏

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_155859_line_follow_openmv_servo_high_torque_v7_center_correction.c`
- 上一版：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260705_223215_line_follow_openmv_servo_high_torque_v6_sensor_pulldown.c`

## 现场现象

小车前两圈可以正常循迹，但第三圈在后半圈直线某处偏出轨道并停车；前两圈仍主要由从左到右第 5 个传感器 `S5` 压在黑线上。

## 判断

这说明小车能识别赛道，但直线居中偏弱，长期贴着线的一侧跑。这样前两圈可能看起来可用，但累计偏差或赛道某处反光/胶带变化后就容易出轨。

## 改动

只加强普通直线/小弯纠偏，不改急弯、不改高扭矩、不改 OpenMV：

V6：

```c
#define LINE_KP_DIVISOR             20
#define LINE_CORR_LIMIT             150
```

V7：

```c
#define LINE_KP_DIVISOR             16
#define LINE_CORR_LIMIT             190
```

当只有 `S5` 压线时，普通修正由约 `50 tick` 提高到约 `62 tick`，让车更主动把线拉回中间。

## 测试重点

1. 观察直线阶段黑线是否从 `S5` 更接近 `S4/S5` 之间或 `S4`。
2. 观察直线是否出现明显左右抖动。
3. 如果仍长期压 `S5`，下一步需要机械调整传感器排位置，或加入 `LINE_CENTER_OFFSET`。
4. 如果直线抖动明显，则把 `LINE_KP_DIVISOR` 回调到 `18`。

## 验证

CCS `gmake all` 编译通过。未自动烧录。

