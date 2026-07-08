# 20260705_223215 高扭矩合并 V6：红外输入下拉防悬空

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260705_223215_line_follow_openmv_servo_high_torque_v6_sensor_pulldown.c`
- 上一版：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260705_222246_line_follow_openmv_servo_high_torque_v5_straight_balance.c`

## 现场澄清

之前观察到某一路传感器持续异常，实际原因是传感器线松动，不是循迹修正方向反。

## 改动

7 路红外输入由无上下拉改为内部下拉：

```c
DL_GPIO_RESISTOR_NONE -> DL_GPIO_RESISTOR_PULL_DOWN
```

目的：

- 传感器 DO 线松动/断开时，MSPM0 输入不容易悬空乱跳。
- 断线更倾向读作 `0`，也就是“未检测到黑线”，降低假黑线导致突然偏航的概率。

## 保留

- V5 的左右基础 PWM 平衡。
- V4 的急弯降速受控策略。
- OpenMV 双舵机参数不变。

## 注意

这个改动不能替代硬件固定。红外模块 `VCC/GND/DO` 仍必须压紧，尤其是面包板/杜邦线在车体震动时很容易瞬断。

每次把车拿起来再放回赛道测试，建议按一次 `RST`，避免 `g_line_last_error/g_line_lost_count/g_phase` 等状态保留上一段测试的结果。

## 验证

CCS `gmake all` 编译通过。未自动烧录。

