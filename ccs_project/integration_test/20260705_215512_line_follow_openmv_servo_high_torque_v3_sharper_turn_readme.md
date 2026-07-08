# 20260705_215512 高扭矩合并 V3：轻微加强转弯

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260705_215512_line_follow_openmv_servo_high_torque_v3_sharper_turn.c`
- 上一版：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260705_215010_line_follow_openmv_servo_high_torque_v2.c`

## 改动

普通直线/小弯参数不变，只轻微加强急弯：

V2：

```c
#define LINE_SHARP_ERROR            1800
#define LINE_SHARP_KP_DIVISOR       8
#define LINE_SHARP_CORR_LIMIT       380
```

V3：

```c
#define LINE_SHARP_ERROR            1600
#define LINE_SHARP_KP_DIVISOR       7
#define LINE_SHARP_CORR_LIMIT       430
```

含义：

- 更早进入急弯模式。
- 急弯修正略强。
- 内侧轮更容易明显降速，转弯会更急一点。

## 保留内容

- 高扭矩基础 PWM 和启动助推保持 V2 不变。
- OpenMV 双舵机最终调参保持不变。
- 普通循迹 `LINE_KP_DIVISOR=20`、`LINE_CORR_LIMIT=150` 不变，尽量不增加直线抖动。

## 验证

CCS `gmake all` 编译通过。未自动烧录。

