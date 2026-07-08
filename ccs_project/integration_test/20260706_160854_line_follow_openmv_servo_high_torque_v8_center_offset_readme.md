# 20260706_160854 高扭矩合并 V8：S4/S5 中心偏置

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_160854_line_follow_openmv_servo_high_torque_v8_center_offset.c`
- 上一版：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_155859_line_follow_openmv_servo_high_torque_v7_center_correction.c`

## 现场判断

车身中心感觉在 `S4` 和 `S5` 中间，而旧算法一直把 `S4` 当作软件中心：

```c
S4 = 0
S5 = +1000
```

因此软件中心应调整为 `+500`，也就是 `S4/S5` 中间。

## 改动

新增：

```c
#define LINE_CENTER_OFFSET          500
```

误差计算从：

```c
g_line_error = (int16_t) (sum / (int32_t) count);
```

改为：

```c
g_line_error = (int16_t) ((sum / (int32_t) count) - LINE_CENTER_OFFSET);
```

效果：

- `S4 + S5` 同时压线时，平均位置为 `500`，减去偏置后误差为 `0`。
- 单独 `S5` 压线时，误差从 `+1000` 变为 `+500`。
- 单独 `S4` 压线时，误差从 `0` 变为 `-500`。

## 保留

- V7 普通居中纠偏强度。
- V6 红外输入下拉。
- V5 左右 PWM 平衡。
- V4 急弯降速受控策略。
- OpenMV 双舵机参数不变。

## 测试重点

观察黑线是否稳定在 `S4/S5` 中间，而不是只压 `S5`。如果仍偏向 `S5`，说明还存在机械偏心或左右动力偏差；如果明显偏向 `S4`，则 `LINE_CENTER_OFFSET` 可减小到 `300~400`。

## 验证

CCS `gmake all` 编译通过。未自动烧录。

