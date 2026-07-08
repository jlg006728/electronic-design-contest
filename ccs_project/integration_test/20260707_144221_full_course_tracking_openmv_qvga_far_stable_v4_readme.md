# 20260707_144221 全程跟踪 + OpenMV QVGA 远距离稳定识别 V4

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- MSPM0 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_144221_full_course_tracking_openmv_qvga_far_stable_v4.c`
- MSPM0 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_144221_before_revert_openmv_qvga_far_stable_empty.c`
- OpenMV 新脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260707_144221_openmv_uart_stream_qvga_far_stable_v2.py`
- 失败的 VGA 版本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260707_143104_openmv_uart_stream_vga_far_red_v1.py`

## 为什么回滚 VGA

VGA 版本现场表现为：

- OpenMV 红灯常亮
- 插电脑时颜色明显异常
- 帧率很卡
- 小车无法追踪

按脚本逻辑，红灯是异常恢复指示，不是目标检测指示。绿灯才表示检测到目标，蓝灯是心跳。因此 VGA 版不适合作为当前实车追踪版本。

## 本版策略

回到稳定的 QVGA 320x240，不再设置 `sensor.set_saturation(2)`，只做温和远距离识别增强：

```python
RED_THRESHOLDS = [
    (20, 80, 20, 90, 10, 80),
    (14, 90, 16, 100, -2, 90),
]

MIN_RED_PIXELS = 14
MIN_RED_AREA = 14
MIN_RED_W = 2
MIN_RED_H = 2
MIN_RED_DENSITY_PERCENT = 10
CENTER_PENALTY_DIVISOR = 4
```

含义：

1. 保留旧稳定阈值。
2. 新增一组稍宽的红色阈值，用于远处较暗/较小目标。
3. 允许更小的红色目标被识别。
4. 通过宽高和密度过滤减少空场景噪声。
5. 降低中心惩罚，远处偏边目标不容易被误丢。

## MSPM0 同步改动

```c
#define IMG_WIDTH                   320
#define IMG_HEIGHT                  240
#define AIM_DEAD_X_PIXELS           20
#define AIM_DEAD_Y_PIXELS           16
#define AIM_MIN_PIXELS              14U
```

`AIM_MIN_PIXELS` 必须同步降低，否则 OpenMV 识别到的小远目标会被 MSPM0 当成“小目标”拒绝。

## 测试顺序

1. 先用 OpenMV IDE 运行：

```text
C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260707_144221_openmv_uart_stream_qvga_far_stable_v2.py
```

2. 确认：
   - 红灯不应常亮。
   - 蓝灯应周期闪烁。
   - 看到红色靶时绿灯应稳定亮。
   - IDE 画面颜色应比 VGA 版正常，帧率应明显恢复。

3. 静态测试 A/B/BC/CD 距离能否框住目标。
4. 静态稳定后，再手动烧录当前 `empty.c` 跑全程跟踪。

## 如果仍然识别不到

下一步不要再上 VGA，优先在 OpenMV IDE Threshold Editor 里重新取现场红色靶的 LAB 阈值，并考虑把靶面红色面积做大、减少白字/黑框占比。

