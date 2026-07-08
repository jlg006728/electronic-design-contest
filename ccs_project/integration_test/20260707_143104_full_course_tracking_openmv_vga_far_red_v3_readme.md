# 20260707_143104 全程跟踪 + OpenMV VGA 远距离识别 V3

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- MSPM0 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_143104_full_course_tracking_openmv_vga_far_red_v3.c`
- MSPM0 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_143104_before_openmv_vga_far_detect_empty.c`
- OpenMV 新脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260707_143104_openmv_uart_stream_vga_far_red_v1.py`

## 参考老师代码后的结论

老师给的 25E/24H 参考工程没有直接可用的 OpenMV 视觉识别代码，主要是底盘、灰度、编码器、任务状态机和蜂鸣器。这里借鉴的是它的思路：先让传感器输入更稳定，再进入任务判断，不直接靠一个脆弱阈值。

## 本版目标

解决“定点稍微远一点 OpenMV 就识别不到红色靶”的问题。

## OpenMV 改动

1. 分辨率从 `QVGA 320x240` 改为 `VGA 640x480`，让远处靶子占更多像素。
2. 红色阈值从单组保守阈值改为两组：

```python
RED_THRESHOLDS = [
    (20, 80, 20, 90, 10, 80),
    (12, 92, 14, 105, -5, 95),
]
```

3. 降低远距离小目标门槛，但增加 blob 质量过滤：

```python
MIN_RED_PIXELS = 45
MIN_RED_AREA = 35
MIN_RED_W = 3
MIN_RED_H = 3
MIN_RED_DENSITY_PERCENT = 14
```

4. 降低中心惩罚，避免远处目标在画面边缘时被小噪声抢走。
5. 提高饱和度 `sensor.set_saturation(2)`，增强红色分离。
6. UART 主动发送间隔改为 `40ms`，适配 VGA 较低帧率。

## MSPM0 改动

OpenMV 使用 VGA 后，MSPM0 必须同步：

```c
#define IMG_WIDTH                   640
#define IMG_HEIGHT                  480
#define AIM_DEAD_X_PIXELS           32
#define AIM_DEAD_Y_PIXELS           24
```

否则 MSPM0 会继续按 `320x240` 的中心计算，舵机会追错方向。

## 测试步骤

1. 先用 OpenMV IDE 运行新脚本：

```text
C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260707_143104_openmv_uart_stream_vga_far_red_v1.py
```

2. 不跑车，先定点测试远距离识别：
   - 近距离
   - A 点视角
   - B 点视角
   - BC 段视角
   - CD 段视角

3. 看 OpenMV 绿灯是否稳定亮；在 IDE 里看框是否能框住红色靶。
4. 静态能识别后，再烧录 MSPM0 当前 `empty.c` 做全程跟踪。

## 如果仍识别不到

1. 先增大红色靶面，减少白字/黑框占比。
2. 在 OpenMV IDE Threshold Editor 里重新取红色阈值。
3. 如果误识别空场景，把 `MIN_RED_PIXELS` 提到 `80`，或把第二组阈值收窄。

