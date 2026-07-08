# 20260704_162030 OpenMV 掉电诊断版：冻结水平舵机

## 文件

- 当前烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_162030_mspm0_openmv_diag_freeze_yaw.c`
- OpenMV 脚本继续使用：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_160413_openmv_uart_stream_smoother_fast_rot180.py`

## 目的

用户反馈即使水平软限位保护后，OpenMV 仍会断电。为了判断断电是否由水平舵机运动造成，本版冻结水平舵机：

```c
#define DIAG_FREEZE_YAW_SERVO 1U
```

水平舵机锁定在 HOME，不参与左右追踪；OpenMV 串口、视觉识别、俯仰舵机仍保留。

## 测试判据

- 如果本版不再断电：问题集中在水平舵机运动、水平舵机供电回路、线束拉扯或水平舵机自身。
- 如果本版仍然断电：问题不在水平追踪角度，优先排查 OpenMV 供电线、GND 走线、UART 线干扰、OpenMV 固定/线束受力，或 OpenMV 板子自身。

## 现场确认

烧录后水平舵机不应该左右动。若仍左右动，说明烧录错版本、接错舵机信号线，或水平/俯仰舵机线对调。
