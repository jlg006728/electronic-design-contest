# 20260704_173227 单引脚 PA17 舵机扫描诊断版

## 文件

- 当前烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_173227_mspm0_diag_single_pin_pa17_wide_fast_sweep_no_uart.c`

## 目的

用户反馈 PA14 单独扫描时没有卡兹声，最开始不断电，运行久了才断电，并怀疑卡兹声来自竖直舵机。

本版只测试 PA17：

- 不初始化 OpenMV UART
- 不接收 OpenMV 数据
- 只给 `PA17` 输出 50Hz 舵机慢扫脉冲
- `PA14` 保持低电平

## 代码状态

```c
#define DIAG_SINGLE_PIN_SWEEP_MODE 1U
#define DIAG_SINGLE_PIN_IS_PA14    0U
#define DIAG_SINGLE_SWEEP_MIN_US   1100U
#define DIAG_SINGLE_SWEEP_MAX_US   1900U
#define DIAG_SINGLE_SWEEP_STEP_US  8U
```

## 判据

- 如果 PA17 单独扫描出现卡兹声：卡兹声来自竖直舵机或竖直机构。
- 如果 PA17 单独扫描也运行久后导致 OpenMV 断电：更像舵机供电支路或 5V 稳压模块热保护/掉压，而不是某一个轴的软件问题。
- 如果 PA17 单独扫描稳定，PA14 单独扫描久了断电：问题更偏向 PA14 水平舵机/水平机构/该路供电线。
- 如果 PA14/PA17 单独都稳定，但双舵机追踪会断电：问题是双舵机同时动作的瞬时电流、软件 PWM 被中断干扰，或共地/电源回路设计。
