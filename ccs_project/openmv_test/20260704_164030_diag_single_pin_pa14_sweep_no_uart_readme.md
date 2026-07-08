# 20260704_164030 单引脚 PA14 舵机扫描诊断版

## 文件

- 当前烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_164030_mspm0_diag_single_pin_pa14_sweep_no_uart.c`

## 目的

用户反馈慢扫诊断版中竖直轴也会动。为了区分是接线/轴定义问题，还是 UART 中断造成软件 PWM 抖动，本版做最小化测试：

- 不初始化 OpenMV UART
- 不接收 OpenMV 数据
- 只给 `PA14` 输出 50Hz 舵机慢扫脉冲
- `PA17` 保持低电平，不输出舵机控制脉冲

## 代码状态

```c
#define DIAG_SINGLE_PIN_SWEEP_MODE 1U
#define DIAG_SINGLE_PIN_IS_PA14    1U
```

PA14 在 `1300..1700us` 之间慢扫。

## 判据

- 如果水平轴动：`PA14` 确认是水平舵机信号，之前竖直动更可能是 UART 中断/供电噪声导致的抖动。
- 如果竖直轴动：`PA14` 实际接到了竖直舵机，当前 PA14/PA17 物理轴定义与代码命名相反。
- 如果两个轴都动：有信号线串扰、共地/供电干扰，或两个舵机电源/信号线接错。
- 如果 OpenMV 仍断电：即使没有 UART 干扰，PA14 这一路舵机运动仍会干扰 OpenMV 供电，需要检查这一路舵机本体/电源/GND。
