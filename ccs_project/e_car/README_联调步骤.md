# E 题 e_car 代码联调步骤

## 1. 代码放置

在 CCS 里先从 `File -> New -> CCS Project` 创建 LP-MSPM0G3507 的 empty 工程，路径建议用英文，例如：

```text
C:\Users\1\Desktop\ELECTRIC\e_car
```

然后把本目录下这些文件复制到工程根目录：

```text
main.c
board.c board.h
config.h
motor.c motor.h
encoder.c encoder.h
line_track.c line_track.h
gimbal.c gimbal.h
openmv.c openmv.h
pid.c pid.h
state_machine.c state_machine.h
```

把 empty 工程自带的 `empty.c` 从工程里删除或右键 `Exclude from Build`，否则会出现两个 `main()`。

保留 empty 工程自带的 `empty.syscfg`，代码会继续调用 `SYSCFG_DL_init()` 初始化系统和 GPIO 电源；所有实际引脚、PWM、UART 都在 `board.c` 手动初始化。

## 2. 当前固定接线

### TB6612

| MSPM0 | TB6612 |
|---|---|
| PB0 | AIN1 |
| PB1 | AIN2 |
| PB2 | BIN1 |
| PB3 | BIN2 |
| PB4 | STBY |
| PA12 | PWMA |
| PA13 | PWMB |

### 巡线 DO

| 传感器 | MSPM0 |
|---|---|
| S1 最左 | PA2 |
| S2 | PA5 |
| S3 | PA6 |
| S4 中间 | PA7 |
| S5 | PA8 |
| S6 | PB9 |
| S7 最右 | PB10 |

代码默认 `黑线=HIGH, 白底=LOW`。如果实测相反，改 `config.h`：

```c
#define LINE_BLACK_IS_HIGH 0U
```

### 编码器

原文档里 `PC0-PC3` 不要用。当前 SDK/板级代码按 PB12-PB15：

| 编码器 | MSPM0 |
|---|---|
| 左 A | PB12 |
| 左 B | PB13 |
| 右 A | PB14 |
| 右 B | PB15 |

当前 ISR 只用 A 相双边沿计数、B 相判方向，所以 `ENCODER_COUNTS_PER_REV` 先设为 30000，需要手转一圈现场校准。

### 舵机与 OpenMV

| 模块 | MSPM0 |
|---|---|
| 舵机 X 信号 | PA3 |
| 舵机 Y 信号 | PA4 |
| MSPM0 UART0 TX | PA10 -> OpenMV RX |
| MSPM0 UART0 RX | PA11 <- OpenMV TX |

OpenMV 脚本在：

```text
ccs_project\e_car\openmv\openmv_e_target.py
```

## 3. 分模块验证顺序

1. 只接 TB6612 + 电机，确认 PA12/PA13 有 PWM，电机能开环巡线输出。
2. 接 7 路 TCRT5000 DO，把车举起，用黑线逐个扫传感器，观察电机左右修正方向。
3. 接编码器，手转左右轮一圈，先校准 `ENCODER_COUNTS_PER_REV`。
4. 接一个 MG90S，确认 PA3 中位 1500us；再接 PA4。
5. OpenMV 单独用 IDE 调阈值，确认能识别红色靶心。
6. 拔掉 LaunchPad UART0 到 XDS110 的 J21/J22 跳线帽，再接 OpenMV UART。
7. 静态瞄准成功后，再上车跑 `MODE_TRACK_AIM_N1/N2`。

## 4. 按键模式

`PB8 MODE` 每按一次切换模式：

```text
0 TRACK_ONLY       只巡线 1 圈
1 AIM_STATIC       原地瞄准
2 TRACK_AIM_N1     巡线 1 圈并瞄准
3 TRACK_AIM_N2     巡线 2 圈并瞄准
4 TRACK_DRAW       巡线 1 圈并画 6cm 圆弧
```

`PB11 START` 启动；运行中再按会停止；停止后再按回到空闲。

## 5. 必调参数

先改这些，不要一上来调所有 PID：

```c
TRACK_OPEN_LOOP_DUTY
POSITION_PID_KP
POSITION_PID_KD
GIMBAL_X_PID_KP
GIMBAL_Y_PID_KP
ENCODER_COUNTS_PER_REV
```

如果瞄准越调越偏，先在 `gimbal.c` 里把 `gimbal_aiming_update()` 的 `dx/dy` 正负号对调。
