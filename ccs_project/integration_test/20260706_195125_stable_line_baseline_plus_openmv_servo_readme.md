# 20260706_195125 stable line baseline plus OpenMV servo

## 目的

基于用户指定的稳定循迹代码：

`C:\Users\1\Desktop\电赛\ccs_project\line_sensor_test\line_follow_sharp_turn_stable_baseline.c`

加入 OpenMV 串口和 PA14/PA17 双舵机追踪。

## 文件

当前 CCS 活动文件：

`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`

时间戳源码：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_195125_stable_line_baseline_plus_openmv_servo.c`

备份：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_195125_before_stable_baseline_plus_openmv_empty.c`

推荐 OpenMV 脚本：

`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260706_163540_openmv_uart_stream_edge_guard_rot180.py`

## 循迹基线恢复内容

来自 `line_follow_sharp_turn_stable_baseline.c`：

- `PWM_LEFT_BASE_TICKS = 620`
- `PWM_RIGHT_BASE_TICKS = 590`
- `PWM_MIN_TICKS = 430`
- `PWM_MAX_TICKS = 850`
- `LINE_KP_DIVISOR = 20`
- `LINE_CORR_LIMIT = 150`
- `LINE_SHARP_ERROR = 1800`
- `LINE_SHARP_KP_DIVISOR = 8`
- `LINE_SHARP_CORR_LIMIT = 380`
- `LINE_CENTER_OFFSET = 0`

急弯速度也恢复为基线等价行为：

- `PWM_SHARP_LEFT_BASE_TICKS = 620`
- `PWM_SHARP_RIGHT_BASE_TICKS = 590`
- `PWM_SHARP_MAX_TICKS = 850`

## 保留的集成内容

- OpenMV UART0：`PA10 TX`、`PA11 RX`
- 舵机：`PA14` 水平，`PA17` 俯仰
- V9 OpenMV 丢目标搜索和边缘目标保护
- 用户调过的舵机范围参数
- 主循环仍采用集成版 20ms 舵机帧节拍，而不是 baseline 的阻塞 `delay_ms(20)` for-loop
- 运行时间保持集成版约 90 秒，不使用 baseline 的约 12 秒测试时长

## 现场测试

1. OpenMV 运行 `20260706_163540_openmv_uart_stream_edge_guard_rot180.py`。
2. CCS 手动烧录当前 `empty.c`。
3. 先架空确认：电机转、PA14/PA17 舵机动、OpenMV 识别红色目标。
4. 再放地上看：循迹是否回到稳定 baseline 手感，同时云台是否能追踪。

本次只编译验证，未自动烧录。
