# 20260703_212219 MSPM0 + OpenMV 双舵机回正校准版

## 文件

- 当前烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_212219_mspm0_openmv_servo_home_calibration.c`
- OpenMV 脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_162934_openmv_full_selftest.py`

## 接线

- `OpenMV P4 TX -> MSPM0 PA11 RX`
- `OpenMV P5 RX <- MSPM0 PA10 TX`
- `OpenMV GND -> MSPM0 GND`
- `PA14 -> 水平舵机信号`
- `PA17 -> 俯仰舵机信号`
- `舵机红线 -> 5V`
- `舵机黑/棕线 -> 公共 GND`

## 这版新增功能

1. 上电后先把云台打到 HOME 位置，保持约 1.5 秒。
2. HOME 位置由 `g_servo_x_home_us` 和 `g_servo_y_home_us` 表示，当前默认都是 `1500us`。
3. OpenMV 断联或长时间没目标时，舵机慢慢回到 HOME，不再固定回到旧的 1500 中位。
4. `g_servo_calibrate_enable = 1` 时进入手动 HOME 校准模式，暂停追踪，只持续输出 HOME 脉宽。

## Watch 变量

- `g_servo_x_home_us`
- `g_servo_y_home_us`
- `g_servo_x_us`
- `g_servo_y_us`
- `g_servo_calibrate_enable`
- `g_servo_startup_home_done`
- `g_servo_startup_home_frames`
- `g_aim_state`

## g_aim_state 含义

- `0`：OpenMV 串口未连接/超时
- `1`：OpenMV 正常但没看到目标
- `2`：正在追踪
- `3`：已进入中心死区
- `4`：上电 HOME 初始化中
- `5`：手动 HOME 校准模式

## 校准方法

普通三线舵机不能通过信号线读出当前机械角度，所以程序不能“读取你手扭到的位置”。正确方法是用 PWM 脉宽反向校准：

1. CCS 烧录并运行当前 `empty.c`。
2. Watch 添加 `g_servo_calibrate_enable`、`g_servo_x_home_us`、`g_servo_y_home_us`、`g_servo_x_us`、`g_servo_y_us`。
3. 把 `g_servo_calibrate_enable` 改成 `1`。
4. 逐步修改 `g_servo_x_home_us` 和 `g_servo_y_home_us`，例如每次加减 `10`。
5. 看到 OpenMV 云台回到你认为正确的打靶初始方向后，记下这两个数。
6. 把这两个数告诉我，我再把它们写进 `SERVO_X_HOME_US` 和 `SERVO_Y_HOME_US`，形成正式固定版。

注意：校准时不要硬掰正在上电保持的舵机；如果需要手动调整机械角度，先断开舵机 5V。
