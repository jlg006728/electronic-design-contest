# 20260705_152451 水平舵机 1500us 对称中位版

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260705_152451_mspm0_openmv_track_x_home_1500_symmetric.c`
- OpenMV 推荐脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260705_145746_openmv_uart_stream_watchdog_no_print_rot180.py`

## 改动

只改水平舵机 HOME：

```c
#define SERVO_X_HOME_US 1500U
```

保持：

```c
#define SERVO_X_MIN_US  500U
#define SERVO_X_MAX_US  2500U
```

这样水平舵机以 `1500us` 为中心，左右软件余量对称。

## 现场操作

1. 手动烧录当前 `empty.c`。
2. 上电后水平舵机会回到 `1500us`。
3. 松开水平舵机摆臂/云台固定结构。
4. 手动把 OpenMV 镜头调到正前方。
5. 锁紧机械结构。
6. 再做红色目标追踪测试。

## 注意

如果不做机械回中，单纯把 HOME 改到 `1500us` 会导致镜头初始方向变化；这是预期现象。正确目标是让“镜头正前方”物理上对应 `1500us`。

