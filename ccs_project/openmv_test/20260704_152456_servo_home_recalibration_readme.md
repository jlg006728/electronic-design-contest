# 20260704_152456 双舵机 HOME 重新校准版

## 文件

- 当前烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260704_152456_mspm0_servo_home_recalibration.c`

## 使用场景

舵机刚才到过机械临界值，且舵机方向/机械安装方向发生变化。此时不能继续运行 OpenMV 追踪版，否则可能再次把舵机推到边界。

本版是纯校准版：

- 不追踪 OpenMV 目标。
- 不根据 `cx/cy` 改舵机。
- 只持续输出 Watch 里的 `g_servo_x_home_us/g_servo_y_home_us`。

## 初始值

- `g_servo_x_home_us = 1500`
- `g_servo_y_home_us = 1500`
- `g_aim_state = 5`
- `g_servo_calibrate_enable = 1`

这里从 `1500/1500` 中位重新开始，不沿用上一轮 `Y=580`，因为机械方向已经改变，旧 HOME 可能变成危险位置。

## 可调范围

- `g_servo_x_home_us`: `500..2500`
- `g_servo_y_home_us`: `500..2500`

注意：这是校准探索范围，不代表机械结构允许全范围。只要舵机持续嗡鸣、发热、顶住机械结构，立刻往回调。

## Watch 变量

- `g_servo_x_home_us`
- `g_servo_y_home_us`
- `g_servo_x_us`
- `g_servo_y_us`
- `g_servo_frame_count`
- `g_aim_state`
- `g_servo_calibrate_enable`

## 校准步骤

1. 烧录当前 `empty.c`。
2. Resume 运行。
3. Watch 确认：
   - `g_aim_state = 5`
   - `g_servo_calibrate_enable = 1`
   - `g_servo_frame_count` 持续增加
4. 从 `1500/1500` 开始调：
   - 先调 `g_servo_x_home_us`
   - 再调 `g_servo_y_home_us`
5. 每次建议加减 `20` 或 `50`，不要一次大跳。
6. 找到 OpenMV 初始瞄准方向后，记录最终两个数。
7. 把最终 `X/Y` 数值告诉我，我再生成新的追踪版。

## 重要

这版是校准版，不用管 OpenMV 是否检测目标，也不用管串口 Watch。校准完成前不要跑追踪版。
