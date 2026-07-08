# 电机不动红外传感器静态测试

时间：2026-07-07 20:00

## 文件

- 当前 CCS 活动源码：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\line_sensor_test\20260707_200036_static_line_sensor_motor_off_test.c`
- 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\line_sensor_test\20260707_200036_before_sensor_static_test_current_backup.c`

## 安全行为

电机不会动：

```text
PB0/PB1/PB2/PB3/PB4 全部输出 LOW
PWM 不初始化
TB6612 STBY = LOW
```

程序只循环读取 7 路红外。

## Watch 变量

原始 GPIO 电平：

```c
g_raw_s1
g_raw_s2
g_raw_s3
g_raw_s4
g_raw_s5
g_raw_s6
g_raw_s7
g_line_raw_mask
```

归一化后的“黑线有效”状态：

```c
g_s1
g_s2
g_s3
g_s4
g_s5
g_s6
g_s7
g_line_mask
```

位置计算：

```c
g_line_valid
g_line_active_count
g_line_error
g_line_weighted_sum
g_loop_count
```

LED：

```text
任意一个归一化传感器有效时，PA15 LED 亮。
没有检测到线时，PA15 LED 灭。
```

## 测试方法

1. 放白底：
   - 记录 `g_raw_s1..g_raw_s7` 和 `g_s1..g_s7`。
2. 用黑线依次经过 S1 到 S7：
   - 对应那一路应该从 0 变 1。
3. 放到 D->A 有线段，车身摆正：
   - `g_line_valid` 应为 1。
   - 中间压线时 `g_line_error` 应接近 0。

## 如果读反

如果现场发现白底为 1、黑线为 0，则把：

```c
#define LINE_SENSOR_ACTIVE_HIGH     1U
```

改为：

```c
#define LINE_SENSOR_ACTIVE_HIGH     0U
```

## 验证

已执行 clean build 并编译通过。按当前规则：不自动烧录，由用户手动烧录。
