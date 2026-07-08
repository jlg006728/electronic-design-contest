# 20260707_154420 编码器距离版 V10

## 文件

- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_154420_course_point_stop_buzzer_encoder_distance_v10.c`
- 基于 V8：关键点停车 + OpenMV + 双舵机 + 5V 有源蜂鸣器。

## 编码器接线

沿用历史已验证接线：

```text
左轮 E1A -> PB12
左轮 E1B -> PB13
右轮 E2A -> PB14
右轮 E2B -> PB15
GND 共地
```

历史方向结论：

```text
左轮前进时 encoder count 为负，软件取反。
右轮前进时 encoder count 为正。
```

## 核心逻辑

V10 不再用 `g_run_sample` 触发 ABCD 点位，而是使用左右编码器前进边沿平均值：

```c
g_encoder_distance_edges = (left_forward + right_forward) / 2
```

点位距离：

```c
#define COURSE_POINT_B_CM           100U
#define COURSE_BC_MID_CM            163U
#define COURSE_POINT_C_CM           226U
#define COURSE_POINT_D_CM           326U
#define COURSE_LAP_CM               451U
```

编码器换算：

```c
#define ENCODER_EDGES_PER_CM        1471U
```

这个默认值按 `500线GMR * 1:30`，并假设轮径约 `65mm`、A相双边沿计数估算：

```text
500 * 30 * 2 / (pi * 6.5cm) ≈ 1470 edges/cm
```

## 重要校准

`ENCODER_EDGES_PER_CM` 必须现场校准。方法：

1. 烧录 V10。
2. 架空或低速确认 `g_left_edges/g_right_edges` 都会变化。
3. 放地跑到实际 100cm。
4. 看 `g_encoder_distance_edges`。
5. 用下面公式改：

```text
ENCODER_EDGES_PER_CM = g_encoder_distance_edges / 100
```

如果不校准，编码器版只是“有距离闭环结构”，不保证第一次就准。

## Watch 建议

```c
g_left_count
g_right_count
g_left_edges
g_right_edges
g_encoder_distance_edges
g_encoder_lap_edges
g_point_stop_target_edges
g_point_hold_id
g_point_hold_frames_left
```

## 验证

- CCS `gmake all` 编译通过。
- 未自动烧录。
