# 20260707_140926 全程跟踪主动搜索 V2

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_140926_full_course_tracking_active_search_v2.c`
- V2 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_140850_before_full_course_tracking_v2_empty.c`
- V1 源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_135432_full_course_tracking_only_v1.c`

## V1 真实逻辑

V1 只关闭了 B/C/D/A 任务点：

```c
#define MISSION_ENABLE_B_AIM        0U
#define MISSION_ENABLE_POINT_PROMPTS 0U
```

因此 V1 的行为是：

1. 小车持续循迹，不在 B 点停车。
2. 不做 C/D/A 提示。
3. OpenMV 仍按原来的静态/半静态追踪逻辑工作。

V1 不是重新设计的全程追踪算法。原追踪逻辑里有两条保护策略，对全程运动场景偏保守：

1. 底盘处于急弯/丢线找回时，云台不主动搜索目标。
2. 目标靠近画面边缘时，MSPM0 把它当作不可靠目标拒绝追踪。

这会导致：小车一过弯或目标到画面边缘，云台可能保持不动/搜索慢，目标离开视野后难以找回。

## V2 改动

V2 只改 OpenMV/舵机追踪策略，不改循迹电机参数。

```c
#define AIM_X_DIVISOR                 2
#define AIM_MAX_STEP_US               60
#define AIM_SEARCH_START_FRAMES       6U
#define AIM_SEARCH_STEP_US            35
#define FULL_TRACK_SEARCH_DURING_TURN 1U
#define FULL_TRACK_ACCEPT_EDGE_TARGET 1U
```

含义：

1. 水平追踪响应更快。
2. 单帧最大舵机步长从 `45us` 提到 `60us`。
3. 丢目标后从约 `14` 帧等待改为 `6` 帧等待，更早开始搜索。
4. 搜索步长从 `22us` 提到 `35us`。
5. 车身转弯时，云台仍允许搜索目标。
6. 目标靠近画面边缘时，不再直接拒绝追踪。

## 测试重点

1. 过弯时云台是否还会明显停住不找目标。
2. 红色目标到画面边缘时，舵机是否继续朝目标方向追。
3. 椭圆弯后半段丢目标后，是否比 V1 更快扫回来。
4. 如果云台抖动明显，下一步先把 `AIM_MAX_STEP_US 60 -> 50`。
5. 如果仍找回慢，下一步把 `AIM_SEARCH_STEP_US 35 -> 45`。

## Watch 建议

- `g_omv_target_detected`
- `g_omv_pixels`
- `g_omv_cx`
- `g_omv_cy`
- `g_aim_state`
- `g_aim_search_active`
- `g_aim_x_step_us`
- `g_servo_x_us`
- `g_servo_y_us`
- `g_turn_mode`
- `g_diag_stage`
- `g_diag_edge_target_count`
- `g_diag_turn_hold_count`

