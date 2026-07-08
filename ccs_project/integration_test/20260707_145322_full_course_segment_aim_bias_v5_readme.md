# 20260707_145322 全程跟踪赛段预瞄偏置 V5

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_145322_full_course_segment_aim_bias_v5.c`
- 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_145140_before_segment_aim_bias_empty.c`
- OpenMV 脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260707_144221_openmv_uart_stream_qvga_far_stable_v2.py`

## 本版目标

在 OpenMV 远距离识别已经改善后，给云台增加“按赛段预瞄”：

- AB 段：正常居中/视觉追踪。
- BC 段：丢目标时持续向左偏，优先找左侧靶。
- CD 段：丢目标时才开始向右回到中位。
- DA 段：丢目标时持续右偏。

这里的“左偏/右偏”指云台水平舵机预瞄，不是小车循迹偏线。

## 关键参数

```c
#define COURSE_SEGMENT_AIM_ENABLE   1U
#define COURSE_SEGMENT_AB           1U
#define COURSE_SEGMENT_BC           2U
#define COURSE_SEGMENT_CD           3U
#define COURSE_SEGMENT_DA           4U
#define COURSE_SEGMENT_BC_LEFT_US   2450U
#define COURSE_SEGMENT_CD_RETURN_US SERVO_X_HOME_US
#define COURSE_SEGMENT_DA_RIGHT_US  800U
#define COURSE_SEGMENT_AIM_STEP_US  35
```

赛段边界暂时复用原任务点采样：

```c
#define MISSION_B_STOP_SAMPLE       180U
#define MISSION_C_PROMPT_SAMPLE     360U
#define MISSION_D_PROMPT_SAMPLE     540U
#define MISSION_A_FINISH_SAMPLE     720U
```

注意：当前没有编码器距离闭环，赛段判断仍然靠 `g_run_sample`。如果现场发现 BC/CD/DA 切换早了或晚了，优先改这些 sample 边界。

## 逻辑说明

视觉识别到目标时：

- 仍然按 OpenMV `cx/cy` 闭环追踪。

视觉丢目标时：

- 不再盲目左右扫。
- 根据 `g_run_sample` 判断当前段：
  - AB：目标 X = HOME
  - BC：目标 X = `2450us`，向左预瞄
  - CD：目标 X = HOME，开始向右回
  - DA：目标 X = `800us`，向右预瞄
- 每帧最多移动 `35us`，避免突然甩头。

## Watch 建议

- `g_course_segment`
- `g_segment_aim_x_us`
- `g_run_sample`
- `g_omv_target_detected`
- `g_omv_pixels`
- `g_omv_cx`
- `g_omv_cy`
- `g_aim_state`
- `g_aim_search_active`
- `g_aim_x_step_us`
- `g_servo_x_us`
- `g_servo_y_us`

`g_course_segment` 含义：

- `1` = AB
- `2` = BC
- `3` = CD
- `4` = DA

## 现场调参方向

如果 BC 左偏不够：

```c
COURSE_SEGMENT_BC_LEFT_US 2450U -> 2550U
```

如果 BC 左偏过头：

```c
COURSE_SEGMENT_BC_LEFT_US 2450U -> 2350U
```

如果 DA 右偏不够：

```c
COURSE_SEGMENT_DA_RIGHT_US 800U -> 650U
```

如果 DA 右偏过头：

```c
COURSE_SEGMENT_DA_RIGHT_US 800U -> 950U
```

如果赛段切换太早/太晚：

改 `MISSION_B_STOP_SAMPLE / MISSION_C_PROMPT_SAMPLE / MISSION_D_PROMPT_SAMPLE`。

