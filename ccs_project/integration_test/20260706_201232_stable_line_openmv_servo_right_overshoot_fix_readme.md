# 20260706_201232 right overshoot fix

## 目的

用户视频显示：小车明显向右多偏出一点后再回正。本版不再继续改左右基础 PWM，而是在循迹修正量上对“右偏后回正”方向做小幅加强。

## 文件

当前 CCS 活动文件：

`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`

时间戳源码：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_201232_stable_line_openmv_servo_right_overshoot_fix.c`

备份：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260706_201232_before_right_overshoot_fix_empty.c`

## 改动

新增：

```c
#define LINE_RIGHT_OVERSHOOT_BOOST_NUM 6
#define LINE_RIGHT_OVERSHOOT_BOOST_DEN 5
```

含义：当修正量为负时，将修正量放大到 `1.2` 倍，并仍受原来的修正上限限制。

接入位置：

- 普通循迹修正
- 急弯/丢线找回修正

保留：

- 当前速度：`408/398`
- `PWM_MIN=287`、`PWM_MAX=567`
- baseline 循迹参数 `20/150`
- 急弯参数 `1800/8/380`
- OpenMV/舵机逻辑

## 现场判断

- 如果右偏明显改善且不抖：保留。
- 如果变成左偏或右弯内侧过早切线：把 `6/5` 改成 `11/10`。
- 如果几乎没变化：把 `6/5` 改成 `13/10`。
- 如果现象反而更严重：说明右偏对应的误差方向判断反了，应改为增强正修正方向。

本次只编译验证，未自动烧录。
