# 20260706_201926 open loop balance fix

## 现象

用户判断：小车每次回正后都会往右前方走，然后再回正。这个现象更像回正后基础 PWM 不平衡，而不是右偏回正力度不够。

当前上一版参数：

- `PWM_LEFT_BASE_TICKS = 408`
- `PWM_RIGHT_BASE_TICKS = 398`

当 `g_line_correction` 回到 0 时，左轮仍比右轮快，车自然会往右前方走，然后靠循迹再次拉回，形成周期性右偏。

## 改动

本版保持平均速度不变，把基础 PWM 改成左右相等：

- `PWM_LEFT_BASE_TICKS 408 -> 403`
- `PWM_RIGHT_BASE_TICKS 398 -> 403`
- `PWM_LEFT_BOOST_TICKS 408 -> 403`
- `PWM_RIGHT_BOOST_TICKS 398 -> 403`
- `PWM_SHARP_LEFT_BASE_TICKS 408 -> 403`
- `PWM_SHARP_RIGHT_BASE_TICKS 398 -> 403`

同时撤掉上一版的右偏回正增强：

- 删除 `LINE_RIGHT_OVERSHOOT_BOOST_NUM`
- 删除 `LINE_RIGHT_OVERSHOOT_BOOST_DEN`
- 删除 `boost_right_overshoot_correction()`
- 普通循迹和急弯循迹恢复原始 baseline 修正

## 保留

- 当前速度档平均值仍为 `403`
- `PWM_MIN_TICKS = 287`
- `PWM_MAX_TICKS = 567`
- baseline 循迹参数 `20/150`
- 急弯参数 `1800/8/380`
- OpenMV/舵机逻辑

## 测试判断

- 如果回正后不再固定往右走：说明根因确实是左轮基础 PWM 偏高。
- 如果仍轻微右偏：下一步不要恢复右偏增强，优先改成 `400/406`。
- 如果变成左偏：改成 `406/400`。

本次只编译验证，未自动烧录。
