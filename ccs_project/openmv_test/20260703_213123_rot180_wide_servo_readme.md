# 20260703_213123 OpenMV 倒装修正 + 舵机大范围校准版

## 文件

- 当前烧录文件：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- MSPM0 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_213123_mspm0_openmv_servo_wide_range_rot180.c`
- OpenMV 新脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_213123_openmv_full_selftest_rot180.py`

## 修改原因

现场 OpenMV 处于倒装状态，IDE 画面和坐标都倒置。只改 MSPM0 舵机方向会让云台可能追得动，但 IDE 图像仍然倒，后续调阈值、看目标、做打靶中心校准会非常别扭。

因此这版同时修改两边：

1. OpenMV 端开启 `sensor.set_hmirror(True)` 和 `sensor.set_vflip(True)`，等效把画面旋转 180 度。
2. MSPM0 端把舵机范围从 `1250..1750us` 放宽到 `900..2100us`。
3. MSPM0 默认 HOME 保留现场截图里的 `X=1500us`、`Y=1250us`，避免重新烧录后突然回到旧中位。

## OpenMV 使用步骤

1. 用 OpenMV IDE 打开：
   `C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260703_213123_openmv_full_selftest_rot180.py`
2. 运行后先看 IDE 图像是否已经变正。
3. 如果图像变正，再继续和 MSPM0 联调。
4. 如果图像还是倒，说明实际安装不是 180 度倒装，而可能只是单轴翻转，需要再改 `CAMERA_HMIRROR/CAMERA_VFLIP`。

## MSPM0 校准步骤

Watch 保持：

- `g_servo_x_home_us`
- `g_servo_y_home_us`
- `g_servo_x_us`
- `g_servo_y_us`
- `g_servo_calibrate_enable`
- `g_aim_state`

操作：

1. 烧录当前 `empty.c`。
2. Resume 运行。
3. 把 `g_servo_calibrate_enable` 改成 `1`。
4. 从当前值开始调：
   - `g_servo_x_home_us = 1500`
   - `g_servo_y_home_us = 1250`
5. 每次加减 `10` 或 `20`，找到正确的初始打靶方向。

注意：如果某个方向舵机开始顶住、持续嗡嗡响、机械结构明显卡住，立刻往回调。虽然代码允许 `900..2100us`，但机械结构未必允许全范围。

## 下一步判断

- 如果 IDE 图像已正，但追踪方向反：改 MSPM0 的 `AIM_X_PULSE_SIGN` 或 `AIM_Y_PULSE_SIGN`。
- 如果 IDE 图像仍反：先不要改舵机方向，先修 OpenMV 的 `CAMERA_HMIRROR/CAMERA_VFLIP`。
- 如果 HOME 仍不够：记录顶到哪个值，再判断是继续放宽范围还是调整机械安装角度。
