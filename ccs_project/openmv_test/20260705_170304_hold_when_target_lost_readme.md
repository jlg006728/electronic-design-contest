# 20260705_170304 丢目标保持当前位置版

## 文件

- 当前 CCS 烧录源：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260705_170304_mspm0_openmv_track_hold_when_target_lost.c`
- OpenMV 推荐脚本：`C:\Users\1\Desktop\电赛\ccs_project\openmv_test\20260705_145746_openmv_uart_stream_watchdog_no_print_rot180.py`

## 改动

暂时删除“丢失目标一段时间后舵机慢慢回 HOME”的逻辑。

现在行为：

- 识别到红色目标：继续追踪。
- 短时间没识别到目标：舵机保持最后位置。
- 长时间没识别到目标：仍保持最后位置，只重置坐标滤波，避免重新识别时被旧坐标拖慢。

保留：

- 上电回 HOME。
- OpenMV 链路异常时的诊断变量。
- X 轴 `1500us` 对称中位设置。

