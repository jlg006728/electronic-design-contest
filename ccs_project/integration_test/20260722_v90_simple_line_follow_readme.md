# V90 极简循迹说明

## 文件

- 活动主程序：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 纯控制逻辑：`C:\Users\1\Desktop\ELECTRIC\ele-car\simple_line_control.h`
- V89改前备份：`20260722_before_v90_simple_line_follow_v89_backup.c`
- V90归档：`20260722_v90_simple_line_follow.c`、`20260722_v90_simple_line_control.h`
- 桌面测试：`..\line_sensor_test\simple_line_control_test.c`

## 当前控制

控制循环只有以下四步：

1. 扫描8路灰度，和上电白底基线做异或，得到红线mask。
2. 用 `X1=-3500 ... X8=+3500` 求红线平均位置误差。
3. 按 `correction = error / 36` 做左右轮比例差速；修正最大为70 ticks。
4. 暂时全灭时按最后一次误差方向以70 ticks找线；连续全灭200帧（约2秒）停车。

已删除起步桥接、弯道检测、弯道状态机、MPU融合、航向保持、出弯制动、曲线前馈、丢线分级补偿和赛道拓扑预测。

保留项：8路灰度扫描、白底自动标定、TB6612方向与PWM底层、安全输出、蜂鸣器静音、PA15状态灯。OpenMV、MPU、舵机、编码器和任务逻辑均不初始化。

## 上电操作

1. 第一次测试把车轮架空，OpenMV和MPU先不参与。
2. 复位前让X1到X8全部照在同一块白色底面上，不能压到红线、黑线或数字框。
3. 复位后PA15闪烁时保持不动，约0.5秒完成50次白底扫描。
4. PA15随后常亮2秒；在这2秒内把车放正，使红线位于X4/X5附近。
5. 2秒结束后电机开始运行：识别到线时PA15常亮；暂时丢线时PA15快闪；连续丢线约2秒后停车。
6. 程序最多运行3000帧，约30秒，届时自动停车。

## Watch验收

- 白底标定：`g_gray_baseline_ready=1`、`g_gray_baseline_uniform=1`，`g_gray_white_baseline_mask`应为`0x00`或`0xFF`。
- 红线居中：常见 `g_gray_line_mask`包含X4/X5，`g_gray_line_error`接近0，左右PWM接近380/383。
- 红线偏右：`g_gray_line_error>0`、`g_line_correction>0`、左PWM增大、右PWM减小。
- 红线偏左：上述符号和两轮变化相反。
- 连续丢线：`g_line_lost_count`递增；到200时 `g_phase=4`、`g_stop_reason=3`、两轮PWM均为0。
- 固件确认：`g_firmware_version=90`。

## 当前仅需调的参数

全部在 `empty.c` 顶部：

- `PWM_LEFT_BASE_TICKS`、`PWM_RIGHT_BASE_TICKS`：基础速度。
- `LINE_KP_DIVISOR`：越小修正越强，越大越柔和。
- `LINE_CORRECTION_LIMIT`：有线时最大差速。
- `LINE_LOST_SEARCH_TICKS`：短时丢线找回力度。
- `LINE_LOST_TIMEOUT_FRAMES`：连续丢线停车时间。

首轮不要同时调整多个参数。先验证传感器方向正确，再只调基础速度，最后只调比例强度。

## 验证结果

- Host GCC单元测试：6/6通过；纯控制头文件行覆盖率88.24%，测试整体行覆盖率95.18%。
- TI Arm Clang：活动工程执行Clean Build，编译和链接成功，无警告，生成 `Debug\ele-car.out`。
- 尚未烧录，实车方向和机械差速仍需按上述架空流程验证。
