# 20260706_234346 参考代码学习与当前集成优化建议

## 当前备份

- 备份目录：`C:\Users\1\Desktop\电赛备份\backup_20260706_234346_current_integration`
- 已备份：
  - `C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
  - `20260706_220410_line_follow_b_aim_left_preset_v3.c`
  - `20260706_220410_line_follow_b_aim_left_preset_v3_readme.md`
  - `20260706_163540_openmv_uart_stream_edge_guard_rot180.py`
  - `项目记忆.md`
  - `过程记录.md`
  - `知识库\调试复盘_判断失误记录.md`

## 已读参考代码

- `C:\Users\1\Desktop\电赛\参考代码\校赛_2026-06-29\25E题底盘开源\25E题底盘国一开源\ApplicationLayer\Task.c`
- `C:\Users\1\Desktop\电赛\参考代码\校赛_2026-06-29\25E题底盘开源\25E题底盘国一开源\FunctionalModule\Buzzer.c`
- `C:\Users\1\Desktop\电赛\参考代码\校赛_2026-06-29\25E题底盘开源\25E题底盘国一开源\FunctionalModule\GraySensor.c`
- `C:\Users\1\Desktop\电赛\参考代码\校赛_2026-06-29\25E题底盘开源\25E题底盘国一开源\FunctionalModule\MotorsCtrl.c`
- `C:\Users\1\Desktop\电赛\参考代码\校赛_2026-06-29\25E题底盘开源\25E题底盘国一开源\user\main.c`
- `C:\Users\1\Desktop\电赛\参考代码\校赛_2026-06-29\25E题底盘开源\25E题底盘国一开源\user\datatype.h`

## 可直接借鉴的思想

1. **任务状态机要比单纯循迹更高一层**

   参考代码把直角弯拆成“检测到直角 -> 前进一段车身长度 -> 强制转向 -> 重新捕线 -> 保护期”。这比只靠 `line_error` 立即修正更稳定。

   当前代码已有 `MISSION_B/C/D/A` 和 `g_turn_mode`，但直角弯仍散落在 `line_follow_update()` 里。后续可以增加轻量 `corner_state`，不要急着拆多文件。

2. **提示音/提示灯应做成非阻塞事件队列**

   参考 `BuzzerDataUpdate()`：不在任务函数里 `delay`，只设置“还要响几次”，再由低频周期任务慢慢开关 PWM。

   当前若新增蜂鸣器，应该做：

   - `prompt_request(beeps, led_flashes)`
   - `prompt_update_once_per_servo_frame()`
   - 不阻塞循迹、OpenMV、舵机 PWM。

3. **不同阶段可以用不同速度**

   参考代码按 `RightAngleCount` 改 `AverageSpeed`。当前我们也已经有普通速度、急弯速度、B 点停车瞄准。后续可以把速度和任务点绑定：

   - AB 段：为 B 点瞄准准备，速度偏稳
   - B 点：停车静态瞄准
   - 弯道：低速更稳
   - 长直道：可略快，但要防止 OpenMV 追丢

4. **编码器是后续精度升级方向，但不是当前立刻改**

   参考代码大量依赖 `EncoderTotalLengthGet()` 做距离判断和圈数闭环。当前集成版用 `g_run_sample` 估计 B/C/D/A，能跑通但受电压、负重、地面摩擦影响大。

   下一轮大升级应考虑把编码器接回并稳定读数，用编码器距离替代 `MISSION_B_STOP_SAMPLE` 等采样计数。

## 不应直接照搬的部分

1. 参考代码的灰度传感器是 ADC/多路选择/归一化方案，我们当前是 7 路 TCRT5000 数字 DO，不能直接搬 `GraySensorDataUpdate()`。
2. 参考代码电机是 4 电机/编码器 PID，我们当前是 TB6612 两路差速电机，不能直接搬 `MotorsCtrl.c`。
3. 参考代码蜂鸣器使用 `TIMG12 PA25` PWM；我们当前没有确认可用蜂鸣器，也没确认 `PA25` 接线可达，不能直接搬 SysConfig。
4. 当前舵机 PWM 是软件阻塞 20ms 脉冲，这是当前集成版最大的结构限制。参考工程的 200Hz 控制节拍在我们这里暂时达不到。

## 建议的下一步优化顺序

1. **先测试当前 V3 B 点左预瞄**

   目标：确认 `MISSION_B_AIM_X_PRESET_US=2450` 是否把靶带进视野。若不够改 2550，过头改 2300。

2. **确定声音器件**

   你刚拍的 Sound Sensor 是麦克风传感器，不是蜂鸣器。若要声光提示，需要新增有源蜂鸣器模块或无源蜂鸣器加 PWM/驱动。

3. **加非阻塞提示层**

   在 `empty.c` 中加入提示事件：

   - 上电完成：短闪/短响 1 次
   - 到 B 停车：2 次
   - B 点瞄准成功：3 次
   - B 点瞄准超时：长亮/长响
   - C/D 点经过：不同次数提示
   - 最终 A 停车：持续亮灯或多次提示

4. **把 B/C/D/A 的采样计数改成可调表**

   当前宏是分散的：

   - `MISSION_B_STOP_SAMPLE`
   - `MISSION_C_PROMPT_SAMPLE`
   - `MISSION_D_PROMPT_SAMPLE`
   - `MISSION_A_FINISH_SAMPLE`

   后续可以整理成统一任务点逻辑，减少调参时漏改。

5. **中期升级：编码器距离替代采样计数**

   等当前 B 点瞄准和声光跑通后，再回到编码器。这样 B/C/D/A 点位不会随电池电压和负重变化大幅漂移。

6. **长期升级：舵机 PWM 硬件化**

   当前软件舵机 PWM 每 20ms 阻塞一帧，会限制循迹更新频率。若后续速度提高、打靶要求更高，应改用硬件定时器输出 PA14/PA17，主循环专心做循迹和 OpenMV。

