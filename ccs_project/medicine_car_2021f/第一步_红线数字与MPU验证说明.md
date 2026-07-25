# 2021 F题第一步：红线、数字与新陀螺仪验收

本阶段只验证传感器链路和观测量，不驱动电机、不接入整车状态机。目录内的程序不会覆盖正在使用的 `ELECTRIC/ele-car/empty.c`。

## 文件

- [20260722_openmv_red_line_digit_validation.py](20260722_openmv_red_line_digit_validation.py)：OpenMV 红线检测、黑框定位、现场数字模板学习和识别。
- [20260722_mspm0_new_mpu_validation.c](20260722_mspm0_new_mpu_validation.c)：MSPM0 独立验收新 MPU6050/MPU6500 兼容模块。
- [vision_core.py](vision_core.py)：桌面可测试的红线拟合、Hamming 距离、多帧投票和 CRC 核心。
- [tests/test_vision_core.py](tests/test_vision_core.py)：回归测试。

## 关键接线

### OpenMV

第一阶段只用 USB 连接 OpenMV 与电脑，在 OpenMV IDE 的画面和串口终端观察结果。不要同时把 OpenMV UART 接到正在运行的 MSPM0 整车程序，避免把调试帧误当控制帧。

当前 OpenMV H7 实测固件为5.0.0；脚本已兼容5.x的属性式 `blob/rect` API，并按5.0底层绑定把绘图/取像素坐标作为tuple传入。若看到 `TypeError: 'int' object isn't callable`，说明仍使用 `blob.pixels()`/`blob.cx()`；若看到 `TypeError: object 'int' isn't a tuple or list`，说明仍有 `draw_cross(x,y)`或`get_pixel(x,y)`等分散整数调用。两种情况都应整文件替换为本目录最新版，不要只修改弹窗附近一行。

OpenMV 镜头固定斜向下，先保持现有机械安装方向；脚本默认 `CAMERA_HMIRROR=True`、`CAMERA_VFLIP=True`，若画面左右/上下反了只改这两个量。

### 新陀螺仪

| MPU 模块 | MSPM0G3507 | 说明 |
|---|---|---|
| VCC | 3.3V | 禁止接5V |
| GND | GND | 与MSPM0共地 |
| SDA | PA0 / I2C0 SDA | 线尽量短 |
| SCL | PA1 / I2C0 SCL | 线尽量短 |
| AD0 | GND | 地址固定为0x68 |
| INT、XDA、XCL | 悬空 | 本验收不使用 |

先断电接线，再上电；禁止带电插拔。模块若没有 I2C 上拉，才在 SDA/SCL 各加约 4.7 kΩ 到 3.3V。独立验收时断开电机驱动信号、PWM、舵机、灰度板和 OpenMV UART。历史 V61 工程的 LaunchPad 跳帽为 `J4 OFF、J19 1:2、J20 1:2`，实际装配以当前板卡丝印为准。

## A. 红线验证

1. 在脚本顶部保持 `MODE = "red"`，用 OpenMV IDE 运行。
2. 在镜头视野内放一段红胶带/红线，先放在画面中部，再分别放在近端、远端和左右偏移位置。
3. 终端观察 `$RED,...*` 帧；画面绿色十字为近端/远端估计，绿色线为多水平 ROI 拟合结果。红灯表示看到了红色但有效带数不足，蓝灯为心跳。
4. 每个位置至少采集 100 帧。记录 `valid`、`bands`、`near`、`far`、`near_off`、`far_off`、`angle10`、`fps10`。

输出字段：`angle10` 是角度乘10，`near_off/far_off` 是相对画面中心的像素偏移，`pixels` 是各红色 ROI 的像素数。帧末尾 CRC8 用于以后接 UART 时发现乱码；本阶段只需看文本。

红线阶段门槛：

- 有红线且光照稳定时，100帧中 `valid=1` 不少于95帧，且通常 `bands>=3`。
- 无红线白底 100 帧中误报不超过5帧。
- 红线固定不动时 `near_off` 的峰峰值不超过10像素，`angle10` 的峰峰值不超过60（约6°）。
- `fps10` 目标不低于100（10 FPS）；若低于80，先减少 ROI 数量或降低分辨率，不进入整车联调。

## B. 数字框和数字模板

题目附带的 `数字字模.pdf` 当前未在工作区找到，因此第一阶段不假设字体，使用现场模板学习。模板不是“训练模型”，而是数字框内部固定 16×12 网格的黑白签名；同一套纸卡、镜头高度和光照下重复性最好。

### 1. 学习1-8

1. 改为 `MODE = "digit_capture"`，设置 `DIGIT_LABEL = "1"`。
2. 把 1 号数字框完整放入视野，保持小车和卡片不动，运行脚本。
3. 终端 `DIGIT_CAPTURE` 的 `stable` 达到12后自动写入 OpenMV 文件系统 `digit_templates.txt` 并结束本次脚本。
4. 依次把 `DIGIT_LABEL` 改成 `"2"` 到 `"8"`，重复运行。每次运行前不要删除已有 `digit_templates.txt`。

若 `stable` 长期为0，优先调整镜头、光照和黑框阈值，不要先改数字识别逻辑；画面中必须能看到绿色/橙色数字框候选。

### 2. 重复识别

1. 改为 `MODE = "digit_verify"` 并运行。
2. 每个数字随机放置、重复10次，共80次；每次保持框完整进入画面约1秒。
3. 终端观察 `label`（当前帧）、`vote`（5帧投票结果）、`dist`（Hamming距离）和 `margin`（第一、第二名距离差）。绿色灯表示投票稳定，橙灯表示框找到了但识别不可靠。

数字阶段门槛：

- 80次中 `vote` 正确不少于72次（90%）才算达到“可继续优化”的最低线；目标是不少于76次（95%）。
- 正确识别时 `dist` 建议不超过40，`margin` 建议至少3；若大量接近 `dist=55` 或 `margin=1`，说明光照、框定位或模板姿态不一致。
- 不能把 `label=none` 当作错误数字；低距离/低间隔时拒识是保护机制，后续再做透视校正和正式导航。

## C. 新 MPU 独立验收

1. 在 CCS 中复制一份当前 V61 兼容的 MSPM0 工程到临时目录，只在临时副本把 `empty.c` 替换为 `20260722_mspm0_new_mpu_validation.c`；不要覆盖活动工程。若工程通过 SysConfig 生成 `ti_msp_dl_config.h/.c`，保留原工程的 PA0/PA1 I2C 配置。
2. Clean Build 后烧录。上电后保持模块静止约3秒，程序先读 `WHO_AM_I`，再采500个样本做 Z 轴零偏标定。
3. 在 Watch 中观察：`g_mpu_state`、`g_mpu_error`、`g_mpu_link_ok`、`g_mpu_who_am_i`、`g_mpu_device_type`、`g_gyro_z_bias_raw`、`g_gyro_z_mdps`、`g_yaw_mdeg`、`g_mpu_read_fail_count`、`g_mpu_read_fail_total`、`g_stationary_sample_count`、`g_motion_sample_count`。
4. 标定结束后先静止60秒，再手动绕 Z 轴左右各转约90°，最后放回静止。不要接电机。

MPU 阶段通过条件：

- `g_mpu_state=3`、`g_mpu_link_ok=1`、`g_mpu_error=0`；`g_mpu_who_am_i` 必须为 `0x68` 或 `0x70`，且 `g_mpu_device_type` 对应1或2。
- 标定500个样本完成，I2C 连续运行期间 `g_mpu_read_fail_total=0` 为目标；偶发失败必须小于总样本的0.1%，且不能连续增长。
- 静止时 `abs(g_gyro_z_mdps)` 目标不超过1000 mdps，程序保护阈值为3000 mdps；`g_stationary_sample_count` 应持续增加，不能反复清零。
- 手动旋转时 `g_motion_sample_count` 增加，`g_gyro_z_mdps` 符号应随左右旋转改变，`g_yaw_mdeg` 应单调变化；回到静止后角速度应回到阈值内。
- 静止60秒的 `g_yaw_mdeg` 漂移目标不超过5000 mdeg（5°）。超过10°先重新固定模块、检查电源和零偏，不进入整车融合。

## 完成判定和下一步

只有红线、数字、MPU 三个独立门槛都通过，才把 OpenMV 的红线几何量通过明确的 UART 帧接到 MSPM0 状态机。当前程序不包含药品重量检测、卸药执行器、门口停车、路线拓扑、第二辆车、黄灯或无线协同；这些留到视觉和惯性链路验收后逐项增加。
