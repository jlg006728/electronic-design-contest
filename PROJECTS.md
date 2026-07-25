# 项目分区与来源

本仓库同时保存多套电赛工程。下面的分区是有意保留的，不能把不同赛题的
`empty.c`、接线和算法互相覆盖。

## 2025 E题：自动寻迹与视觉瞄准

- 资料与理论：`25年电赛E题/`、`知识库/`。
- MSPM0/电机/云台工程：`ccs_project/e_car/`。
- 旧版活动工程与循迹实验：`ccs_project/ele-car_active_project/`、`ccs_project/integration_test/`、`ccs_project/line_sensor_test/`、`ccs_project/mpu6050_test/`。
- 版本范围：V50–V91；其中 V89 是会话 `019e7371-af12-7d03-a88c-240a230f2d96` 中的控制变量阶段记录。

## 2021 F题：智能送药小车模拟题

- 工程、路线状态机、测试和备份：`ccs_project/medicine_car_2021f/`。
- 当前 CCS 工作区快照：`ccs_project/medicine_car_2021f/active_project/`。
- 固件备份范围：V95–V105；V105 是基于两段实车录像增加首路口距离门控、中央传感器门控和连续确认的最新备份。
- 不要将此目录中的 `medicine_route.h`、`medicine_mission.h` 与 E题的循迹工程混用。

## OpenMV 与通用硬件验证

- OpenMV/MSPM0 联调：`ccs_project/openmv_test/`。
- 蜂鸣器、编码器、舵机和电机隔离测试：`ccs_project/buzzer_test/`、`ccs_project/encoder_test/`、`ccs_project/servo_test/`、`ccs_project/motor_test/`。
- OpenMV 会话记录和功能汇总保留在仓库根目录的 `20260717_OpenMV...md` 与 `20260717_功能更新总结.md`。

## 记录与证据

- 全局决策与稳定参数：`项目记忆.md`。
- 时间线、版本、测试、视频诊断：`过程记录.md`。
- 视频逐帧证据：`视频分析/`；只提交可复核的抽帧和报告，不提交原始临时视频及 CCS 编译产物。
