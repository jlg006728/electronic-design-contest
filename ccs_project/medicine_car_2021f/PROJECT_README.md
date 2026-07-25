# 2021 F题：智能送药小车模拟题

本目录只对应 2021 年 F 题模拟路线，不属于 2025 E题自动瞄准工程。

## 目录约定

- `active_project/`：当前 `C:\Users\1\Desktop\ELECTRIC\mock_problem` 的可复现 CCS 源码快照；只保留源码、头文件、SysConfig、CCS 工程元数据，不保留 `Debug/` 编译产物。
- `backup_code/`：按日期和固件版本保存的单文件 `empty.c`，用户可手动复制覆盖活动工程。
- `tests/`：源码契约、路线 Host 和视觉核心测试。
- `archive/`：V93/V94 的完整软件验证归档及其测试证据。

## 最新状态

- 活动工作区快照为 V104。
- 最新备份为 `backup_code/20260725_v105_video_gated_first_junction_empty.c`。
- V105 相对 V104 的变化仅用于首个路口：50 cm 编码器武装、X4/X5 中央传感器同时有效、连续 3 帧确认；17.5 cm 转弯中心补偿、V98 负反馈方向、固定白底、MPU 转弯和首转自动停车均保留。

## 验证方式

在 CCS 中复制所需备份覆盖活动工程的 `empty.c`，然后 `Build → Debug → Run`。仓库中的 `.out`、`.map`、`.o`、`.exe` 等均为可重建产物，不作为源代码进度提交。
