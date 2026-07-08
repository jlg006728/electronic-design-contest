# V19：恢复动态追踪集成版并保留蜂鸣器

时间：2026-07-07 19:05

## 文件

- 当前 CCS 活动源码：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_190540_v19_restore_dynamic_tracking_with_buzzer.c`
- 改前蜂鸣器测试备份：`C:\Users\1\Desktop\电赛\ccs_project\buzzer_test\20260707_190540_before_restore_current_buzzer_test_backup.c`

## 基底

本版恢复自 V18：

`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_173433_v18_segmented_right_side_preaim.c`

## 恢复原因

蜂鸣器不响的硬件原因已确认：蜂鸣器正负极接反。

PB11 蜂鸣器测试程序已完成定位，因此当前 `empty.c` 从蜂鸣器隔离测试恢复到动态追踪集成版。

## 当前行为

保留 V18 动态追踪/分段预瞄逻辑：

- A/B 前保持左侧预瞄。
- B 后快速切到右侧预瞄。
- B 到 CD 中点回到右 90 度附近。
- CD 中点到 D 再向右。
- D 到 A 回 HOME。

保留蜂鸣器逻辑：

- 上电 1 声。
- B 点 2 声。
- OpenMV 瞄准稳定 3 声。
- C 点 4 声。
- D 点 5 声。
- A 点 1 声。
- 点位停车未锁定目标时长响。

## 蜂鸣器接线提醒

确认后的极性：

```text
蜂鸣器 + 接正极/驱动输出高端
蜂鸣器 - 接 GND/驱动低端
```

若后续再次无声，优先检查正负极、GND 共地、PB11 线是否松动。

## 验证

已执行 clean build：

```text
gmake clean
gmake all
```

CCS 编译通过。按当前规则：不自动烧录，由用户手动烧录。
