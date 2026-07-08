# PB11 蜂鸣器隔离测试

时间：2026-07-07 18:41

## 文件

- 当前 CCS 活动源码：`C:\Users\1\Desktop\ELECTRIC\ele-car\empty.c`
- 时间戳源码：`C:\Users\1\Desktop\电赛\ccs_project\buzzer_test\20260707_184130_pb11_buzzer_isolation_test.c`
- 改前备份：`C:\Users\1\Desktop\电赛\ccs_project\integration_test\20260707_184130_before_buzzer_test_current_v18_backup.c`

## 行为

程序循环执行：

1. `PB11` 持续高电平约 1 秒，LED 常亮。
   - 用来测试 5V 有源蜂鸣器或三极管/MOSFET 驱动。
2. `PB11` 输出约 2kHz 方波约 1 秒，LED 闪烁。
   - 用来测试无源蜂鸣器。
3. `PB11` 低电平约 1 秒，LED 熄灭。

电机、舵机、OpenMV 都不会工作。

## 接线建议

推荐接法，适合 5V 有源蜂鸣器：

```text
5V -> 蜂鸣器 +
蜂鸣器 - -> 三极管/MOSFET 输出端
三极管/MOSFET 另一端 -> GND
PB11 -> 1k 电阻 -> 三极管基极/栅极
MSPM0 GND 与 5V 电源 GND 共地
```

临时直连测试：

```text
PB11 -> 蜂鸣器 +
蜂鸣器 - -> GND
```

直连只适合短时间判断，不建议比赛长期这样用，因为 5V 蜂鸣器电流可能超过 GPIO 舒适范围。

## 判断方法

- LED 不亮/不闪：程序可能没运行或没烧进去。
- LED 亮闪，PB11-GND 万用表有变化，但蜂鸣器不响：接线、驱动、电源或蜂鸣器类型问题。
- 1 秒高电平阶段响：蜂鸣器是有源型，集成代码可用高电平控制。
- 2kHz 方波阶段响但高电平阶段不响：蜂鸣器更像无源型，正式代码需要输出音频方波，不能只拉高。
- 两个阶段都不响：先把蜂鸣器直接接 5V/GND 测，确认蜂鸣器本体和极性。

## 验证

CCS `gmake all` 编译通过。按当前规则：不自动烧录，由用户手动烧录。
