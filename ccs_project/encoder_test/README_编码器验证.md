# 编码器验证程序

## 接线

| TB6612 驱动板 | MSPM0G3507 |
|---|---|
| E1A | PB12 |
| E1B | PB13 |
| E2A | PB14 |
| E2B | PB15 |
| GND | GND |

注意：

- 6P 电机线继续插在 TB6612 板上，不要拆。
- AO1/AO2/BO1/BO2 是电机动力线，不能接 MSPM0。
- 这一步先手转轮子，不让电机自动转。

## CCS 操作

1. 打开当前可以烧录的 empty 工程。
2. 用 `main.c` 的内容替换工程里的 `empty.c`。
3. Build。
4. Debug。
5. 程序停在 `main()` 后，点击绿色 Run。
6. 打开 `View -> Expressions`。
7. 添加这些变量：

```text
g_left_count
g_right_count
g_left_edges
g_right_edges
g_left_a_level
g_left_b_level
g_right_a_level
g_right_b_level
```

## 判断结果

手动慢慢转左轮：

- `g_left_count` 应该变化。
- `g_left_edges` 应该增加。
- `g_left_a_level/g_left_b_level` 应该在 0 和 1 之间变化。

手动慢慢转右轮：

- `g_right_count` 应该变化。
- `g_right_edges` 应该增加。
- `g_right_a_level/g_right_b_level` 应该在 0 和 1 之间变化。

如果 count 是负数，不是故障，只代表 A/B 相方向和代码定义相反，后面可以在软件里取反。

## 故障判断

如果某个轮子完全不变：

1. 确认该轮的 E?A/E?B 是否接到正确 PB 引脚。
2. 确认 TB6612 GND 和 MSPM0 GND 是否共地。
3. 万用表测编码器 A/B 对 GND，慢转轮子时应在 0V 和 3.3V 之间变化。
4. 确认 TB6612 板上 3V3 对 GND 约为 3.3V。

如果 A/B 电平变化，但 count 不变：

- 检查 A 相是否接到了 PB12/PB14。当前测试程序用轮询方式读取 A 相变化，不依赖 GPIO 中断。

如果 count 只变化很少：

- 先慢转，不要快速转。调试器刷新变量很慢，但 `g_left_edges/g_right_edges` 应持续增加。
