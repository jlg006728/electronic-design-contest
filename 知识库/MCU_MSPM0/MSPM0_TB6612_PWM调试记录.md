# MSPM0 + TB6612 PWM 调试记录

## 现象

电机、TB6612、MSPM0G3507、电池已经连接，程序能编译、能烧录、能进入 `main()`，但电机不转。

后续验证发现：

- `PB0/PB1/PB2/PB3/PB4` 作为 GPIO 输出正常。
- `PB4 -> STBY` 拉高后为 3.3V。
- 将 `TB6612 PWMA/PWMB` 直接接 3.3V 后，电机能转。
- 将 `PA12/PA13` 临时配置为 GPIO 高电平后，接到 `PWMA/PWMB`，电机也能转。
- 将 `PA12/PA13` 配置为 TIMG0 PWM 并启动定时器后，电机可以按占空比分档调速。

## 根因

原测试代码的问题不在电机、TB6612、电池或方向控制脚，而在 PWM 输出链路。

关键遗漏：

1. TB6612 的 `PWMA/PWMB` 不是可选脚，必须有有效高电平或 PWM。
   - `AIN1/AIN2/BIN1/BIN2` 只决定方向。
   - `STBY` 只决定芯片是否工作。
   - `PWMA/PWMB` 决定对应通道是否真正输出以及输出占空比。
   - 如果 `PWMA/PWMB = 0`，电机等价于 0% 占空比，不会转。

2. PA12/PA13 作为 PWM 输出时，不能只当普通 GPIO 使用。
   - PA12 需要配置为 `IOMUX_PINCM34_PF_TIMG0_CCP0`。
   - PA13 需要配置为 `IOMUX_PINCM35_PF_TIMG0_CCP1`。
   - MSPM0G3507 上这两个 function macro 必须按芯片头文件确认，不能凭经验猜。

3. 配置 TIMG0 PWM 模式后，还必须启动计数器。
   - 仅执行 `DL_TimerG_initPWMMode()`、设置 period/CCR 不会自动输出 PWM。
   - 必须调用：

```c
DL_TimerG_startCounter(TIMG0);
```

否则 PA12/PA13 不会产生 PWM 波形。

## 正确验证顺序

先不用 PWM，逐级确认硬件链路：

1. `PB4/STBY` 输出 3.3V。
2. `PB0/PB1` 控制 A 通道方向，`PB2/PB3` 控制 B 通道方向。
3. `PWMA/PWMB` 直接拉 3.3V，确认 TB6612 和电机能转。
4. `PA12/PA13` 配成 GPIO 高电平，确认接线能把高电平送到 `PWMA/PWMB`。
5. 最后再把 PA12/PA13 切到 TIMG0 PWM，确认占空比调速。

## MSPM0G3507 本项目引脚对应

```text
PB0  -> IOMUX_PINCM12 -> AIN1
PB1  -> IOMUX_PINCM13 -> AIN2
PB2  -> IOMUX_PINCM15 -> BIN1
PB3  -> IOMUX_PINCM16 -> BIN2
PB4  -> IOMUX_PINCM17 -> STBY
PA12 -> IOMUX_PINCM34 -> TIMG0_CCP0 -> PWMA
PA13 -> IOMUX_PINCM35 -> TIMG0_CCP1 -> PWMB
```

## 经验规则

以后写 MSPM0 + TB6612 电机测试程序时，必须先验证三层：

1. GPIO 方向层：`AIN/BIN/STBY` 是否能输出 0V/3.3V。
2. 使能层：`PWMA/PWMB` 是否有高电平或 PWM。
3. 外设层：TIMG PWM 是否正确 mux 到引脚，并且 timer counter 是否已经 start。

不要把“程序能编译烧录”当作“引脚已经有波形”。电机不转时，优先用万用表测 `STBY` 和 `PWMA/PWMB`。
