# MSPM0 SDK 示例代码索引

> 来源: MSPM0 SDK 2.10.00.04 — `examples/nortos/LP_MSPM0G3507/`
> 共 174 个示例，覆盖所有外设和常用子系统

---

## 目录结构

```
sdk-examples/
├── driverlib/        # 141 个外设级示例 (单外设基础用法)
├── msp_subsystems/   #  32 个综合示例 (多外设协同)
└── demos/            #   1 个出厂演示程序
```

---

## E题相关度标记

| 标记 | 含义 |
|------|------|
| ★★★ | 直接相关，必看 |
| ★★☆ | 可能使用，建议了解 |
| ★☆☆ | 备查参考 |
| — | 本题暂不需要 |

---

## 一、GPIO (8个) — ★★★

| 示例 | 说明 | E题用途 |
|------|------|---------|
| `gpio_toggle_output` | LED 翻转 | 状态指示灯 |
| `gpio_toggle_output_hiz` | 高阻态切换 | 低功耗 |
| `gpio_input_capture` | 输入捕获中断 | 编码器读取 |
| `gpio_software_poll` | 软件轮询输入 | 按键读取 |
| `gpio_simultaneous_interrupts` | 多引脚同步中断 | 多传感器触发 |

---

## 二、PWM / Timer (20+个) — ★★★

### 电机 PWM 用
| 示例 | 说明 |
|------|------|
| `timx_timer_mode_pwm_edge_sleep` | 边沿对齐 PWM（最常用） |
| `timx_timer_mode_pwm_edge_sleep_shadow` | 带影子寄存器的 PWM（占空比更新无毛刺） |
| `timx_timer_mode_pwm_center_stop` | 中心对齐 PWM |
| `tima_timer_mode_pwm_dead_band` | 带死区 PWM（H桥互补输出） |

### 编码器 / 舵机 用
| 示例 | 说明 |
|------|------|
| `timg_qei_mode` | **正交编码器接口（电机测速）** |
| `timx_timer_mode_capture_duty_and_period` | 输入捕获（测PWM脉宽） |
| `timx_timer_mode_one_shot_standby` | 单次脉冲（舵机定位） |

### 定时器
| 示例 | 说明 |
|------|------|
| `systick_periodic_timer` | SysTick 周期定时 |
| `timx_timer_mode_periodic_sleep` | 周期定时器 + 休眠 |
| `timx_timer_mode_periodic_standby` | 周期定时器 + 待机 |
| `timx_timer_mode_periodic_stop` | 周期定时器 + 停止 |

---

## 三、UART (15+个) — ★★★（与 OpenMV 通信）

| 示例 | 说明 | E题用途 |
|------|------|---------|
| `uart_echo_interrupts_standby` | 中断回环 | 基础收发框架 |
| `uart_rw_multibyte_fifo_poll` | FIFO 多字节轮询收发 | **OpenMV 串口通信模板** |
| `uart_rx_multibyte_fifo_dma_interrupts` | DMA 接收 + 中断 | 高效接收目标坐标 |
| `uart_tx_multibyte_fifo_dma_interrupts` | DMA 发送 + 中断 | 高效发送数据 |
| `uart_external_loopback_interrupt` | 外部回环测试 | 硬件调试 |
| `uart_internal_loopback_standby_restore` | 内部回环 | 软件调试 |
| `uart_tx_hw_flow_control` | 硬件流控发送 | 可选 |
| `uart_rx_hw_flow_control` | 硬件流控接收 | 可选 |
| `uart_rs485_send_packet` | RS-485 发送 | 如用 RS-485 |
| `uart_extend_manchester_echo` | Manchester 编码 | 特殊编码需求 |

---

## 四、I2C (8个) — ★★☆（MPU6050）

| 示例 | 说明 |
|------|------|
| `i2c_controller_rw_multibyte_fifo_interrupts` | **Controller 多字节读写 + FIFO 中断** |
| `i2c_controller_rw_multibyte_fifo_poll` | Controller 多字节轮询读写 |
| `i2c_controller_rw_repeated_start_fifo_interrupts` | Controller 重复起始 |
| `i2c_target_rw_multibyte_fifo_interrupts` | Target 多字节读写 |
| `i2c_target_rw_multibyte_fifo_poll` | Target 轮询读写 |
| `i2c_controller_target_dynamic_switching` | Controller/Target 动态切换 |
| `i2c_multicontroller_arbitration` | 多主机仲裁 |

---

## 五、ADC (15+个) — ★★★（TCRT5000 巡迹）

| 示例 | 说明 | E题用途 |
|------|------|---------|
| `adc12_single_conversion` | 单次转换 | **巡迹传感器单路读取** |
| `adc12_sequence_conversion` | 序列转换 | **7路巡迹传感器循环读取** |
| `adc12_max_freq_dma` | DMA 最高频率采集 | 高速连续采样 |
| `adc12_triggered_by_timer_event` | 定时器触发 ADC | 固定周期采样 |
| `adc12_window_comparator` | 窗口比较器 | 阈值检测（黑/白判定） |
| `adc12_single_conversion_vref_internal` | 内部参考电压 | 提高精度 |
| `adc12_internal_temp_sensor_mathacl` | 内部温度传感器 | 温度补偿 |
| `adc12_simultaneous_trigger_event` | 同步触发 | 多路同步 |

---

## 六、DMA (3个) — ★★☆

| 示例 | 说明 |
|------|------|
| `dma_block_transfer` | 块传输 |
| `dma_fill_data` | 数据填充 |
| `dma_table_transfer` | 表传输（链式DMA） |

---

## 七、SPI (8个) — ★☆☆

| 示例 | 说明 |
|------|------|
| `spi_controller_echo_interrupts` | Controller 回环 + 中断 |
| `spi_controller_fifo_dma_interrupts` | Controller FIFO DMA |
| `spi_peripheral_echo_interrupts` | Peripheral 回环 |
| `spi_peripheral_fifo_dma_interrupts` | Peripheral DMA |

---

## 八、其他外设备查

### 系统控制 ★★☆
| 示例 | 说明 |
|------|------|
| `sysctl_frequency_clock_counter` | 时钟频率测量 |
| `sysctl_mclk_syspll` | 主时钟 + PLL 配置 |
| `sysctl_power_policy_sleep_to_standby` | 休眠 → 待机 |
| `sysctl_power_policy_sleep_to_stop` | 休眠 → 停止 |
| `sysctl_shutdown` | 关机模式 |

### NVIC 中断 ★★☆
| 示例 | 说明 |
|------|------|
| `nvic_interrupt_disable` | 中断禁用 |
| `nvic_interrupt_grouping` | 中断分组 |

### 比较器 — (可用于 ADC 阈值替代方案)
| `comp_analog_filter` | 模拟滤波比较器 |
| `comp_dac_to_timer_event` | 比较器触发定时器 |

### RTC ★☆☆ (时间戳、闹钟)
| `rtc_calendar_alarm_standby` | 日历闹钟 |
| `rtc_periodic_alarm_lfosc_standby` | 周期闹钟 |

### 其他
| 示例 | 说明 |
|------|------|
| `crc_calculate_checksum` | CRC 校验 |
| `mathacl_mpy_div_op` | 硬件乘除加速 |
| `mathacl_trig_op` | 硬件三角函数加速 |
| `trng_sample` | 真随机数 |

---

## 九、子系统示例 (msp_subsystems) — ★★★

| 示例 | 说明 | E题用途 |
|------|------|---------|
| `pushbutton_change_pwm` | 按键改变 PWM 占空比 | **PWM 控制参考模板** |
| `adc_to_pwm` | ADC → PWM 映射 | **传感器→电机控制原型** |
| `adc_to_uart` | ADC → UART 转发 | **OpenMV 巡迹数据上报** |
| `adc_simultaneous_sample` | 多路 ADC 同步采样 | 7路巡迹同步 |
| `adc_to_spi_peripheral` | ADC → SPI 转发 | 备选通信 |
| `adc_dma_ping_pong` | ADC + DMA 乒乓缓冲 | 高效巡迹数据采集 |
| `task_scheduler` | 任务调度器 | **多任务框架参考** |
| `uart_to_i2c_bridge` | UART ↔ I2C 桥 | OpenMV ↔ MPU6050 |
| `uart_to_spi_bridge` | UART ↔ SPI 桥 | 扩展通信 |
| `pwm_led_driver` | PWM LED 驱动 | 状态指示 |
| `fir_low_pass_filter` | FIR 低通滤波 | 传感器滤波 |
| `iir_low_pass_filter` | IIR 低通滤波 | 传感器滤波 |
| `signal_acquisition` | 信号采集框架 | 多传感器统一采集 |

---

## 快速参考：E题代码映射

| E题功能 | 参考示例 | 路径 |
|---------|----------|------|
| LED 闪烁 (调试标志) | `gpio_toggle_output` | driverlib/ |
| 按键读取 (圈数设置) | `gpio_software_poll` | driverlib/ |
| 电机 PWM 驱动 | `timx_timer_mode_pwm_edge_sleep` | driverlib/ |
| 编码器测速 | `timg_qei_mode` | driverlib/ |
| 舵机 PWM | `timx_timer_mode_pwm_edge_sleep` | driverlib/ |
| 巡迹传感器 ADC 读取 | `adc12_sequence_conversion` | driverlib/ |
| OpenMV 串口通信 | `uart_rw_multibyte_fifo_poll` | driverlib/ |
| MPU6050 I2C 读取 | `i2c_controller_rw_multibyte_fifo_interrupts` | driverlib/ |
| 传感器→PWM 控制闭环 | `adc_to_pwm` | msp_subsystems/ |
| 多任务调度 | `task_scheduler` | msp_subsystems/ |
