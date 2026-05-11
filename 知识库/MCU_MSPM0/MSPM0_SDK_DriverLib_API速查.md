# MSPM0 SDK DriverLib API 速查

> 来源: TI MSPM0-SDK DriverLib API Guide
> 在线: software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/driverlib/

## GPIO (`dl_gpio.h`)

```c
// 读写
uint32_t DL_GPIO_readPins(GPIO_Regs *gpio, uint32_t pins);
void DL_GPIO_writePins(GPIO_Regs *gpio, uint32_t pins);
void DL_GPIO_setPins(GPIO_Regs *gpio, uint32_t pins);
void DL_GPIO_clearPins(GPIO_Regs *gpio, uint32_t pins);
void DL_GPIO_togglePins(GPIO_Regs *gpio, uint32_t pins);

// 引脚宏
DL_GPIO_PIN_0  ~ DL_GPIO_PIN_31

// 中断边沿
DL_GPIO_PIN_N_EDGE_RISE / FALL / RISE_FALL / DISABLE
DL_GPIO_PIN_N_INPUT_FILTER_DISABLE / 1_CYCLE / 3_CYCLES / 8_CYCLES
```

## UART (`dl_uart.h`, `dl_uart_main.h`)

```c
// 初始化
void DL_UART_Main_init(UART_Regs *uart, DL_UART_Main_Config *config);
void DL_UART_Main_enable(UART_Regs *uart);

// 发送
void DL_UART_transmitData(UART_Regs *uart, uint8_t data);            // 非阻塞
void DL_UART_transmitDataBlocking(UART_Regs *uart, uint8_t data);    // 阻塞
bool DL_UART_transmitDataCheck(UART_Regs *uart, uint8_t data);       // 检查后发

// 接收
uint8_t DL_UART_receiveData(UART_Regs *uart);                        // 非阻塞
uint8_t DL_UART_receiveDataBlocking(UART_Regs *uart);                // 阻塞
bool DL_UART_receiveDataCheck(UART_Regs *uart, uint8_t *buffer);     // 检查后收

// 状态
bool DL_UART_isTxFifoEmpty(UART_Regs *uart);
bool DL_UART_isRxFifoEmpty(UART_Regs *uart);

// Config 结构体
typedef struct {
    DL_UART_MAIN_MODE mode;          // IDLE_LINE
    DL_UART_MAIN_DIRECTION direction; // TX_RX
    DL_UART_MAIN_FLOW_CONTROL flowControl; // NONE
    DL_UART_MAIN_PARITY parity;      // NONE
    DL_UART_MAIN_WORD_LENGTH wordLength; // 8_BITS
    DL_UART_MAIN_STOP_BITS stopBits; // ONE
} DL_UART_Main_Config;
```

## ADC (`dl_adc12.h`)

```c
// 核心控制
void DL_ADC12_init(ADC12_Regs *adc12, DL_ADC12_Config *config);
void DL_ADC12_startConversion(ADC12_Regs *adc12);
void DL_ADC12_stopConversion(ADC12_Regs *adc12);
void DL_ADC12_enableConversions(ADC12_Regs *adc12);
void DL_ADC12_disableConversions(ADC12_Regs *adc12);

// 数据读取
uint16_t DL_ADC12_getMemResult(ADC12_Regs *adc12, DL_ADC12_MEM_IDX idx);

// 中断
DL_ADC12_IIDX DL_ADC12_getPendingInterrupt(ADC12_Regs *adc12);
void DL_ADC12_enableInterrupt(ADC12_Regs *adc12, DL_ADC12_IIDX idx);

// 中断索引
DL_ADC12_IIDX_MEM0_RESULT_LOADED ~ DL_ADC12_IIDX_MEM11_RESULT_LOADED

// 电压换算
float voltage = (float)adc_result / 4096.0 * 3.3;
```

## Timer/PWM (`dl_timer.h`, `dl_timerg.h`)

```c
// PWM 初始化
void DL_Timer_initPWMMode(TIMER_Regs *timer, DL_Timer_PWMConfig *config);

// PWM Config 结构体
typedef struct {
    uint32_t period;                 // PWM 周期 (计数值)
    DL_TIMER_PWM_MODE pwmMode;       // EDGE_ALIGN / CENTER_ALIGN
    bool isTimerWithFourCC;          // 是否有4个CC寄存器
    DL_TIMER startTimer;             // 初始化后启动
} DL_Timer_PWMConfig;

// 占空比
void DL_Timer_setCaptureCompareValue(TIMER_Regs *timer,
                                      uint32_t value, DL_TIMER_CC_INDEX idx);
// CC索引
DL_TIMER_CC_0_INDEX ~ DL_TIMER_CC_3_INDEX

// 启停
void DL_TimerG_startCounter(TIMER_Regs *timer);
void DL_TimerG_stopCounter(TIMER_Regs *timer);

// 中断
void DL_TimerG_enableInterrupt(TIMER_Regs *timer, uint32_t mask);
DL_TIMER_IIDX DL_TimerG_getPendingInterrupt(TIMER_Regs *timer);

// 中断掩码
DL_TIMERG_INTERRUPT_CC0_DN/UP_EVENT
DL_TIMERG_INTERRUPT_ZERO_EVENT / LOAD_EVENT
```

## SysConfig 初始化

```c
// 由 SysConfig 工具自动生成
SYSCFG_DL_init();  // 调用后所有外设已配置完毕

// 然后直接使用 DriverLib API
DL_GPIO_setPins(LED_PORT, LED_PIN);
DL_UART_transmitDataBlocking(UART_INST, data);
DL_ADC12_startConversion(ADC12_INST);
DL_TimerG_startCounter(TIMER_INST);
```
