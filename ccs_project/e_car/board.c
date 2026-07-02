#include "board.h"
#include "encoder.h"
#include "openmv.h"
#include "state_machine.h"

static void init_output_pin(uint32_t pincm, GPIO_Regs *port, uint32_t pin, bool high)
{
    DL_GPIO_initDigitalOutput(pincm);
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
    DL_GPIO_enableOutput(port, pin);
}

static void init_input_pin(uint32_t pincm, bool pull_up)
{
    DL_GPIO_initDigitalInputFeatures(pincm,
        DL_GPIO_INVERSION_DISABLE,
        pull_up ? DL_GPIO_RESISTOR_PULL_UP : DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static void board_gpio_init(void)
{
    init_output_pin(IOMUX_PINCM12, GPIOB, MOTOR_AIN1_PIN, false);
    init_output_pin(IOMUX_PINCM13, GPIOB, MOTOR_AIN2_PIN, false);
    init_output_pin(IOMUX_PINCM15, GPIOB, MOTOR_BIN1_PIN, false);
    init_output_pin(IOMUX_PINCM16, GPIOB, MOTOR_BIN2_PIN, false);
    init_output_pin(IOMUX_PINCM17, GPIOB, MOTOR_STBY_PIN, false);

    init_output_pin(IOMUX_PINCM37, GPIOA, LED_PIN, false);
    init_output_pin(IOMUX_PINCM42, GPIOA, LASER_PIN, false);

    init_input_pin(IOMUX_PINCM7, false);   /* PA2 line S1 */
    init_input_pin(IOMUX_PINCM10, false);  /* PA5 line S2 */
    init_input_pin(IOMUX_PINCM11, false);  /* PA6 line S3 */
    init_input_pin(IOMUX_PINCM14, false);  /* PA7 line S4 */
    init_input_pin(IOMUX_PINCM19, false);  /* PA8 line S5 */
    init_input_pin(IOMUX_PINCM26, false);  /* PB9 line S6 */
    init_input_pin(IOMUX_PINCM27, false);  /* PB10 line S7 */

    init_input_pin(IOMUX_PINCM25, true);   /* PB8 MODE */
    init_input_pin(IOMUX_PINCM28, true);   /* PB11 START */
    init_input_pin(IOMUX_PINCM29, false);  /* PB12 left encoder A */
    init_input_pin(IOMUX_PINCM30, false);  /* PB13 left encoder B */
    init_input_pin(IOMUX_PINCM31, false);  /* PB14 right encoder A */
    init_input_pin(IOMUX_PINCM32, false);  /* PB15 right encoder B */

    DL_GPIO_setLowerPinsPolarity(GPIOB,
        DL_GPIO_PIN_8_EDGE_FALL |
        DL_GPIO_PIN_11_EDGE_FALL |
        DL_GPIO_PIN_12_EDGE_RISE_FALL |
        DL_GPIO_PIN_14_EDGE_RISE_FALL);

    DL_GPIO_clearInterruptStatus(GPIOB,
        BUTTON_MODE_PIN |
        BUTTON_START_PIN |
        ENCODER_LEFT_A_PIN |
        ENCODER_RIGHT_A_PIN);

    DL_GPIO_enableInterrupt(GPIOB,
        BUTTON_MODE_PIN |
        BUTTON_START_PIN |
        ENCODER_LEFT_A_PIN |
        ENCODER_RIGHT_A_PIN);

    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

static void board_motor_pwm_init(void)
{
    DL_TimerG_reset(MOTOR_PWM_TIMER);
    DL_TimerG_enablePower(MOTOR_PWM_TIMER);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM34, IOMUX_PINCM34_PF_TIMG0_CCP0);
    DL_GPIO_enableOutput(GPIOA, MOTOR_PWMA_PIN);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM35, IOMUX_PINCM35_PF_TIMG0_CCP1);
    DL_GPIO_enableOutput(GPIOA, MOTOR_PWMB_PIN);

    DL_TimerG_ClockConfig clock_cfg = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U,
    };
    DL_TimerG_setClockConfig(MOTOR_PWM_TIMER, &clock_cfg);

    DL_TimerG_PWMConfig pwm_cfg = {
        .period = MOTOR_PWM_PERIOD,
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .isTimerWithFourCC = false,
        .startTimer = DL_TIMER_STOP,
    };
    DL_TimerG_initPWMMode(MOTOR_PWM_TIMER, &pwm_cfg);

    DL_TimerG_setCaptureCompareOutCtl(MOTOR_PWM_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareOutCtl(MOTOR_PWM_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(MOTOR_PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(MOTOR_PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    board_set_motor_pwm(0U, 0U);
    DL_TimerG_enableClock(MOTOR_PWM_TIMER);
    DL_TimerG_setCCPDirection(MOTOR_PWM_TIMER, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_TimerG_startCounter(MOTOR_PWM_TIMER);
}

static void board_servo_pwm_init(void)
{
    DL_TimerG_reset(SERVO_PWM_TIMER);
    DL_TimerG_enablePower(SERVO_PWM_TIMER);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM8, IOMUX_PINCM8_PF_TIMG8_CCP0);
    DL_GPIO_enableOutput(GPIOA, SERVO_X_PIN);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM9, IOMUX_PINCM9_PF_TIMG8_CCP1);
    DL_GPIO_enableOutput(GPIOA, SERVO_Y_PIN);

    DL_TimerG_ClockConfig clock_cfg = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 31U,
    };
    DL_TimerG_setClockConfig(SERVO_PWM_TIMER, &clock_cfg);

    DL_TimerG_PWMConfig pwm_cfg = {
        .period = SERVO_PERIOD_US,
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .isTimerWithFourCC = false,
        .startTimer = DL_TIMER_STOP,
    };
    DL_TimerG_initPWMMode(SERVO_PWM_TIMER, &pwm_cfg);

    DL_TimerG_setCaptureCompareOutCtl(SERVO_PWM_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareOutCtl(SERVO_PWM_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SERVO_PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(SERVO_PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    board_set_servo_us(0U, SERVO_CENTER_US);
    board_set_servo_us(1U, SERVO_CENTER_US);
    DL_TimerG_enableClock(SERVO_PWM_TIMER);
    DL_TimerG_setCCPDirection(SERVO_PWM_TIMER, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_TimerG_startCounter(SERVO_PWM_TIMER);
}

static void board_uart_init(void)
{
    DL_UART_Main_reset(UART_OPENMV_INST);
    DL_UART_Main_enablePower(UART_OPENMV_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX);

    DL_UART_Main_ClockConfig clock_cfg = {
        .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1,
    };
    DL_UART_Main_setClockConfig(UART_OPENMV_INST, &clock_cfg);

    DL_UART_Main_Config uart_cfg = {
        .mode = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity = DL_UART_MAIN_PARITY_NONE,
        .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits = DL_UART_MAIN_STOP_BITS_ONE,
    };
    DL_UART_Main_init(UART_OPENMV_INST, &uart_cfg);
    DL_UART_Main_configBaudRate(UART_OPENMV_INST, SYSCLK_HZ, OPENMV_UART_BAUDRATE);
    DL_UART_Main_setRXFIFOThreshold(UART_OPENMV_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(UART_OPENMV_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(UART_OPENMV_INST);

    NVIC_EnableIRQ(UART0_INT_IRQn);
}

void board_init(void)
{
    board_gpio_init();
    board_motor_pwm_init();
    board_servo_pwm_init();
    board_uart_init();
}

void board_set_motor_pwm(uint16_t left_ticks, uint16_t right_ticks)
{
    if (left_ticks > MOTOR_PWM_PERIOD) {
        left_ticks = MOTOR_PWM_PERIOD;
    }
    if (right_ticks > MOTOR_PWM_PERIOD) {
        right_ticks = MOTOR_PWM_PERIOD;
    }
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_TIMER, left_ticks, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_TIMER, right_ticks, DL_TIMER_CC_1_INDEX);
}

void board_set_motor_standby(bool enable)
{
    if (enable) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_STBY_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_STBY_PIN);
    }
}

void board_set_servo_us(uint8_t channel, uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_US) {
        pulse_us = SERVO_MIN_US;
    } else if (pulse_us > SERVO_MAX_US) {
        pulse_us = SERVO_MAX_US;
    }

    if (channel == 0U) {
        DL_TimerG_setCaptureCompareValue(SERVO_PWM_TIMER, pulse_us, DL_TIMER_CC_0_INDEX);
    } else {
        DL_TimerG_setCaptureCompareValue(SERVO_PWM_TIMER, pulse_us, DL_TIMER_CC_1_INDEX);
    }
}

uint8_t board_read_line_mask(void)
{
    uint8_t mask = 0U;
    uint32_t a = DL_GPIO_readPins(GPIOA, LINE_A_MASK);
    uint32_t b = DL_GPIO_readPins(GPIOB, LINE_B_MASK);

    if ((a & LINE_S1_PIN) != 0U) mask |= (uint8_t)(1U << 0);
    if ((a & LINE_S2_PIN) != 0U) mask |= (uint8_t)(1U << 1);
    if ((a & LINE_S3_PIN) != 0U) mask |= (uint8_t)(1U << 2);
    if ((a & LINE_S4_PIN) != 0U) mask |= (uint8_t)(1U << 3);
    if ((a & LINE_S5_PIN) != 0U) mask |= (uint8_t)(1U << 4);
    if ((b & LINE_S6_PIN) != 0U) mask |= (uint8_t)(1U << 5);
    if ((b & LINE_S7_PIN) != 0U) mask |= (uint8_t)(1U << 6);

#if LINE_BLACK_IS_HIGH == 0
    mask = (uint8_t)(~mask) & 0x7FU;
#endif
    return mask;
}

bool board_button_mode_pressed(void)
{
    return (DL_GPIO_readPins(BUTTON_PORT, BUTTON_MODE_PIN) == 0U);
}

bool board_button_start_pressed(void)
{
    return (DL_GPIO_readPins(BUTTON_PORT, BUTTON_START_PIN) == 0U);
}

bool board_encoder_left_a_high(void)
{
    return (DL_GPIO_readPins(ENCODER_PORT, ENCODER_LEFT_A_PIN) != 0U);
}

bool board_encoder_left_b_high(void)
{
    return (DL_GPIO_readPins(ENCODER_PORT, ENCODER_LEFT_B_PIN) != 0U);
}

bool board_encoder_right_a_high(void)
{
    return (DL_GPIO_readPins(ENCODER_PORT, ENCODER_RIGHT_A_PIN) != 0U);
}

bool board_encoder_right_b_high(void)
{
    return (DL_GPIO_readPins(ENCODER_PORT, ENCODER_RIGHT_B_PIN) != 0U);
}

void board_led_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
    }
}

void board_led_toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_PIN);
}

void board_laser_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(LASER_PORT, LASER_PIN);
    } else {
        DL_GPIO_clearPins(LASER_PORT, LASER_PIN);
    }
}

void board_uart_send_byte(uint8_t byte)
{
    DL_UART_Main_transmitDataBlocking(UART_OPENMV_INST, byte);
}

void board_gpio_interrupt_handler(void)
{
    uint32_t pins = BUTTON_MODE_PIN |
        BUTTON_START_PIN |
        ENCODER_LEFT_A_PIN |
        ENCODER_RIGHT_A_PIN;
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOB, pins);

    if ((status & ENCODER_LEFT_A_PIN) != 0U) {
        encoder_left_isr();
    }
    if ((status & ENCODER_RIGHT_A_PIN) != 0U) {
        encoder_right_isr();
    }
    if ((status & BUTTON_MODE_PIN) != 0U) {
        sm_button_mode();
    }
    if ((status & BUTTON_START_PIN) != 0U) {
        sm_button_start();
    }

    DL_GPIO_clearInterruptStatus(GPIOB, status);
}

void UART0_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_OPENMV_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_OPENMV_INST)) {
            board_uart_on_rx_byte(DL_UART_Main_receiveDataBlocking(UART_OPENMV_INST));
        }
        break;
    default:
        break;
    }
}

void GROUP1_IRQHandler(void)
{
    board_gpio_interrupt_handler();
}

void board_uart_on_rx_byte(uint8_t byte)
{
    openmv_uart_isr(byte);
}
