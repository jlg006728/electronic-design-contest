/**
 * 电机测试程序 — MSPM0G3507 LaunchPad
 *
 * 测试 TB6612FNG 驱动的两个直流电机
 * 接线见 硬件接口图.md
 *
 * 测试流程:
 *   1. LED 闪烁 3 次 (程序启动)
 *   2. 左电机正转 50% → 停 → 反转 50% → 停
 *   3. 右电机正转 50% → 停 → 反转 50% → 停
 *   4. 双电机同时正转 30% → 50% → 70% → 停
 *   5. LED 快闪 (测试完成)
 */

#include "ti_msp_dl_config.h"

/* ============================================================
 * 引脚定义 (根据 硬件接口图.md)
 * ============================================================ */
/* 电机方向 — GPIOB */
#define MOTOR_AIN1_PIN   DL_GPIO_PIN_0   /* PB0 */
#define MOTOR_AIN2_PIN   DL_GPIO_PIN_1   /* PB1 */
#define MOTOR_BIN1_PIN   DL_GPIO_PIN_2   /* PB2 */
#define MOTOR_BIN2_PIN   DL_GPIO_PIN_3   /* PB3 */
#define MOTOR_STBY_PIN   DL_GPIO_PIN_4   /* PB4 */
#define MOTOR_DIR_PORT   GPIOB

/* 电机 PWM — TIMG0, PA12(CCP0) / PA13(CCP1) */
#define PWM_INST         TIMG0
#define PWM_PERIOD       1600   /* 32MHz / 20kHz = 1600 */

/* LED — PA15 */
#define LED_PORT         GPIOA
#define LED_PIN          DL_GPIO_PIN_15

/* ============================================================
 * 延时函数
 * ============================================================ */
static void delay_ms(uint32_t ms)
{
    /* 粗略延时, SYSOSC=32MHz, 每ms约32000次循环 */
    for (uint32_t i = 0; i < ms; i++) {
        delay_cycles(32000);
    }
}

/* ============================================================
 * LED 控制
 * ============================================================ */
static void led_on(void)
{
    DL_GPIO_setPins(LED_PORT, LED_PIN);
}

static void led_off(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_PIN);
}

static void led_blink(uint32_t times, uint32_t period_ms)
{
    for (uint32_t i = 0; i < times; i++) {
        led_on();
        delay_ms(period_ms / 2);
        led_off();
        delay_ms(period_ms / 2);
    }
}

/* ============================================================
 * 电机控制
 * ============================================================ */
static void motor_enable(void)
{
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_STBY_PIN);
}

static void motor_disable(void)
{
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_STBY_PIN);
}

/**
 * 设置电机占空比
 * @param duty: -100 ~ +100, 正=正转, 负=反转
 */
static void motor_left(float duty)
{
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN);
    if (duty >= 0.0f) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_AIN1_PIN);
    } else {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_AIN2_PIN);
    }
    uint32_t ccr = (uint32_t)(fabsf(duty) / 100.0f * PWM_PERIOD);
    DL_TimerG_setCaptureCompareValue(PWM_INST, ccr, DL_TIMER_CC_0_INDEX);
}

static void motor_right(float duty)
{
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    if (duty >= 0.0f) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_BIN1_PIN);
    } else {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_BIN2_PIN);
    }
    uint32_t ccr = (uint32_t)(fabsf(duty) / 100.0f * PWM_PERIOD);
    DL_TimerG_setCaptureCompareValue(PWM_INST, ccr, DL_TIMER_CC_1_INDEX);
}

static void motor_stop_all(void)
{
    motor_left(0.0f);
    motor_right(0.0f);
    DL_GPIO_clearPins(MOTOR_DIR_PORT,
        MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |
        MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
}

/* ============================================================
 * PWM 初始化 (TIMG0, PA12/PA13, 20kHz)
 * ============================================================ */
static void pwm_init(void)
{
    /* 1. 使能 TIMG0 时钟 */
    DL_TimerG_reset(PWM_INST);
    DL_TimerG_enablePower(PWM_INST);
    delay_cycles(16);

    /* 2. 配置 PA12/PA13 为 TIMG0 CCP 输出 */
    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM34, IOMUX_PINCM34_PF_TIMG0_CCP0);  /* PA12 */
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_12);

    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM35, IOMUX_PINCM35_PF_TIMG0_CCP1);  /* PA13 */
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_13);

    /* 3. 时钟配置: BUSCLK, 不分频 */
    DL_TimerG_ClockConfig clockCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 0U,
    };
    DL_TimerG_setClockConfig(PWM_INST, &clockCfg);

    /* 4. PWM 模式: 边沿对齐 */
    DL_TimerG_PWMConfig pwmCfg = {
        .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period            = PWM_PERIOD,
        .isTimerWithFourCC = false,
        .startTimer        = DL_TIMER_STOP,
    };
    DL_TimerG_initPWMMode(PWM_INST, &pwmCfg);

    /* 5. CC 输出控制 */
    DL_TimerG_setCaptureCompareOutCtl(PWM_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(PWM_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    /* 6. 立即更新 */
    DL_TimerG_setCaptCompUpdateMethod(PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    /* 7. 初始占空比 0 */
    DL_TimerG_setCaptureCompareValue(PWM_INST, 0, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_INST, 0, DL_TIMER_CC_1_INDEX);

    /* 8. 使能时钟, 设置输出方向 */
    DL_TimerG_enableClock(PWM_INST);
    DL_TimerG_setCCPDirection(PWM_INST,
        DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
}

/* ============================================================
 * 主函数
 * ============================================================ */
int main(void)
{
    /* SysConfig 初始化 (GPIO 时钟等) */
    SYSCFG_DL_init();

    /* 手动初始化 PWM */
    pwm_init();

    /* 使能 TB6612 */
    motor_enable();

    /* === 启动提示: LED 闪 3 次 === */
    led_blink(3, 300);
    delay_ms(500);

    /* === 测试1: 左电机正转 50% === */
    motor_left(50.0f);
    delay_ms(2000);
    motor_stop_all();
    delay_ms(500);

    /* === 测试2: 左电机反转 50% === */
    motor_left(-50.0f);
    delay_ms(2000);
    motor_stop_all();
    delay_ms(500);

    /* === 测试3: 右电机正转 50% === */
    motor_right(50.0f);
    delay_ms(2000);
    motor_stop_all();
    delay_ms(500);

    /* === 测试4: 右电机反转 50% === */
    motor_right(-50.0f);
    delay_ms(2000);
    motor_stop_all();
    delay_ms(500);

    /* === 测试5: 双电机同时正转, 逐步加速 === */
    motor_left(30.0f);
    motor_right(30.0f);
    delay_ms(1500);

    motor_left(50.0f);
    motor_right(50.0f);
    delay_ms(1500);

    motor_left(70.0f);
    motor_right(70.0f);
    delay_ms(1500);

    motor_stop_all();
    motor_disable();

    /* === 测试完成: LED 快闪 === */
    while (1) {
        led_blink(1, 100);
        delay_ms(200);
    }
}
