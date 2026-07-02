#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "config.h"

#define MOTOR_AIN1_PIN           DL_GPIO_PIN_0   /* PB0 */
#define MOTOR_AIN2_PIN           DL_GPIO_PIN_1   /* PB1 */
#define MOTOR_BIN1_PIN           DL_GPIO_PIN_2   /* PB2 */
#define MOTOR_BIN2_PIN           DL_GPIO_PIN_3   /* PB3 */
#define MOTOR_STBY_PIN           DL_GPIO_PIN_4   /* PB4 */
#define MOTOR_DIR_PORT           GPIOB

#define MOTOR_PWM_PORT           GPIOA
#define MOTOR_PWMA_PIN           DL_GPIO_PIN_12  /* PA12 TIMG0_CCP0 */
#define MOTOR_PWMB_PIN           DL_GPIO_PIN_13  /* PA13 TIMG0_CCP1 */
#define MOTOR_PWM_TIMER          TIMG0

#define SERVO_PWM_PORT           GPIOA
#define SERVO_X_PIN              DL_GPIO_PIN_3   /* PA3 TIMG8_CCP0 */
#define SERVO_Y_PIN              DL_GPIO_PIN_4   /* PA4 TIMG8_CCP1 */
#define SERVO_PWM_TIMER          TIMG8

#define UART_OPENMV_INST         UART0

#define LINE_S1_PIN              DL_GPIO_PIN_2   /* PA2 */
#define LINE_S2_PIN              DL_GPIO_PIN_5   /* PA5 */
#define LINE_S3_PIN              DL_GPIO_PIN_6   /* PA6 */
#define LINE_S4_PIN              DL_GPIO_PIN_7   /* PA7 */
#define LINE_S5_PIN              DL_GPIO_PIN_8   /* PA8 */
#define LINE_A_MASK              (LINE_S1_PIN | LINE_S2_PIN | LINE_S3_PIN | LINE_S4_PIN | LINE_S5_PIN)
#define LINE_S6_PIN              DL_GPIO_PIN_9   /* PB9 */
#define LINE_S7_PIN              DL_GPIO_PIN_10  /* PB10 */
#define LINE_B_MASK              (LINE_S6_PIN | LINE_S7_PIN)

#define ENCODER_LEFT_A_PIN       DL_GPIO_PIN_12  /* PB12 */
#define ENCODER_LEFT_B_PIN       DL_GPIO_PIN_13  /* PB13 */
#define ENCODER_RIGHT_A_PIN      DL_GPIO_PIN_14  /* PB14 */
#define ENCODER_RIGHT_B_PIN      DL_GPIO_PIN_15  /* PB15 */
#define ENCODER_PORT             GPIOB

#define BUTTON_MODE_PIN          DL_GPIO_PIN_8   /* PB8 */
#define BUTTON_START_PIN         DL_GPIO_PIN_11  /* PB11 */
#define BUTTON_PORT              GPIOB

#define LED_PIN                  DL_GPIO_PIN_15  /* PA15 */
#define LED_PORT                 GPIOA

#define LASER_PIN                DL_GPIO_PIN_20  /* PA20, only for TTL/EN laser input */
#define LASER_PORT               GPIOA

void board_init(void);

void board_set_motor_pwm(uint16_t left_ticks, uint16_t right_ticks);
void board_set_motor_standby(bool enable);

void board_set_servo_us(uint8_t channel, uint16_t pulse_us);

uint8_t board_read_line_mask(void);
bool board_button_mode_pressed(void);
bool board_button_start_pressed(void);

bool board_encoder_left_a_high(void);
bool board_encoder_left_b_high(void);
bool board_encoder_right_a_high(void);
bool board_encoder_right_b_high(void);

void board_led_set(bool on);
void board_led_toggle(void);
void board_laser_set(bool on);

void board_uart_send_byte(uint8_t byte);
void board_uart_on_rx_byte(uint8_t byte);
void board_gpio_interrupt_handler(void);

#endif
