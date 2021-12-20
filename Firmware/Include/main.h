/**
 * @file header.h
 * @author Tom Klijn (tomklijn@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifdef __cplusplus
extern "C"
{
#endif

#ifndef MAIN_H
#define MAIN_H
/* ========================== include files =========================== */
#include <stdint.h>

/* ============================ constants ============================= */
/** GPIO constants for TK111110V01 **/
#define UART1_TX_GPIO_CLK RCU_GPIOA
#define UART1_TX_GPIO_PORT GPIOA
#define UART1_TX_PIN GPIO_PIN_9

#define UART1_RX_GPIO_CLK RCU_GPIOA
#define UART1_RX_GPIO_PORT GPIOA
#define UART1_RX_PIN GPIO_PIN_10

#define UART1_CLK RCU_USART0

#define SPWM1_GPIO_CLK RCU_GPIOA
#define SPWM1_GPIO_PORT GPIOA
#define SPWM1_PIN GPIO_PIN_2

#define SPWM2_GPIO_CLK RCU_GPIOA
#define SPWM2_GPIO_PORT GPIOA
#define SPWM2_PIN GPIO_PIN_1

#define SPWM3_GPIO_CLK RCU_GPIOA
#define SPWM3_GPIO_PORT GPIOA
#define SPWM3_PIN GPIO_PIN_0

#define SPWM4_GPIO_CLK RCU_GPIOA
#define SPWM4_GPIO_PORT GPIOA
#define SPWM4_PIN GPIO_PIN_7

#define SPWM5_GPIO_CLK RCU_GPIOA
#define SPWM5_GPIO_PORT GPIOA
#define SPWM5_PIN GPIO_PIN_6

#define SPWM6_GPIO_CLK RCU_GPIOA
#define SPWM6_GPIO_PORT GPIOA
#define SPWM6_PIN GPIO_PIN_5

#define SPWM7_GPIO_CLK RCU_GPIOA
#define SPWM7_GPIO_PORT GPIOA
#define SPWM7_PIN GPIO_PIN_4

#define SPWM8_GPIO_CLK RCU_GPIOA
#define SPWM8_GPIO_PORT GPIOA
#define SPWM8_PIN GPIO_PIN_3

#define ADC_TRIG_OUT_GPIO_CLK RCU_GPIOA
#define ADC_TRIG_OUT_GPIO_PORT GPIOA
#define ADC_TRIG_OUT_PIN GPIO_PIN_8

#define BIAS_GPIO_CLK RCU_GPIOA
#define BIAS_GPIO_PORT GPIOA
#define BIAS_PIN GPIO_PIN_11

#define TIMEBASE_GPIO_CLK RCU_GPIOA
#define TIMEBASE_GPIO_PORT GPIOA
#define TIMEBASE_PIN GPIO_PIN_12

#define PERIOD_GPIO_CLK RCU_GPIOA
#define PERIOD_GPIO_PORT GPIOA
#define PERIOD_PIN GPIO_PIN_15

#define GND1_GPIO_CLK RCU_GPIOB
#define GND1_GPIO_PORT GPIOB
#define GND1_PIN GPIO_PIN_0

#define SIGNAL_GPIO_CLK RCU_GPIOB
#define SIGNAL_GPIO_PORT GPIOB
#define SIGNAL_PIN GPIO_PIN_1

#define GND2_GPIO_CLK RCU_GPIOB
#define GND2_GPIO_PORT GPIOB
#define GND2_PIN GPIO_PIN_2

#define ADC_TRIG_IN_GPIO_CLK RCU_GPIOB
#define ADC_TRIG_IN_GPIO_PORT GPIOB
#define ADC_TRIG_IN_PIN GPIO_PIN_11

#define LD2_GPIO_CLK RCU_GPIOB
#define LD2_GPIO_PORT GPIOB
#define LD2_PIN GPIO_PIN_12

#define LD1_GPIO_CLK RCU_GPIOB
#define LD1_GPIO_PORT GPIOB
#define LD1_PIN GPIO_PIN_13

/** Buffer constants **/
#define TX_BUFF_SIZE 512 //TODO remove from header file


/* ======================= public functions =========================== */
void gpio_toggle(uint32_t gpio_periph, uint32_t pin);

void led_on(uint8_t ledNbr);
void led_off(uint8_t ledNbr);
void led_toggle(uint8_t ledNbr);

uint32_t calcBuffLoc(uint32_t baseLoc, uint32_t offSet, uint32_t buffSize);


#endif /* MAIN_H */
#ifdef __cplusplus
}
#endif
