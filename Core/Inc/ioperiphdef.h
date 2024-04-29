/*
 * ioperiphdef.h
 *
 *  Created on: Apr 28, 2024
 *      Author: Christopher Tinker
 *
 *      IO pin and peripheral definitions for the STM32 SDR project.
 */

#ifndef INC_IOPERIPHDEF_H_
#define INC_IOPERIPHDEF_H_

#include "lwio.h"

/*
 * Peripheral Definitions
 */
#define SDR_ADC            ADC1
#define SDR_ADC_IRQn       ADC1_IRQn
#define CONSOLE_UART       USART2
#define CONSOLE_UART_IRQn  USART2_IRQn


/*
 * IO Defines
 */

/*
 * Debug/Console UART: USART2, TX: PA2, RX: PA3
 */
#define CONSOLE_UART_TX_PORT GPIOA
#define CONSOLE_UART_TX_PIN  LWIO_PIN_2
#define CONSOLE_UART_RX_PORT GPIOA
#define CONSOLE_UART_RX_PIN  LWIO_PIN_3

/*
 * Debug/Error LED's: PB0-3
 */
#define ERR_LED_PORT  GPIOB
#define ERR_LED_PIN_0 LWIO_PIN_0
#define ERR_LED_PIN_1 LWIO_PIN_1
#define ERR_LED_PIN_2 LWIO_PIN_2
#define ERR_LED_PIN_3 LWIO_PIN_3

/*
 * Debug PWM: PA1
 */
#define PWM_PORT GPIOA
#define PWM_PIN  LWIO_PIN_1

/*
 * Onboard LED: PA5
 */
#define LD2_PORT GPIOA
#define LD2_PIN  LWIO_PIN_5

/*
 * SDR RX: ADC1 AIN12_IN5 (ADC1_IN5) on PA0
 * See STM32L476xx Datasheet Table 16 (p. 73)
 */
#define ADC_RX_PORT GPIOA
#define ADC_RX_PIN  LWIO_PIN_0

/*
 * I2S Speaker Out:
 */

#endif /* INC_IOPERIPHDEF_H_ */
