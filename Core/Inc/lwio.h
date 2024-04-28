/*
 * lwio.h
 *
 *  Created on: Jan 31, 2024
 *      Author: Christopher Tinker
 *
 *      Lightweigt GPIO (LWIO) helper functions and Macros for STM32.
 *
 *      USAGE:
 *      	Each GPIO pin is identified by three parameters: GPIO PORT, NUM, and PIN.
 *      	PORT: GPIO peripheral instance, e.g. GPIOA or GPIOC.
 *      	NUM: GPIO pin number (0-15), e.g. 5 for PA5, or 13 for PC13. NUM = log2(PIN)
 *      	PIN: GPIO pin bitmask, used for some register accesses. PIN = 2 ^ NUM
 *
 */

#ifndef INC_LWIO_H_
#define INC_LWIO_H_

/* Includes */
#include <stdint.h>
#include "stm32l4xx.h"
#include "stm32l476xx.h"

/* Structures */
typedef struct {
	GPIO_TypeDef *port;
	uint32_t     pin;    // GPIO Pinmask (1u << 0-15)
	uint8_t      mode;   // GPIO mode (MODER)
	uint8_t      otype;  // Output type - PP, OD (OTYPER)
	uint8_t      pupd;   // Pullup/pulldown select (PUPDR)
	uint8_t      ospeed; // Output speed (OSPEEDR)
	uint8_t      afsel;  // Alternate Function select
} lwio_cfg;

/*
 * GPIO Pin masks
 */
#define LWIO_PIN_0  (0x0001u)
#define LWIO_PIN_1  (0x0002u)
#define LWIO_PIN_2  (0x0004u)
#define LWIO_PIN_3  (0x0008u)
#define LWIO_PIN_4  (0x0010u)
#define LWIO_PIN_5  (0x0020u)
#define LWIO_PIN_6  (0x0040u)
#define LWIO_PIN_7  (0x0080u)
#define LWIO_PIN_8  (0x0100u)
#define LWIO_PIN_9  (0x0200u)
#define LWIO_PIN_10 (0x0400u)
#define LWIO_PIN_11 (0x0800u)
#define LWIO_PIN_12 (0x1000u)
#define LWIO_PIN_13 (0x2000u)
#define LWIO_PIN_14 (0x4000u)
#define LWIO_PIN_15 (0x8000u)

/*
 * GPIO Pin Numbers
 */
#define LWIO_NUM_0   (0u)
#define LWIO_NUM_1   (1u)
#define LWIO_NUM_2   (2u)
#define LWIO_NUM_3   (3u)
#define LWIO_NUM_4   (4u)
#define LWIO_NUM_5   (5u)
#define LWIO_NUM_6   (6u)
#define LWIO_NUM_7   (7u)
#define LWIO_NUM_8   (8u)
#define LWIO_NUM_9   (9u)
#define LWIO_NUM_10  (10u)
#define LWIO_NUM_11  (11u)
#define LWIO_NUM_12  (12u)
#define LWIO_NUM_13  (13u)
#define LWIO_NUM_14  (14u)
#define LWIO_NUM_15  (15u)

/*
 * GPIO Mode Constants
 */
#define LWIO_OUTPUT (0x01u) // Output Mode
#define LWIO_AF     (0x02u) // Alternate Function Mode
#define LWIO_INPUT  (0x00u) // Input Mode
#define LWIO_ANALOG (0x03u) // Analog mode

#define LWIO_PUSHPULL  (0x00u) // Push-pull output
#define LWIO_OPENDRAIN (0x01u) // Open drain output
#define LWIO_NOOUTPUT  (0x00u)

#define LWIO_NOPULL   (0x00u) // No internal pullup or pulldown resistor
#define LWIO_PULLUP   (0x01)  // I/O pullup resistor
#define LWIO_PULLDOWN (0x2)   // I/O pulldown resistor

#define LWIO_SPEED_NONE     (0x00u)
#define LWIO_SPEED_LOW      (0x00u)
#define LWIO_SPEED_MED      (0x01u)
#define LWIO_SPEED_HIGH     (0x02u)
#define LWIO_SPEED_VERYHIGH (0x03u)

#define LWIO_AF_0  (0x00u)
#define LWIO_AF_1  (0x01u)
#define LWIO_AF_2  (0x02u)
#define LWIO_AF_3  (0x03u)
#define LWIO_AF_4  (0x04u)
#define LWIO_AF_5  (0x05u)
#define LWIO_AF_6  (0x06u)
#define LWIO_AF_7  (0x07u)
#define LWIO_AF_8  (0x08u)
#define LWIO_AF_9  (0x09u)
#define LWIO_AF_10 (0x0Au)
#define LWIO_AF_11 (0x0Bu)
#define LWIO_AF_12 (0x0Cu)
#define LWIO_AF_13 (0x0Du)
#define LWIO_AF_14 (0x0Eu)
#define LWIO_AF_15 (0x0Fu)

/* Function Macros */

static inline uint32_t lwioPinToNum(uint32_t pin) {
	int num;
	for (num = -1; pin != 0; num++) pin >>= 1;
	return num;
}

#define lwioNumToPin(num) (1u << num)

#define lwioSetPin(port, pin)        ((port)->BSRR = pin)
#define lwioClearPin(port, pin)      ((port)->BRR = pin)
#define lwioWritePin(port, pin, val) ((port)->BSRR = val ? pin : (pin << 16))
#define lwioTogglePin(port, pin)     ((port)->BSRR = (port->ODR & pin) ? (pin << 16) : pin)

#define lwioSetNum(port, num)        ((port)->BSRR = (1u << num))
#define lwioClearNum(port, num)      ((port)->BRR = (1u << num))
#define lwioWriteNum(port, num, val) ((port)->BSRR = val ? (1u << num) : ((1u << num) << 16))
#define lwioToggleNum(port, num)     ((port)->BSRR = (port->ODR & (1u << num)) ? \
																		 ((1u << num) << 16) : (1u << num))

/* Function Prototypes */
// TODO: switch from num to pin? Use pin everywhere?
void lwio_init(GPIO_TypeDef *port, uint32_t pin, uint8_t mode, uint8_t otype, uint8_t pupd, uint8_t ospeed);
void lwio_cfg_init(lwio_cfg *cfg);
void lwio_set_mode(GPIO_TypeDef *port, uint32_t pin, uint8_t mode);
void lwio_set_afsel(GPIO_TypeDef *port, uint32_t pin, uint8_t afsel);

#endif /* INC_LWIO_H_ */
