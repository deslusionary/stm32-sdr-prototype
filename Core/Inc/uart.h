/*
 * uart.h
 *
 *  Created on: Mar 3, 2024
 *      Author: Christopher Tinker
 *
 *      UART-only driver for STM32 USART peripheral.
 */

#ifndef INC_UART_H_
#define INC_UART_H_

/*
 * Includes
 */
#include "stm32l476xx.h"
#include <stdint.h>
#include <stddef.h>

/*
 * UART Config struct
 */
// STM32 HAL setup where a handle struct to a peripheral contains a peripheral pointer,
// and config structs seems like a pretty useful way of doing things.
typedef struct UART_cfg {
	uint32_t     wordlen;
	uint32_t     stopbits;
	uint32_t     parity;
	uint32_t     mode; //
} UART_cfg;


typedef struct UartInst {
	USART_TypeDef *uart;
	IRQn_Type     IRQn;
	void (*tx_callback)(uint32_t);
	void (*rx_callback)(uint32_t);
	UART_cfg *cfg;
} UartInst;

/*
 * Function Prototypes
 */
void uart_init_8n1(USART_TypeDef *uart, IRQn_Type IRQn, uint16_t usartdiv);
uint16_t uart_baud_to_usartdiv(uint32_t baudrate, uint32_t periphclk, uint32_t is_ovsamp8);
void uart_enable(USART_TypeDef *, IRQn_Type);
void uart_disable(USART_TypeDef *, IRQn_Type IRQn);
void uart_sendbyte(USART_TypeDef *, uint8_t c);
void uart_send(USART_TypeDef *uart, const void *buffer, size_t length);
void uart_sendstr(USART_TypeDef *uart, char *str);


// TODO: have a blocking send (SW block), and an interrupt-driven UART transmit.
// No real way but interrupt-driven to do receives
// might have UART instance hold half vs. full duplex info
// void uart_txenable(USART_TypeDef *);
// void uart_rxenable(USART_TypeDef *);


/*
 * UART Typedefs
 */
// TODO: define these as register-wide bitmasks onto the correct bit position for wordlen
#define UART_WORDLEN_7B (0x02u) // 1 start bit, 7 data bits
#define UART_WORDLEN_8B (0x00u) // 1 start bit, 8 data bits
#define UART_WORDLEN_9B (0x01u) // 1 start bit, 9 data bits

#define UART_STOPBIT_1B  (0x0u)
#define UART_STOPBIT_0B5 (0x1u)
#define UART_STOPBIT_2B  (0x2u)
#define UART_STOPBIT_1B5 (0x3u)

#endif /* INC_UART_H_ */
