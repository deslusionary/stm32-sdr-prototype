/*
 * uart.c
 *
 *  Created on: Mar 3, 2024
 *      Author: Christopher Tinker
 *
 *      Low-level driver for STM32 USART peripheral.
 *      TX - blocking in SW
 *      RX - interrupt driven (?)
 */

#include "uart.h"

void uart_init_8n1(USART_TypeDef *uart, IRQn_Type IRQn, uint16_t usartdiv) {
	/*
	 * TODO: documentation
	 * TODO: consider enabling UART interrupt within the UART init?
	 * TODO: consider whether uart_init_8n1 should get a baudrate or a usartdiv
	 *
	 * Currently TX-related interrupts (TXE, TC) not configured (for SW blocking TX) (?)
	 *
	 * uart_init_8n1 provides a lightweight way of enabling the give USART peripheral
	 * to act as a typical 8N1 bidirectional UART with no frills (8N1 -- 8 bit word length,
	 * no parity, 1 stop bit). User is responsible for determining the correct value of
	 * USARTDIV to program the USART baud rate register for the desired baud rate.
	 */
	// make sure UART is disabled
	// Restore control registers to reset values
	uart->CR1 &= ~(USART_CR1_UE); // disable UART TODO handle where UART still tx'ing - TC, TE bits
	uart->CR1 = 0u; // per STM32L4 TRM
	uart->CR2 = 0u; // per TRM
	uart->CR3 = 0u; // per TRM

	// Initializing for 8N1 -- 8 bit word, no parity, 1 stop -- is handled
	// by default register values
	// UART interrupt enables
	//uart->CR1 |= (USART_CR1_TCIE | USART_CR1_TXIE); // TXE - TDR empty, TC - transmission complete
	uart->CR1 |= USART_CR1_RXNEIE; // RXNE - RX register not empty

	// Program baud rate
	uart->BRR = usartdiv;

	//uart->CR1 |= (USART_CR1_TE | USART_CR1_RE); // UART TX, RX enables
	//uart->CR1 |= (USART_CR1_TE);
	NVIC_EnableIRQ(IRQn); // USART interrupt enable
	uart->CR1 |= USART_CR1_UE; // Enable UART
	uart->CR1 |= (USART_CR1_TE | USART_CR1_RE); // UART TX, RX enables
}


uint16_t uart_baud_to_usartdiv(uint32_t baudrate, uint32_t periphclk, uint32_t is_ovsamp8) {
	/*
	 * Calculate the USARTDIV value to program the STM32 USART_BRR register, given
	 * a baud rate, UART/USART peripheral clock frequency, and if the USART peripheral
	 * is configured for 8x or 16x oversampling.
	 */
	uint16_t usartdiv;
	if (is_ovsamp8) usartdiv = (periphclk * 2) / baudrate;
	else            usartdiv = periphclk / baudrate;
	return usartdiv;
}


// TODO: consider changing USART_TypeDef *uart to USART_TypeDef *inst, *usart
// TODO: should interrupt enables be part of the uart_enable? Maybe those
// should be handled separately. Maybe we don't want the UART interrupt?
void uart_enable(USART_TypeDef *uart, IRQn_Type IRQn) {
	/*
	 * Enable a UART/USART hardware peripheral and its associated
	 * interrupt vector. Enables both TX and RX.
	 */
	// USART enable
	uart->CR1 |= USART_CR1_UE;
	// Enable USART interrupt vector
	NVIC_EnableIRQ(IRQn);
}


void uart_disable(USART_TypeDef *uart, IRQn_Type IRQn) {
	/*
	 * Disable a UART/USART peripheral. Per STM32L4 TRM, it is mandatory
	 * to wait until the TC bit is set (=1) before disabling the UART.
	 * For consistency with uart_enable, uart_disable will disable
	 * the UART interrupt vector -- but not sure this is necessary?
	 * TODO: think about this -- why?
	 */
	while (!(uart->ISR & USART_ISR_TC)); // block until TC = 1
	uart->CR1 &= ~(USART_CR1_TE); // See STM32L4 TRM p.1369
	uart->CR1 &= ~(USART_CR1_UE);
	NVIC_DisableIRQ(IRQn);
}

void uart_sendbyte(USART_TypeDef *uart, uint8_t c) {
	/*
	 * Transmit a single byte c on the specified UART peripheral.
	 */
	uart->CR1 |= USART_CR1_TE; // send idle frame as first transmission

	while (!(uart->ISR & USART_ISR_TXE)); // block while TX register not empty - TXE not set
	uart->TDR = (uint32_t) c;
}

void uart_send(USART_TypeDef *uart, const void *buffer, size_t length) {
	/*
	 * Transmit 'length' number of bytes pointed to by *buffer on the given
	 * UART/USART peripheral.
	 *
	 * Blocking, SW-only (non-interrupt UART send for now)
	 */
	// Setup - need to wait for
	//uart->CR1 |= USART_CR1_TE;
	// Transmit
	for (int i = 0; i < length; i++) {
		uint8_t *c = (uint8_t *) buffer + i;

		while (!(uart->ISR & USART_ISR_TXE)); // block while TX register not empty - TXE not set
		uart->TDR = (uint32_t) *c;
	}
	// Teardown - wait until TC complete flag
	while (!(uart->ISR & USART_ISR_TC));
	uart->ICR = USART_ICR_TCCF;
}


void uart_sendstr(USART_TypeDef *uart, char *str) {
	/*
	 * Send a null-terminated string over the UART/USART peripheral specified by
	 * *uart.
	 */
	int i = 0;
	char c;
	while((c = str[i]) != '\0') {
		while(!(uart->ISR & USART_ISR_TXE)); // block while TX register not empty
		uart->TDR = c;
		i++;
	}

	while(!(uart->ISR & USART_ISR_TC));
	uart->ICR = USART_ICR_TCCF;
}





