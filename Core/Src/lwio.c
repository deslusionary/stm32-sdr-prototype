/*
 * lwio.c
 *
 *  Created on: Jan 31, 2024
 *      Author: Christopher Tinker
 *
 *      Lightweight, generic GPIO driver (or helper code?) for the STM32L4 GPIO peripheral.
 */

#include <lwio.h>

/**
 * @brief Basic initialization function for a single GPIO pin.
 * TODO: rest of docs
 */
void lwio_init(GPIO_TypeDef *port, uint32_t pin, uint8_t mode, uint8_t otype, uint8_t pupd, uint8_t ospeed)
{
	uint32_t num = lwioPinToNum(pin);
	uint8_t cfg_pos = num << 1u; // bitshift amount for MODER, PUPDR, OSPEEDR

	port->MODER = (port->MODER & ~(0x3u << cfg_pos)) | (mode << cfg_pos);
	port->OTYPER = (port->OTYPER & (0x1u << num)) | (otype << num);
	port->PUPDR = (port->PUPDR & ~(0x3u << cfg_pos)) | (pupd << cfg_pos);
	port->OSPEEDR = (port->OSPEEDR & ~(0x3u << cfg_pos)) | (ospeed << cfg_pos);

	// Connect analog switch to ADC input if GPIO analog mode selected
	if (mode == LWIO_ANALOG) {
		port->ASCR |= (0x1u << num);
	}
}


void lwio_cfg_init(lwio_cfg *iocfg) {
	/*
	 * Alternate method for initializing GPIO, using a configuration struct.
	 * TODO: should this struct control analog switch control? or should the user be expected to configure that elsewhere
	 */
	uint32_t num = lwioPinToNum(iocfg->pin);
	uint32_t cfg_pos = (num) << 1u;
	GPIO_TypeDef *port = iocfg->port;

	port->MODER = (port->MODER & ~(0x3u << cfg_pos)) | (iocfg->mode << cfg_pos);
	port->OTYPER = (port->OTYPER & (0x1u << num)) | (iocfg->otype << num);
	port->PUPDR = (port->PUPDR & ~(0x3u << cfg_pos)) | (iocfg->pupd << cfg_pos);
	port->OSPEEDR = (port->OSPEEDR & ~(0x3u << cfg_pos)) | (iocfg->ospeed << cfg_pos);

	// Alternate Function select TODO: figure out the clever way of doing this
	if (iocfg->mode == LWIO_AF) {
		if (num < 8) { // IO 0-7, use AFRL
			uint32_t afrl_pos = num << 2; // AFRL bitfield position for given IO NUM
			port->AFR[0] = (port->AFR[0] & ~(0xFu << afrl_pos)) | (iocfg->afsel << afrl_pos);
		}
		else { // IO 8-15, use AFRH
			uint32_t afrh_pos = (num - 8) << 2; // AFRH bitfield position for given IO num, e.g. IO 9 --> afrh_pos = 4
			port->AFR[1] = (port->AFR[1] & ~(0xFu << afrh_pos)) | (iocfg->afsel << afrh_pos);
		}
	}

	// Connect analog switch to ADC input if GPIO analog mode selected
	else if (iocfg->mode == LWIO_ANALOG) {
		port->ASCR |= (1u << num);
	}
}

/**
 * @brief Set mode (MODER, and if MODE_ANALOG, ASCR) for a single GPIO pin.
 * @param port GPIO peripheral
 * @param pin  GPIO pin bitmask
 * @param mode GPIO mode - LWIO_OUTPUT, LWIO_INPUT, LWIO_AF, LWIO_ANALOG
 * @retval none
 */
void lwio_set_mode(GPIO_TypeDef *port, uint32_t pin, uint8_t mode)
{
	uint32_t num = lwioPinToNum(pin);
	uint8_t cfg_pos = num << 1u; // bitshift amount for MODER, PUPDR, OSPEEDR

	port->MODER = (port->MODER & ~(0x3u << cfg_pos)) | (mode << cfg_pos);
	// Connect analog switch to ADC input if GPIO analog mode selected
	if (mode == LWIO_ANALOG) {
		port->ASCR |= (0x1u << num);
	}
}


/**
 * @brief Set GPIO alternate function selection.
 */
void lwio_set_afsel(GPIO_TypeDef *port, uint32_t pin, uint8_t afsel)
{
	uint32_t num = lwioPinToNum(pin);

	if (num < 8) { // IO 0-7, use AFRL
		uint32_t afrl_pos = num << 2; // AFRL bitfield position for given IO NUM
		port->AFR[0] = (port->AFR[0] & ~(0xFu << afrl_pos)) | (afsel << afrl_pos);
	}
	else { // IO 8-15, use AFRH
		uint32_t afrh_pos = (num - 8) << 2; // AFRH bitfield position for given IO num, e.g. IO 9 --> afrh_pos = 4
		port->AFR[1] = (port->AFR[1] & ~(0xFu << afrh_pos)) | (afsel << afrh_pos);
	}
}
