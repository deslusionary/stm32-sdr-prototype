/*
 * dma.h
 *
 *  Created on: Apr 10, 2024
 *      Author: Christopher Tinker
 *
 *      Driver for STM32L4 7 channel DMA.
 *      Designed to be compatible with macros from the
 *      STM32 LL DMA driver to save time (instead of having
 *      to define new macros).
 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

/* Includes */
#include <stdint.h>
#include "stm32l476xx.h"

/* Macros */
#define DMA_CSELR_REGOFFSET (0x00A8UL)

/* Data Structures */
typedef struct {
	uint32_t src_periph_addr; /* Peripheral or M2M source address */
	uint32_t dest_mem_addr;   /* P2M/M2M destination address */
	uint32_t transfer_len;    /* Data transfer length */
	uint32_t periph_sel;      /* Peripheral selection in DMA_CSELR */
	uint32_t mode;            /* P2M, M2P, or M2M */
	uint32_t circular_mode;   /* Normal or Circular mode */
	uint32_t priority;        /* DMA channel priority */
	uint32_t mem_datasize;
	uint32_t periph_datasize;
	uint32_t mem_incmode;
	uint32_t periph_incmode;
	uint32_t irq_en;
} DMA_Cfg;


/**
 * @brief Handle for a single DMA channel.
 */
typedef struct {
	DMA_TypeDef *DMA_inst;
	DMA_Channel_TypeDef *channel;
	IRQn_Type IRQn;
} DMACh_Inst;

/* Function Prototypes */
void dma_init(DMACh_Inst *dmach, DMA_Cfg *cfg);
void dma_enable(DMACh_Inst *dmach);
void dma_disable(DMACh_Inst *dmach);

#endif /* INC_DMA_H_ */
