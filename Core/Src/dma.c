/*
 * dma.c
 *
 *  Created on: Apr 7, 2024
 *      Author: Christopher Tinker
 */

#include "dma.h"

/**
 * @brief Initialize a single DMA channel.
 * @param dmach DMA channel instance handle
 * @param cfg DMA configuration object
 */
void dma_init(DMACh_Inst *dmach, DMA_Cfg *cfg)
{
	DMA_Channel_TypeDef *channel = dmach->channel;
	dma_disable(dmach);

	channel->CPAR = cfg->src_periph_addr;
	channel->CMAR = cfg->dest_mem_addr;
	channel->CNDTR = (uint16_t) cfg->transfer_len;

	channel->CCR = 0u;
	channel->CCR = cfg->mode | cfg->circular_mode | cfg->priority \
								 | cfg->mem_datasize | cfg->periph_datasize \
								 | cfg->mem_incmode | cfg->periph_incmode \
								 | cfg->irq_en;

	/* DMA_CSELR - peripheral select register */
	volatile uint32_t *dma_cselr = (uint32_t *) (((uint32_t) dmach->DMA_inst) + DMA_CSELR_REGOFFSET);
	uint32_t cselr_offset = (((uint32_t *) channel) - (((uint32_t *) dmach->DMA_inst) + 0x14u)) / 0x14u;
	*dma_cselr = (*dma_cselr & ~(0xFu << cselr_offset)) | cfg->periph_sel;
}


void dma_enable_irq(DMACh_Inst *dmach, uint32_t mask)
{
	(dmach->channel)->CCR |= (mask & 0xCu);
}


void dma_disable_irq(DMACh_Inst *dmach, uint32_t mask)
{
	(dmach->channel)->CCR &= ~(mask & 0xCu);
}


void dma_enable(DMACh_Inst *dmach)
{
	(dmach->channel)->CCR |= DMA_CCR_EN;
}


void dma_disable(DMACh_Inst *dmach)
{
	(dmach->channel)->CCR &= ~DMA_CCR_EN;
}
