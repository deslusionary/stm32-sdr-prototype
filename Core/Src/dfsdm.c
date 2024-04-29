/*
 * dfsdm.c
 *
 *  Created on: Apr 28, 2024
 *      Author: Christopher Tinker
 */

#include "dfsdm.h"

/**
 * @brief Initialize a DFSDM channel instance for FIR decimation in STM32 AM RX.
 * 				Using interleaved mode permits 32-bit DMA accesses to DATINR, reducing
 * 				load on memory interconnect by DFSDM DMA channels.
 * @param
 */
void dfsdm_ch_init(DFSDM_Channel_TypeDef *channel)
{
	channel->CHCFGR1 &= ~DFSDM_CHCFGR1_CHEN; // Disable channel
	channel->CHCFGR1 |= DFSDMCH_DATPACK_INTERLEAVED | DFSDMCH_DATMPX_DATINR; // DATPACK, DATMPX

	uint8_t data_bitshift = 0u; // 0-31, amount to bitshift data by
	uint32_t offset = 0; // 24 bit calibration offset TODO: figure this out
	channel->CHCFGR2 |= (data_bitshift & 0x1Fu) << DFSDM_CHCFGR2_DTRBS_Pos;
	channel->CHCFGR2 |= (offset & 0xFFFFFFu) << DFSDM_CHCFGR2_OFFSET_Pos;

	channel->CHCFGR1 |= DFSDM_CHCFGR1_CHEN; // Channel enable
}

/**
 * @brief Initialize a DFSDM filter instance for FIR decimation in STM32 AM RX
 * 				chain. Not extensible or reusable for other applications.
 * @param filt DFSDM filter instance
 * @param channel DFSDM channel selection for filter instance, one of DFSDMFILT_RCH_CHx
 * @retval None
 */
void dfsdm_filt_init(DFSDM_Filter_TypeDef *filt, uint32_t channel_num)
{
	filt->FLTCR1 &= ~(DFSDM_FLTCR1_DFEN); // disable filter instance

	/*
	 * FLTCR1: Select DFSDM channel, enable read DMA channel, continuous mode
	 * Fast mode enabled.
	 * Keep reset values for FLTCR2
	 */
	filt->FLTCR1 |= channel_num; // Channel selection
	filt->FLTCR1 |= (DFSDM_FLTCR1_RDMAEN | DFSDM_FLTCR1_RCONT | DFSDM_FLTCR1_FAST);

	/*
	 * AM SDR: TODO: update these
	 * 	FastSinc filter
	 * 	200x decimation
	 * 	No additional averaging
	 */
	uint32_t fosr = 200u; // oversampling ratio
	uint32_t iosr = 0u;
	filt->FLTFCR |= (fosr & 0x3FFu) << DFSDM_FLTFCR_FOSR_Pos;
	filt->FLTFCR |= DFSDMFILT_FORD_FAST; // Sinc filter order
	filt->FLTFCR |= (iosr & 0xFFu);

	filt->FLTCR1 |= DFSDM_FLTCR1_DFEN; // DFSDMFILT enable
}

//void dfsdm_start_conversion()

void dfsdm_global_enable()
{
	DFSDM_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_DFSDMEN;
}


void dfsdm_global_disable() {
	DFSDM_Channel0->CHCFGR1 &= DFSDM_CHCFGR1_DFSDMEN;
}
