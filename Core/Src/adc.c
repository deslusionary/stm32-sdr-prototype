/*
 * adc.c
 *
 *  Created on: Mar 15, 2024
 *      Author: Christopher Tinker
 */

#include "adc.h"


/**
 * @brief Initialize ADC for SDR. While parts of this code are written in a generalizable way
 * 				for a possible ADC driver, this function is a one-off for the SDR.
 * 				Enabling ADC clocks happens elsewhere, as does GPIO init, interrupts, etc.
 * 				Assumes all ADC register are in reset state.
 *
 * @param ADC_Inst ADC peripheral instance and state
 * @retval None
 */
void sdr_adc_init(ADC_Inst *inst)
{
	/* Bring ADC out of deep power-down mode, enable regulator */
	ADC_TypeDef *adc = inst->adc; /* Hardware address of ADC peripheral */

	/* TODO: ensure ADC is disabled before attempting initialization */
	adc->CR &= ~(ADC_CR_DEEPPWD); /* Exit deep-power-down mode */
	adc->CR |= ADC_CR_ADVREGEN; /* Enable ADC voltage regulator */
	for (int i = 0; i < 64; i++); /* STM32L4 datasheet: wait for at least 1 conv cycle */

	/* ADC Clocking */
	ADC123_COMMON->CCR |= (0x1u << ADC_CCR_CKMODE_Pos); /* HCLK/1 Synchronous Clock - 80 MHz */
	// No prescaler

	/* ADC Differential Mode Selection */
	adc->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_1); /* Configure channel 1 in single-ended mode */

	/* ADC Calibration */
	adc->CR &= ~(ADC_CR_ADCALDIF); /* Single-ended calibration */
	adc->CR |= ADC_CR_ADCAL; /* Start ADC calibration */

	while ((adc->CR & ADC_CR_ADCAL)); /* Block until ADCAL = 0 -- calibration complete */

	/* ADC Enable (set ADEN = 1), among other things.
	 * Control bits related to configuration of regular and injected conversions
	 * can only be set after the ADC is enabled.
	 */
	adc_enable(inst);

	/* Interrupt Configuration */
	adc->IER |= ADC_IER_OVRIE; /* Overrun interrupt enable */
	// Add analog watchdog interrupts here

	/* DMA Configuration */
	adc->CFGR |= (ADC_CFGR_DMACFG | ADC_CFGR_DMAEN); /* Enable DMA in DMA circular mode */

	/* Modes - DISCEN, CONT, AUTDLY.
	 *  Given SYSCLK = 80 MHz, ADC *should* run at a sampling rate
	 *  Fs = 4 MHz if a timer is configured with an external trigger/
	 */
	// discontinuous mode enable
	// enable external trigger
	// select external trigger source

	/* FOR ADC bringup: run ADC at Fs = 3.2 MHz with a 25 cycle conversion time.
	 * Tconv = 25 cycles (for SYSCLK = 80 MHz) = 12.5 + 12.5
	 */
	adc->CFGR |= ADC_CFGR_CONT; /* Enable continuous conversion mode */
	adc->CFGR &= ~(ADC_CFGR_EXTEN); /* SW trigger only */
	//adc->SMPR1 |= (0x2u << ADC_SMPR1_SMP5_Pos); /* Add 12.5 additional cycles to sampling time */

	/* Configure regular conversion sequence.
	 * For SDR, each sequence is a single conversion on ADC12_IN5 (ADC1_IN5)
	 */
	adc->SQR1 = (adc->SQR1 & ~ADC_SQR1_L_Msk) | (0x0u << ADC_SQR1_L_Pos); /* Sequence Length = 1 */
	adc->SQR1 |= (0x5u << ADC_SQR1_SQ1_Pos); /* 1st Conversion: Channel 5 */

	/* Enable ADC Interrupts */
	NVIC_EnableIRQ(inst->IRQn);
}


/**
 * @brief Enable the ADC peripheral following STM32L4 TRM procedure.
 * @param ADC_Inst ADC peripheral instance and state
 * @retval None
 */
void adc_enable(ADC_Inst *inst)
{
	ADC_TypeDef *adc = inst->adc;

	adc->ISR |= ADC_ISR_ADRDY; /* Clear ADC ready bit */
	adc->CR |= ADC_CR_ADEN; /* Set ADC enable bit */

	while (!(adc->ISR & ADC_ISR_ADRDY)); /* Block until ADRDY = 1 */
	adc->ISR |= ADC_ISR_ADRDY; /* Clear ADC ready bit */
}


/**
 * @brief Disable ADC peripheral, following STM32L4 TRM guidelines.
 * @param ADC_Inst ADC peripheral instance and state
 * @retval None
 */
void adc_disable(ADC_Inst *inst)
{
	/// TODO, following the TRM
}


/**
 * @brief Start regular ADC conversions by setting ADCSTART.
 * @param ADC_Inst ADC peripheral instance and state
 * @retval None
 */
void adc_start(ADC_Inst *inst)
{
	(inst->adc)->CR |= ADC_CR_ADSTART;
}


/**
 * @brief Stop regular ADC conversions.
 * @param ADC_Inst ADC peripheral instance and state
 * @retval None
 */
void adc_stop(ADC_Inst *inst)
{
	ADC_TypeDef *adc = inst->adc;
	if (!(adc->CR & ADC_CR_ADDIS) && (adc->CR & ADC_CR_ADSTART)) adc->CR |= ADC_CR_ADSTP;
}


// TODO docs
void adc_calibrate(ADC_Inst *inst)
{
	// Disable ADC
	// Configure for single-ended or differential
	// Start calibration
	// return calibration word? idk
}
