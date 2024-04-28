/*
 * adc.h
 *
 *  Created on: Mar 15, 2024
 *      Author: Christopher Tinker
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

/*
 * Includes
 */
#include <stdint.h>
#include "stm32l476xx.h"
#include "stm32l4xx_ll_adc.h" /* Use HAL Macros for simplicity */

/*
 * Typedefs and Data types
 */
typedef struct {
	// Unused for now, may eventually find use
} ADC_Cfg;

/*
 * Handle for ADC peripheral instance.
 */
typedef struct {
	ADC_TypeDef *adc; /* Hardware address of ADC periperal */
	IRQn_Type IRQn; /* ADC NVIC Interrupt Request Number */
	//ADC_Cfg *config; /* ADC configuration and initialization */
} ADC_Inst;

/*
 * Function Prototypes
 */
//void adc_init(ADC_Inst *inst);
void sdr_adc_init(ADC_Inst *inst);
void adc_enable(ADC_Inst *inst);
void adc_disable(ADC_Inst *inst);
void adc_start(ADC_Inst *inst);
void adc_stop(ADC_Inst *inst);

/*
 * Macros
 */

#endif /* INC_ADC_H_ */
