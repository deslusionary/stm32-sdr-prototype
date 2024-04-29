/*
 * dfsdm.h
 *
 *  Created on: Apr 28, 2024
 *      Author: Christopher Tinker
 */

#ifndef INC_DFSDM_H_
#define INC_DFSDM_H_

/*
 * Includes
 */
#include "stm32l476xx.h"

/*
 * Macros
 */

/*
 * DFSDMCH_DATMPX_x: DFSDM Channel Input Data mux
 */
#define DFSDMCH_DATMPX_DATINR (2u << DFSDM_CHCFGR1_DATMPX_Pos) // Data from DATINR by CPU/DMA write
/*
 * DFSDMCH_DATPACK_xx: Data packing mode for DFSDM Channel
 */
#define DFSDMCH_DATPACK_STANDARD    (0u << DFSDM_CHCFGR1_DATPACK_Pos)
#define DFSDMCH_DATPACK_INTERLEAVED (1u << DFSDM_CHCFGR1_DATPACK_Pos)
#define DFSDMCH_DATPACK_DUAL        (2u << DFSDM_CHCFGR1_DATPACK_Pos)

/*
 * DFSDMFILT_RCH_CHx: Channel selection for DFSDM Filter conversions
 */
#define DFSDMFILT_RCH_CH0 (0u << DFSDM_FLTCR1_RCH_Pos)
#define DFSDMFILT_RCH_CH1 (1u << DFSDM_FLTCR1_RCH_Pos)
#define DFSDMFILT_RCH_CH2 (2u << DFSDM_FLTCR1_RCH_Pos)
#define DFSDMFILT_RCH_CH3 (3u << DFSDM_FLTCR1_RCH_Pos)
#define DFSDMFILT_RCH_CH4 (4u << DFSDM_FLTCR1_RCH_Pos)
#define DFSDMFILT_RCH_CH5 (5u << DFSDM_FLTCR1_RCH_Pos)
#define DFSDMFILT_RCH_CH6 (6u << DFSDM_FLTCR1_RCH_Pos)
#define DFSDMFILT_RCH_CH7 (7u << DFSDM_FLTCR1_RCH_Pos)

/*
 * DFSDMFILT_FORD_xx: DFSDM Sinc filter order
 */
#define DFSDMFILT_FORD_FAST  (0u << DFSDM_FLTFCR_FORD_Pos)
#define DFSDMFILT_FORD_SINC1 (1u << DFSDM_FLTFCR_FORD_Pos)
#define DFSDMFILT_FORD_SINC2 (2u << DFSDM_FLTFCR_FORD_Pos)
#define DFSDMFILT_FORD_SINC3 (3u << DFSDM_FLTFCR_FORD_Pos)
#define DFSDMFILT_FORD_SINC4 (4u << DFSDM_FLTFCR_FORD_Pos)
#define DFSDMFILT_FORD_SINC5 (5u << DFSDM_FLTFCR_FORD_Pos)


/*
 * Function Prototypes
 */
void dfsdm_ch_init(DFSDM_Channel_TypeDef *channel);
void dfsdm_filt_init(DFSDM_Filter_TypeDef *filt, uint32_t channel_num);
void dfsdm_global_enable(void);
void dfsdm_global_disable(void);


#endif /* INC_DFSDM_H_ */
