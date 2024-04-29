/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_USART2_UART_Init(void);

/* Peripheral Handles */
DMACh_Inst dmach_adc = { // ADC to Mem
		.DMA_inst = DMA1,
		.channel = DMA1_Channel1,
		.IRQn = DMA1_Channel1_IRQn
 };

DMACh_Inst dmach_itodec = { // Post-DDC I samples to DFSDM decimation
		.DMA_inst = DMA2,
		.channel = DMA2_Channel1,
		.IRQn = DMA2_Channel1_IRQn
};

DMACh_Inst dmach_qtodec = { // Post-DDC Q samples to DFSDM decimation
		.DMA_inst = DMA2,
		.channel = DMA2_Channel2,
		.IRQn = DMA2_Channel2_IRQn
};

ADC_Inst adc = {
  		.adc = SDR_ADC,
			.IRQn = SDR_ADC_IRQn
  };

/*
 * Interrupt Shared Context
 */
volatile uint16_t g_adcbuf[ADC_BUFLEN] = {0};

volatile ppbuf_uint16 g_adc_ppbuf = {
	.wrbuf = g_adcbuf,
	.rdbuf = (uint16_t *) (g_adcbuf + (ADC_BUFLEN / 2))
};

volatile uint8_t g_adc_drdy = 0;

volatile uint16_t g_isampbuf[SDRBLOCKLEN] = {0}; // make this a ping-pong buffer
volatile uint16_t g_qsampbuf[SDRBLOCKLEN] = {0}; // make this a ping-pong buffer


/* USER CODE END PV */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */


void sdr_dma_init()
{
  DMA_Cfg dmach_adc_cfg = {
  		.src_periph_addr = (uint32_t) &(SDR_ADC->DR),
			.dest_mem_addr = (uint32_t) g_adcbuf,
			.transfer_len = ADC_BUFLEN,
  		.periph_sel = LL_DMA_REQUEST_0,
			.mode = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
			.circular_mode = LL_DMA_MODE_CIRCULAR,
			.priority = LL_DMA_PRIORITY_VERYHIGH,
			.mem_datasize = LL_DMA_MDATAALIGN_HALFWORD,
			.periph_datasize = LL_DMA_PDATAALIGN_HALFWORD,
			.mem_incmode = LL_DMA_MEMORY_INCREMENT,
			.periph_incmode = LL_DMA_PERIPH_NOINCREMENT,
			.irq_en = DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_HTIE
  };
  dma_init(&dmach_adc, &dmach_adc_cfg);

  // Configure M2M DMA from DDC I samples to DFSDM FIR decimator
  DMA_Cfg dmach_itodec_cfg = {
    		.src_periph_addr = (uint32_t) g_isampbuf,
  			.dest_mem_addr = (uint32_t) &(ISAMP_DFSDM_CH->CHDATINR),
  			.transfer_len = SDRBLOCKLEN,                              // TODO: fix
    		.periph_sel = LL_DMA_REQUEST_0,
  			.mode = LL_DMA_DIRECTION_MEMORY_TO_MEMORY,
  			.circular_mode = LL_DMA_MODE_NORMAL,
  			.priority = LL_DMA_PRIORITY_HIGH,
  			.mem_datasize = LL_DMA_MDATAALIGN_WORD,
  			.periph_datasize = LL_DMA_PDATAALIGN_WORD,
  			.mem_incmode = LL_DMA_MEMORY_INCREMENT,
  			.periph_incmode = LL_DMA_PERIPH_NOINCREMENT,
  			.irq_en = DMA_CCR_TCIE | DMA_CCR_TEIE
    };
  dma_init(&dmach_itodec, &dmach_itodec_cfg);

  DMA_Cfg dmach_qtodec_cfg = {
  		.src_periph_addr = (uint32_t) g_qsampbuf,
    	.dest_mem_addr = (uint32_t) &(QSAMP_DFSDM_CH->CHDATINR),
   		.transfer_len = SDRBLOCKLEN,                               // TODO: fix
      .periph_sel = LL_DMA_REQUEST_0,
   		.mode = LL_DMA_DIRECTION_MEMORY_TO_MEMORY,
    	.circular_mode = LL_DMA_MODE_NORMAL,
   		.priority = LL_DMA_PRIORITY_HIGH,
    	.mem_datasize = LL_DMA_MDATAALIGN_WORD,
   		.periph_datasize = LL_DMA_PDATAALIGN_WORD,
    	.mem_incmode = LL_DMA_MEMORY_INCREMENT,
   		.periph_incmode = LL_DMA_PERIPH_NOINCREMENT,
  		.irq_en = DMA_CCR_TCIE | DMA_CCR_TEIE
      };
  dma_init(&dmach_qtodec, &dmach_qtodec_cfg);
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // USART2
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN; // USART3
  RCC->AHB2ENR  |= RCC_AHB2ENR_ADCEN; // ADC
  RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN; // DMA1
  RCC->AHB1ENR  |= RCC_AHB1ENR_DMA2EN; // DMA2
  RCC->APB2ENR  |= RCC_APB2ENR_DFSDM1EN; // DFSDM1

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* GPIO Initialization */
  lwio_cfg console_tx_cfg = {
  	.port = CONSOLE_UART_TX_PORT,
		.pin = CONSOLE_UART_TX_PIN,
		.mode = LWIO_AF, // alternate function for USART2
		.otype = LWIO_PUSHPULL,
		.pupd = LWIO_NOPULL,
		.ospeed = LWIO_SPEED_MED,
		.afsel = LWIO_AF_7
  };

  lwio_cfg console_rx_cfg = {
  		.port = CONSOLE_UART_RX_PORT,
			.pin = CONSOLE_UART_RX_PIN,
			.mode = LWIO_AF,
			.otype = LWIO_PUSHPULL,
			.pupd = LWIO_NOPULL,
			.ospeed = LWIO_SPEED_MED,
			.afsel = LWIO_AF_7
  };

  lwio_cfg adc_rx_cfg = {
  		.port = ADC_RX_PORT,
			.pin = ADC_RX_PIN,
			.mode = LWIO_ANALOG,
			.otype = LWIO_NOOUTPUT,
			.pupd = LWIO_NOPULL,
			.ospeed = LWIO_SPEED_NONE,
			.afsel = LWIO_AF_0
  };

  lwio_cfg_init(&console_tx_cfg);
  lwio_cfg_init(&console_rx_cfg);
  lwio_cfg_init(&adc_rx_cfg);

  lwio_init(PWM_PORT, PWM_PIN, LWIO_OUTPUT, LWIO_PUSHPULL, LWIO_NOPULL, LWIO_SPEED_MED);
  lwio_init(LD2_PORT, LD2_PIN, LWIO_OUTPUT, LWIO_PUSHPULL, LWIO_NOPULL, LWIO_SPEED_LOW); // LD2

  // LED

  /* Peripheral Initialization */
  // uint16_t usartdiv = uart_baud_to_usartdiv(115200, 80e6, 0);
  uint16_t usartdiv = 694; // 80e6 / 115200
  uart_init_8n1(CONSOLE_UART, CONSOLE_UART_IRQn, usartdiv);

  // DFSDM
  dfsdm_ch_init(ISAMP_DFSDM_CH);
  dfsdm_ch_init(QSAMP_DFSDM_CH);
  dfsdm_filt_init(ISAMP_DFSDM_FILT, DFSDMFILT_RCH_CH0); // Ch0 w/ Filt0 - ISAMP
  dfsdm_filt_init(QSAMP_DFSDM_FILT, DFSDMFILT_RCH_CH1); // Ch1 w/ Filt1 - QSAMP

  sdr_dma_init();
  sdr_adc_init(&adc);

  /* Interrupt Initialization */
   NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  /* Application */
  uart_sendstr(CONSOLE_UART, "Data Out:\r\n");
  dma_enable(&dmach_adc);
  adc_start(&adc);

//  char uartbuf[201] = {0};
//  for (int i = 0; i < 64; i++) {
//  	snprintf(uartbuf, 200, "%hu\r\n", adcbuf[i]);
//  	uart_sendstr(CONSOLE_UART, uartbuf);
//  }
  int cnt = 0;
  while (1) {
  	while(!g_adc_drdy);
  	__disable_irq();
  	g_adc_drdy = 0;
  	__enable_irq();

  	cnt++;
  	lwioTogglePin(PWM_PORT, PWM_PIN);

//  	if (cnt == 2) {
//  		__disable_irq();
//  		for (int i = 0; i < ADC_BUFLEN; i++) {
//  			char uartbuf[50];
//  			snprintf(uartbuf, 200, "%d: %hu\r\n", i, g_adcbuf[i]);
//  		  uart_sendstr(CONSOLE_UART, uartbuf);
//  		}
//  		while (1);
//  	}
  }
}

static inline void swap_ppbuf_uint16(volatile ppbuf_uint16 *ppbuf) {
	uint16_t *tmp = ppbuf->rdbuf;
	ppbuf->rdbuf = ppbuf->wrbuf;
	ppbuf->wrbuf = tmp;
}


/**
 * @brief GPIO initialization routine for the STM32 SDR.
 * @param None
 * @retval None
 */
//void sdr_gpio_init()
//{
//
//}

void DMA1_Channel1_IRQHandler()
{
	NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
	uint32_t isr = DMA1->ISR;

	if (isr & DMA_ISR_TEIF1) {
		uart_sendstr(CONSOLE_UART, "DMA CH 1 Error\n");
		Error_Handler();
	}

	// Callback stuff on the status word only here?
	swap_ppbuf_uint16(&g_adc_ppbuf);

	if (g_adc_drdy != 0) Error_Handler();
	g_adc_drdy = 1u;

	DMA1->IFCR = DMA_IFCR_CGIF1;
	//NVIC_DisableIRQ(DMA1_Channel1_IRQn);
}


/**
 * @brief USART2 interrupt service routine.
 * @retval None
 */
void USART2_IRQHandler()
{

	// Clear Pending IRQ in NVIC
	NVIC_ClearPendingIRQ(USART2_IRQn);

	// Read ISR status
	uint32_t status = USART2->ISR;

	// handle and clear interrupt flags
	// do callbacks usually deal with resetting flags themselves?
	if (status & USART_ISR_RXNE) {
		uart_rxecho_cb(USART2, status);
	} else {
		// TODO: do something? light an LED? take note of getting a different interrupt?
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_ADC1_Init(void)
//{
//
//  /* USER CODE BEGIN ADC1_Init 0 */
////
//  /* USER CODE END ADC1_Init 0 */
//
//  LL_ADC_InitTypeDef ADC_InitStruct = {0};
//  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
//  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
//
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
//
//  /** Initializes the peripherals clock
//  */
//  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
//  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Peripheral clock enable */
//  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);
//
//  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
//  /**ADC1 GPIO Configuration
//  PC0   ------> ADC1_IN1
//  */
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  LL_GPIO_EnablePinAnalogControl(GPIOC, LL_GPIO_PIN_0);
//
//  /* USER CODE BEGIN ADC1_Init 1 */
////
//  /* USER CODE END ADC1_Init 1 */
//
//  /** Common config
//  */
//  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
//  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
//  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
//  LL_ADC_Init(ADC1, &ADC_InitStruct);
//  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
//  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
//  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
//  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
//  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
//  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
//  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
//  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
//  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
//  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
//  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);
//
//  /* Disable ADC deep power down (enabled by default after reset state) */
//  LL_ADC_DisableDeepPowerDown(ADC1);
//  /* Enable ADC internal voltage regulator */
//  LL_ADC_EnableInternalRegulator(ADC1);
//  /* Delay for ADC internal voltage regulator stabilization. */
//  /* Compute number of CPU cycles to wait for, from delay in us. */
//  /* Note: Variable divided by 2 to compensate partially */
//  /* CPU processing cycles (depends on compilation optimization). */
//  /* Note: If system core clock frequency is below 200kHz, wait time */
//  /* is only a few CPU processing cycles. */
//  uint32_t wait_loop_index;
//  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
//  while(wait_loop_index != 0)
//  {
//    wait_loop_index--;
//  }
//  /* USER CODE BEGIN ADC1_Init 2 */
////
//  /* USER CODE END ADC1_Init 2 */
//
//}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_USART2_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART2_Init 0 */
////
//  /* USER CODE END USART2_Init 0 */
//
//  /* USER CODE BEGIN USART2_Init 1 */
////
//  /* USER CODE END USART2_Init 1 */
//  huart2.Instance = USART2;
//  huart2.Init.BaudRate = 115200;
//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.StopBits = UART_STOPBITS_1;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  huart2.Init.Mode = UART_MODE_TX_RX;
//  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART2_Init 2 */
////
//  /* USER CODE END USART2_Init 2 */
//
//}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
///* USER CODE BEGIN MX_GPIO_Init_1 */
///* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : B1_Pin */
//  GPIO_InitStruct.Pin = B1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : LD2_Pin */
//  GPIO_InitStruct.Pin = LD2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
//
///* USER CODE BEGIN MX_GPIO_Init_2 */
///* USER CODE END MX_GPIO_Init_2 */
//}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
