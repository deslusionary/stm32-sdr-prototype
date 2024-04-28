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

//UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // USART2 clock enable
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN; // USART3
  RCC->AHB2ENR  |= RCC_AHB2ENR_ADCEN; // ADC
  RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN; // DMA1

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_USART2_UART_Init();

  /* GPIO Initialization */
  lwio_cfg usart_tx_cfg = {
  	.port = UART_TX_PORT,
		.pin = UART_TX_PIN,
		.mode = LWIO_AF, // alternate function for USART2
		.otype = LWIO_PUSHPULL,
		.pupd = LWIO_NOPULL,
		.ospeed = LWIO_SPEED_MED,
		.afsel = LWIO_AF_7
  };

  lwio_cfg usart_rx_cfg = {
  		.port = UART_RX_PORT,
			.pin = UART_RX_PIN,
			.mode = LWIO_AF,
			.otype = LWIO_PUSHPULL,
			.pupd = LWIO_NOPULL,
			.ospeed = LWIO_SPEED_MED,
			.afsel = LWIO_AF_7
  };

  lwio_cfg adc_rxin_cfg = {
  		.port = ADC_RXIN_PORT,
			.pin = ADC_RXIN_PIN,
			.mode = LWIO_ANALOG,
			.otype = LWIO_NOOUTPUT,
			.pupd = LWIO_NOPULL,
			.ospeed = LWIO_SPEED_NONE,
			.afsel = LWIO_AF_0
  };

  lwio_cfg_init(&usart_tx_cfg);
  lwio_cfg_init(&usart_rx_cfg);
  lwio_cfg_init(&adc_rxin_cfg);

  lwio_init(PWM_GPIO_PORT, PWM_GPIO_PIN, LWIO_OUTPUT, LWIO_PUSHPULL, LWIO_NOPULL, LWIO_SPEED_MED);

  /* Peripheral Initialization */
  // uint16_t usartdiv = uart_baud_to_usartdiv(115200, 80e6, 0);
  uint16_t usartdiv = 694; // 80e6 / 115200
  uart_init_8n1(UART_PERIPH, UART_PERIPH_IRQN, usartdiv);

  DMACh_Inst dmach_adcdma = {
  		.DMA_inst = DMA1,
			.channel = DMA1_Channel1,
			.IRQn = DMA1_Channel1_IRQn
  };

  /*
   * Configure DMA channel 1 for continuous conversions from ADC1
   */
  uint16_t adcbuf[64] = {0};

  DMA_Cfg dmach_adcdma_cfg = {
  		.src_periph_addr = (uint32_t) &(ADC1->DR),
			.dest_mem_addr = (uint32_t) adcbuf,
			.transfer_len = 64u,
  		.periph_sel = LL_DMA_REQUEST_0,
			.mode = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
			.circular_mode = LL_DMA_MODE_CIRCULAR,
			.priority = LL_DMA_PRIORITY_VERYHIGH,
			.mem_datasize = LL_DMA_MDATAALIGN_HALFWORD,
			.periph_datasize = LL_DMA_PDATAALIGN_HALFWORD,
			.mem_incmode = LL_DMA_MEMORY_INCREMENT,
			.periph_incmode = LL_DMA_PERIPH_NOINCREMENT,
			.irq_en = DMA_CCR_TCIE | DMA_CCR_TEIE
  };

  dma_init(&dmach_adcdma, &dmach_adcdma_cfg);

  ADC_Inst adc = {
  		.adc = ADC1,
			.IRQn = ADC1_IRQn
  };

  sdr_adc_init(&adc);

  /* Interrupt Initialization */
   NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  /* Application */
  dma_enable(&dmach_adcdma);
  adc_start(&adc);

  uart_sendstr(UART_PERIPH, "Data Out:");
  char uartbuf[201] = {0};
  for (int i = 0; i < 64; i++) {
  	snprintf(uartbuf, 200, "%hu\r\n", adcbuf[i]);
  	uart_sendstr(UART_PERIPH, uartbuf);
  }
//  //char *test = "Hello\n";
//  uart_sendstr(UART_PERIPH, "\e[H\e[3B\e[5C");
//  uart_sendstr(UART_PERIPH, "All good students read the");
//  uart_sendstr(UART_PERIPH, "\e[1B\e[21D");
//  uart_sendstr(UART_PERIPH, "\e[5m");
//  uart_sendstr(UART_PERIPH, "Reference Manual");
//  uart_sendstr(UART_PERIPH, "\e[H\e[0m");
//  uart_sendstr(UART_PERIPH, "Input:");

  while (1)
  {
  	HAL_Delay(500);
  	//uart_send(UART_PERIPH, test, 6); // Send only first 3 characters, not null byte
  	//uart_sendbyte(UART_PERIPH, 'A');
  	//uart_sendstr(UART_PERIPH, test);
  	lwioTogglePin(LD2_GPIO_Port, LD2_Pin);
  }
}

void DMA1_Channel1_IRQHandler() {
	NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
	NVIC_DisableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief USART2 interrupt service routine.
 * @retval None
 */
void USART2_IRQHandler() {

	// Clear Pending IRQ in NVIC
	NVIC_ClearPendingIRQ(USART2_IRQn);
	lwioSetPin(PWM_GPIO_PORT, PWM_GPIO_PIN);

	// Read ISR status
	uint32_t status = USART2->ISR;

	// handle and clear interrupt flags
	// do callbacks usually deal with resetting flags themselves?
	if (status & USART_ISR_RXNE) {
		uart_rxecho_cb(USART2, status);
	} else {
		// TODO: do something? light an LED? take note of getting a different interrupt?
	}
	lwioClearPin(PWM_GPIO_PORT, PWM_GPIO_PIN);

}

/**
 * @brief UART RX interrupt callback function. Echoes received byte and issues VT100 color change commands.
 * 				Assumes 8N1 UART.
 * @param uart pointer to USART peripheral
 * @param status USART interrupt status
 * @retval None
 */
void uart_rxecho_cb(USART_TypeDef *uart, uint32_t status)
{
	// This is really too much stuff to do within an ISR. Future work would be to refactor such that all
	// this processing is done in main(), with the UART RX interrupt writing the received byte into a single
	// byte buffer for main() to process

	char c = (char) (0xFFu & uart->RDR); // Are typecasts useful here? since RDR is a uint16_t

	//uart_sendbyte(uart, c);

	char *txstr;
	switch (c) {
		case 'R':
			txstr = "\e[31m";
			uart_sendstr(uart, txstr);
			break;

		case 'G':
			txstr = "\e[32m";
			uart_sendstr(uart, txstr);
			break;

		case 'B':
			txstr = "\e[34m";
			uart_sendstr(uart, txstr);
			break;

		default:
			uart_sendbyte(uart, c);
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

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
