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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TAU 6.2831853071795864769252867665590
#define kTAU 0.0062831853071795864769252867665590
#define DMASIZE 300

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint16_t dmabuffer[DMASIZE];
static uint8_t i2cbuffer[8];
static uint32_t mHz = 1022;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 uint32_t scale[133] = {
     1022,1083,1147,1215,1288,1364,1445,1531,1622,1719,1821,1929,2044,2165,2294,2431,
     2575,2728,2891,3062,3245,3437,3642,3858,4088,4331,4589,4861,5150,5457,5781,6125,
     6489,6875,7284,7717,8176,8662,9177,9723,10301,10913,11562,12250,12978,13750,14568,
     15434,16352,17324,18354,19445,20602,21827,23125,24500,25957,27500,29135,30868,32703,
     34648,36708,38891,41203,43654,46249,48999,51913,55000,58270,61735,65406,69296,73416,
     77782,82407,87307,92499,97999,103826,110000,116541,123471,130813,138591,146832,155563,
     164814,174614,184997,195998,207652,220000,233082,246942,261626,277183,293665,311127,
     329628,349228,369994,391995,415305,440000,466164,493883,523251,554365,587330,622254,
     659255,698456,739989,783991,830609,880000,932328,987767,1046502,1108731,1174659,
     1244508,1318510,1396913,1479978,1567982,1661219,1760000,1864655,1975533,2093005};


// Populate buffer w/one cycle of sine samples at amplitudes [minAmpl, maxAmpl]
void poplDMABuffer(uint16_t buffer[], float minAmpl, float maxAmpl) {
	double voltage_to_dac = 4095.0 / 3.3;
	double step = TAU / DMASIZE;

	for (int i=0; i<DMASIZE; ++i) {
		double X = sinf(step * i)*(maxAmpl - minAmpl) + minAmpl + maxAmpl;
		buffer[i] = (uint16_t)(X * 0.5 * voltage_to_dac);
	}
}

// Translate frequency into the corresponding DMA timer period
void setDMAFrequency(uint32_t _mHz) {
	double hz = (double)_mHz * 1e-3;
	uint32_t period = (uint32_t)round(10e6 / (hz * DMASIZE)) - 1;
	__HAL_TIM_SET_AUTORELOAD(&htim6, period);
	mHz = _mHz;
}

void logHALError(HAL_StatusTypeDef err) {
	static uint32_t errcnt = 0;
	static char errstr[128];

    switch (err)
    {
    case HAL_OK: sprintf(errstr, "%" PRIu32 " HAL_OK\n\r", errcnt); break;
    case HAL_ERROR: sprintf(errstr, "%" PRIu32 " HAL_ERROR\n\r", errcnt); break;
    case HAL_BUSY: sprintf(errstr, "%" PRIu32 " HAL_BUSY\n\r", errcnt); break;
    case HAL_TIMEOUT: sprintf(errstr, "%" PRIu32 " HAL_TIMEOUT\n\r", errcnt); break;
    default: sprintf(errstr, "%" PRIu32 " Unknown HAL ERROR: %" PRIu32 "\n\r", errcnt, (uint32_t)err);
    }
	HAL_UART_Transmit(&huart2, (uint8_t*)errstr, (uint16_t)strlen(errstr), 1000);
	errcnt += 1;
}

void logI2CError(uint32_t errcode) {
	static uint32_t i2cerrcnt = 0;
	static char i2cerrstr[128];

	switch (errcode)
	{
	case HAL_I2C_ERROR_NONE: sprintf(i2cerrstr, "%" PRIu32 " I2C: No Error \n\r", i2cerrcnt); break;
	case HAL_I2C_ERROR_BERR: sprintf(i2cerrstr, "%" PRIu32 " I2C: BERR Error \n\r", i2cerrcnt); break;
	case HAL_I2C_ERROR_ARLO: sprintf(i2cerrstr, "%" PRIu32 " I2C: ARLO Error \n\r", i2cerrcnt); break;
	case HAL_I2C_ERROR_AF: sprintf(i2cerrstr, "%" PRIu32 " I2C: ACKF Error \n\r", i2cerrcnt); break;
	case HAL_I2C_ERROR_OVR: sprintf(i2cerrstr, "%" PRIu32 " I2C: OVR Error \n\r", i2cerrcnt); break;
	case HAL_I2C_ERROR_DMA: sprintf(i2cerrstr, "%" PRIu32 " I2C: DMA Transfer Error \n\r", i2cerrcnt); break;
	case HAL_I2C_ERROR_TIMEOUT: sprintf(i2cerrstr, "%" PRIu32 " I2C: Timeout Error \n\r", i2cerrcnt); break;
	case HAL_I2C_ERROR_SIZE: sprintf(i2cerrstr, "%" PRIu32 " I2C: Size Management Error \n\r", i2cerrcnt); break;
	case HAL_I2C_ERROR_DMA_PARAM: sprintf(i2cerrstr, "%" PRIu32 " I2C: DMA Parameter Error \n\r", i2cerrcnt); break;
    default: sprintf(i2cerrstr, "%" PRIu32 " Unknown I2C ERROR: %" PRIu32 "\n\r", i2cerrcnt, errcode);
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)i2cerrstr, (uint16_t)strlen(i2cerrstr), 1000);
	i2cerrcnt += 1;
}

void uart_printf(const char* fmt, ...) {
    static char sbuffer[128];

    va_list args;
    va_start(args, fmt);
    vsnprintf(sbuffer, 128, fmt, args);
    va_end (args);

	HAL_UART_Transmit(&huart2, (const uint8_t*)sbuffer, (uint16_t)strlen(sbuffer), HAL_MAX_DELAY);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *p_hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
	HAL_StatusTypeDef err;
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
		uart_printf("HAL_I2C_AddrCallback: I2C_DIRECTION_TRANSMIT : %u \n\r", AddrMatchCode);
		err = HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, i2cbuffer, 4, I2C_FIRST_AND_LAST_FRAME);
		if (err) {
			logHALError(err);
			logI2CError(hi2c1.ErrorCode);
		}
	}
	else {
		uart_printf("HAL_I2C_AddrCallback: I2C_DIRECTION_RECEIVE : (%u) : [", AddrMatchCode);
		for (int i=0, j=24; i < 4; ++i, j -= 8) {
			i2cbuffer[i] = (uint8_t)(0xFF & (mHz >> j));
			if (i < 3) uart_printf("%u, ", i2cbuffer[i]);
			else uart_printf("%u] : %u \n\r", i2cbuffer[i], mHz);
		}

		err = HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, i2cbuffer, sizeof(uint32_t), I2C_FIRST_AND_LAST_FRAME);
		if (err) {
			logHALError(err);
			logI2CError(hi2c1.ErrorCode);
		}
	}
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef err = HAL_I2C_EnableListen_IT (&hi2c1);
	if (err) {
		logHALError(err);
		logI2CError(hi2c1.ErrorCode);
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	uint32_t _mHz = 0;
	uart_printf("HAL_I2C_SlaveRxCpltCallback : [");
	for (int i=0; i < sizeof(uint32_t); ++i) {
		_mHz = (_mHz << 8) | i2cbuffer[i];
		if (i < 3) uart_printf("%u, ", i2cbuffer[i]);
		else uart_printf("%u] : %lu \n\r", i2cbuffer[i], _mHz);
	}

	// Silently drops frequencies outside [0, 2.93K] Hz
	if (0 < _mHz && _mHz <= 2093005) setDMAFrequency(_mHz);
}


void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	uart_printf("HAL_I2C_SlaveTxCpltCallback \n\r");
}

//uint32_t getSineIndex(uint32_t msec, float omega, float minAmpl, float maxAmpl) {
//    float X = cosf(omega * (float)msec * 1e-3);
//    X = X*(maxAmpl - minAmpl) + maxAmpl + minAmpl;
//    return (uint32_t)(X * 0.5 * voltage_to_dac);
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  poplDMABuffer(dmabuffer, 0.1, 2.8);
  if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
	  Error_Handler();

  setDMAFrequency(1022);
  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)&dmabuffer[0], DMASIZE, DAC_ALIGN_12B_R) != HAL_OK)
	  Error_Handler();

  if (HAL_I2C_EnableListen_IT (&hi2c1) != HAL_OK)
	  Error_Handler();

//  uint32_t mHz = 24500;
//  float omega = (float)mHz * kTAU;
//  char sbuffer[128];
//  uint8_t tstbyte[1];

//  uint32_t tentry = HAL_GetTick();
//  uint32_t tlastreport = tentry;
//  uint32_t nsamples = 0;
//  uint32_t ii = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
//    HAL_StatusTypeDef err;
//  	memset(i2cdata, 0, 4);
//
//	if ( (err = HAL_I2C_Slave_Receive(&hi2c1, i2cdata, 4, 2500)) ) {
//		if (hi2c1.ErrorCode != HAL_I2C_ERROR_TIMEOUT) logI2CError(hi2c1.ErrorCode);
//		continue;
//	}
//
//	int32_t mHz = 0;
//	for (int i=0; i<4; ++i)
//		mHz |= (i2cdata[i] << ((3-i)*8));
//
//    // Silently drops frequencies outside [0, 2.93K]
//	sprintf(sbuffer, "mHz: %" PRIu32 " \n\r", mHz);
//	HAL_UART_Transmit(&huart2, (uint8_t*)sbuffer, (uint16_t)strlen(sbuffer), HAL_MAX_DELAY);
//	if (mHz >= 0 && mHz <= 2093005) setDMAFrequency(mHz);


////	  uint32_t msec = HAL_GetTick();
////	  uint32_t dacidx = getSineIndex(msec, omega, 0.0, 3.3);
////	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacidx);
//
//	  nsamples += 1;
//	  if (HAL_GetTick() - tlastreport > 3e3) {
//		  float hz = (float)scale[ii] * 1e-3;
//		  uint32_t period = (uint32_t)round(10e6 / (hz * DMASIZE)) - 1;
//
//		  __HAL_TIM_SET_AUTORELOAD(&htim6, period);
//
//		  float elapsed = (float)(HAL_GetTick() - tentry) * 1e-3;
//          float blkelapsed = (HAL_GetTick() - tlastreport) * 1e-3;
//          sprintf(sbuffer, "%.1fs, %.2fK loops/sec : %.2fHz (%u)\n\r", elapsed, (float)nsamples * 1e-3 / blkelapsed, hz, (uint16_t)period);
//		  HAL_UART_Transmit(&huart2, (uint8_t*)sbuffer, (uint16_t)strlen(sbuffer), 1000);
//          tlastreport = HAL_GetTick();
//          nsamples = 0;
//
//		  ii = (ii+1) % 133;
//	  }
//
////	  dacidx = dacidx == 0 ? 2048 : 0;
////	  uint16_t cntr = __HAL_TIM_GET_COUNTER(&htim6);
////	  sprintf(sbuffer, "Hello %u \n\r", cntr);
////	  HAL_UART_Transmit(&huart2, (uint8_t*)sbuffer, (uint16_t)strlen(sbuffer), 1000);
////	  HAL_Delay(500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30A0A7FB;
  hi2c1.Init.OwnAddress1 = 110;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 17 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 15;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
