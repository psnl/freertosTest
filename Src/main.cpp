/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <L2Status.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "L2Config.h"
#include "LxHeater.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <arm_math.h>

/* USER CODE BEGIN Includes */
#define VEML_ADDRESS 0x10<<1
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId extraTaskHandle;
uint32_t extraTaskBuffer[ 128 ];
osStaticThreadDef_t extraTaskControlBlock;
osTimerId timerLedHandle;
osStaticTimerDef_t timerLedControlBlock;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
L2Status l2Status;
L2Config l2Config;
LxHeater lxHeater;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void ExtraWork(void const * argument);
void LedCallback(void const * argument);

extern "C" void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
char buffer[100];


#define NN_INPUTS 4
#define NN_OUTPUTS 7

//New synaptic weights after training:
//[[ -47.35089345  -24.63222218  -59.21599592  -25.56931986   76.77853135   31.12939791 -103.54337726]
// [ -13.98105245  -23.42661049  138.61228931  -24.48080183 -153.19354046    6.02166287   28.26860691]
// [ -56.34916047   -9.38341148  -41.86215374   -9.55725783  -16.88586002   28.49674502  125.64577683]
// [  41.06440983  -45.57299539  -62.3425506   -44.46980062   12.29234919  -35.99428284  -15.42447511]]

// 4 rows (inputs), 7 columns (outputs)
//const float32_t weights[NN_INPUTS][NN_OUTPUTS] = {
//		{-47.35089345,-24.63222218,-59.21599592,-25.56931986,76.77853135,31.12939791,-103.54337726},
//		{-13.98105245,-23.42661049,138.61228931,-24.48080183,-153.19354046,6.02166287,28.26860691},
//		{-56.34916047,-9.38341148,-41.86215374,-9.55725783,-16.88586002,28.49674502,125.64577683},
//		{41.06440983,-45.57299539,-62.3425506,-44.46980062,12.29234919,-35.99428284,-15.42447511}
//};

//New synaptic weights after training:
//[[-236.74453983  -32.67928928  -99.22864567  -33.57420597  169.18924829
//    43.1275596  -175.35602226]
// [ -76.13114675  -30.9896174   245.66804467  -32.03968971 -276.41118994
//     5.72043396   62.65210549]
// [-232.564032    -12.39999393  -46.6093097   -12.60142124  -22.29661383
//    49.33386959  223.67313328]
// [ 225.21390689  -60.17146292 -120.25086101  -59.08975195    0.37958931
//   -49.04452606  -39.73057214]]
const float32_t weights[NN_INPUTS][NN_OUTPUTS] = {
		{-236.74453983,-32.67928928,-99.22864567,-33.57420597,169.18924829,43.1275596,-175.35602226},
		{-76.13114675,-30.9896174,245.66804467,-32.03968971,-276.41118994,5.72043396,62.65210549},
		{-232.564032,-12.39999393,-46.6093097,-12.60142124,-22.29661383,49.33386959,223.67313328},
		{225.21390689,-60.17146292,-120.25086101,-59.08975195,0.37958931,-49.04452606,-39.73057214}
};


int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of timerLed */
  osTimerStaticDef(timerLed, LedCallback, &timerLedControlBlock);
  timerLedHandle = osTimerCreate(osTimer(timerLed), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of extraTask */
  osThreadStaticDef(extraTask, ExtraWork, osPriorityNormal, 0, 128, extraTaskBuffer, &extraTaskControlBlock);
  extraTaskHandle = osThreadCreate(osThread(extraTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1200;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_Pin */
  GPIO_InitStruct.Pin = TOUCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
enum Color
{   // Sequence of neural network
	RED   	= 4,
	GREEN	= 2,
	BLUE	= 6,
	YELLOW	= 1,
	ORANGE	= 3,
	BROWN	= 5,
	UNKNOWN = 0
};

struct TOutputData
{
	uint8_t data[NN_OUTPUTS];
};

Color Neural(float32_t red, float32_t green, float32_t blue, float32_t white, TOutputData* ptOutputData)
{
	/*
	New synaptic weights after training:
	[[ -47.35089345  -24.63222218  -59.21599592  -25.56931986   76.77853135   31.12939791 -103.54337726]
	 [ -13.98105245  -23.42661049  138.61228931  -24.48080183 -153.19354046    6.02166287   28.26860691]
	 [ -56.34916047   -9.38341148  -41.86215374   -9.55725783  -16.88586002   28.49674502  125.64577683]
	 [  41.06440983  -45.57299539  -62.3425506   -44.46980062   12.29234919  -35.99428284  -15.42447511]]
	*/
	float32_t product[NN_OUTPUTS];
	for (int column=0; column<NN_OUTPUTS; column++) {
		product[column] = red * weights[0][column] + green * weights[1][column] + blue * weights[2][column] + white * weights[3][column];
	}

    //def __sigmoid(self, x):
    //    return 1 / (1 + exp(-x))
	float32_t sigmoidOut[NN_OUTPUTS];
	for (int column=0; column<NN_OUTPUTS; column++) {
		sigmoidOut[column] = (1 / (1 + exp(-product[column])));
	}

	int index = 0;

	for (int column=0; column<NN_OUTPUTS; column++) {
		ptOutputData->data[column] = (uint8_t)(sigmoidOut[column]*100);
	}

	for (int column=0; column<NN_OUTPUTS; column++) {
	  if (sigmoidOut[column]>sigmoidOut[index])
	  {
		  index = column;
	  }
	}

	//sprintf(buffer, "re%f,gr%f,bl%f,or%f,ye%f,br%f,un%f\r\n", sigmoidOut[RED], sigmoidOut[GREEN], sigmoidOut[BLUE],
	//		sigmoidOut[ORANGE], sigmoidOut[YELLOW], sigmoidOut[BROWN], sigmoidOut[UNKNOWN]);
	//sprintf(buffer, "Index:%d\r\n", index);
	//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
    return (Color)index;
}

Color GetColor()
{
	uint16_t colorRedRaw = 0;
	uint16_t colorGreenRaw = 0;
	uint16_t colorBlueRaw = 0;
	uint16_t colorWhiteRaw = 0;
	uint16_t colorRed = 0;
	uint16_t colorGreen = 0;
	uint16_t colorBlue = 0;

	HAL_I2C_Mem_Read(&hi2c2, VEML_ADDRESS, 0x8, 1, (uint8_t*)&colorRedRaw, 2, 1000);
	HAL_I2C_Mem_Read(&hi2c2, VEML_ADDRESS, 0x9, 1, (uint8_t*)&colorGreenRaw, 2, 1000);
	HAL_I2C_Mem_Read(&hi2c2, VEML_ADDRESS, 0xa, 1, (uint8_t*)&colorBlueRaw, 2, 1000);
	HAL_I2C_Mem_Read(&hi2c2, VEML_ADDRESS, 0xb, 1, (uint8_t*)&colorWhiteRaw, 2, 1000);

	colorRed = colorRedRaw*200/colorWhiteRaw;
	colorGreen = colorGreenRaw*300/colorWhiteRaw;
	colorBlue = colorBlueRaw*600/colorWhiteRaw;

//	sprintf(buffer, "%u,%u,%u,%u\r\n", colorRedRaw, colorGreenRaw, colorBlueRaw, colorWhiteRaw);
//	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
	Color index = UNKNOWN;
	TOutputData outputData;
	index = Neural( ((float32_t)colorRedRaw)/10000, ((float32_t)colorGreenRaw)/10000, ((float32_t)colorBlueRaw)/10000, ((float32_t)colorWhiteRaw)/10000, &outputData );

	sprintf(buffer, "%d %d %d %d %d %d %d %d\r\n", index, outputData.data[0], outputData.data[1], outputData.data[2],
			outputData.data[3], outputData.data[4], outputData.data[5], outputData.data[6]);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 1000);
	return index;
/*
	if (colorWhiteRaw > 6000)
	{
		if ((colorGreen-50) > colorRed)
		{
			return YELLOW;
		}
		else
		{
			return ORANGE;
		}
	}
	else if ((colorWhiteRaw > 500) && (colorWhiteRaw < 2000))
	{
		return BROWN;
	}
	else if (colorWhiteRaw > 2000)
	{
		if ((colorRed > colorGreen) && (colorRed > colorBlue))
		{
			return RED;
		}
		else if ((colorGreen > colorBlue))
		{
			return GREEN;
		}
		else if ((colorBlue > colorGreen))
		{
			return BLUE;
		}
	}

	return UNKNOWN;
	*/
}

void SelectColor(Color color, bool all = true)
{
	switch (color)
	{
	case RED:
		if (all) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 100);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 62);
		}
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 100);
		break;
	case GREEN:
		if (all) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 25);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 42);
		}
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 90);
		break;
	case BLUE:
		if (all) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 100);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 62);
		}
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 40);
		break;
	case YELLOW:
		if (all) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 25);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 42);
		}
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 40);
		break;
	case ORANGE:
		if (all) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 25);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 42);
		}
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 68);
		break;
	case BROWN:
		if (all) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 100);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 62);
		}
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 68);
		break;
	case UNKNOWN:
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 52);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 68);
		break;
	}
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  l2Status.Start();

  //lxHeater.Start((IHeaterConfig*)&l2Config, (IHeaterStatus*)&l2Status);
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  uint16_t dataI2c = 0x00;
  HAL_I2C_Mem_Write(&hi2c2, VEML_ADDRESS, 0x00, 1, (uint8_t*)&dataI2c, 2, 1000);

  sprintf(buffer, "M&M color sorter started\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, 7, 1000);
  Color color = UNKNOWN;

  for(;;)
  {
	  if (HAL_GPIO_ReadPin(TOUCH_GPIO_Port, TOUCH_Pin) == GPIO_PIN_SET)
	  {
		  // Top open
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 25);
		  osDelay(1000);
		  // Top close
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 80);
		  osDelay(1000);
		  // Midle open
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 25);
		  osDelay(1000);
		  // Middle meaure
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 50);
		  // Set unkown
		  SelectColor(Color::UNKNOWN);
		  osDelay(500);
		  color = GetColor();
		  SelectColor(color, false);
		  osDelay(500);
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 100);
		  osDelay(1000);
		  SelectColor(color);
		  osDelay(1000);

		  /*
		//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 80); // 2
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 100);
	  osDelay(1000);
		//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 40); // 1
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 25);
	  osDelay(1000);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 100);
	  osDelay(1000);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 25);
	  osDelay(1000);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 100);
	  osDelay(1000);
	//  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 100);
	//  osDelay(1000);
	//  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 25);
	//  osDelay(1000);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 100);
	  osDelay(1000);
		//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 40); // 1
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 25);
	  osDelay(1000);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 100);
	  osDelay(1000);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 25);
	  osDelay(1000);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 25);
	  osDelay(1000);
	  */

	  }
	  else
	  {
//		  osDelay(1000);
//		  color = GetColor();
//		  SelectColor(color);
	  }
  }
  /* USER CODE END 5 */ 
}



/* ExtraWork function */
void ExtraWork(void const * argument)
{
  /* USER CODE BEGIN ExtraWork */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ExtraWork */
}

/* LedCallback function */
void LedCallback(void const * argument)
{
  /* USER CODE BEGIN LedCallback */
  
  /* USER CODE END LedCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
