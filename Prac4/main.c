/* USER CODE BEGIN Header */
/**
*******************************************************
Info:		STM32 DMA and PWM with HAL
Author:		Amaan Vally
*******************************************************
In this practical you will to use PWM using DMA on the STM32 using the HAL.
We also set up an interrupt to switch the waveform between various LUTs.

  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//TO DO:
//TASK 2
//Assign values to NS, TIM2CLK and F_SIGNAL
#define NS 256	//num values in look up tables
#define TIM2CLK  48000000//the clock frequency of the STM32 discovery board is 48MHz
#define F_SIGNAL  10// a signal of 10Hz was chosen as higher frequencies would output less clear

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */

//TO DO:
//TASK 1

//Create global variables for LUTs
uint8_t mode=0;//used for switching between different output waveforms in task 5

//below are all of the look up tables for a triangular wave, sawtooth wave and sine wave respectively

uint32_t sin_LUT[NS] = {12, 524, 537, 549, 562, 574, 587, 599, 612, 624, 636, 648, 661, 673, 684,
		696, 708, 720, 731, 742, 753, 765, 775, 786, 797, 807, 817, 827, 837, 847,
		856, 865, 874, 883, 892, 900, 908, 916, 923, 931, 938, 945, 951, 958, 964,
		969, 975, 980, 985, 990, 994, 998, 1002, 1005, 1008, 1011, 1014, 1016, 1018, 1020,
		1021, 1022, 1023, 1023, 1023, 1023, 1022, 1021, 1020, 1019, 1017, 1015, 1012, 1010, 1007,
		1003, 1000, 996, 992, 987, 983, 977, 972, 967, 961, 954, 948, 941, 934, 927,
		920, 912, 904, 896, 887, 879, 870, 861, 851, 842, 832, 822, 812, 802, 791,
		781, 770, 759, 748, 737, 725, 714, 702, 690, 679, 667, 655, 642, 630, 618,
		605, 593, 581, 568, 556, 543, 530, 518, 505, 493, 480, 467, 455, 442, 430,
		418, 405, 393, 381, 368, 356, 344, 333, 321, 309, 298, 286, 275, 264, 253,
		242, 232, 221, 211, 201, 191, 181, 172, 162, 153, 144, 136, 127, 119, 111,
		103, 96, 89, 82, 75, 69, 62, 56, 51, 46, 40, 36, 31, 27, 23,
		20, 16, 13, 11, 8, 6, 4, 3, 2, 1, 0, 0, 0, 0, 1,
		2, 3, 5, 7, 9, 12, 15, 18, 21, 25, 29, 33, 38, 43, 48,
		54, 59, 65, 72, 78, 85, 92, 100, 107, 115, 123, 131, 140, 149, 158,
		167, 176, 186, 196, 206, 216, 226, 237, 248, 258, 270, 281, 292, 303, 315,
		327, 339, 350, 362, 375, 387, 399, 411, 424, 436, 449, 461, 474, 486, 499,
		511};


uint32_t saw_LUT[NS] = {0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56,
		60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116,
		120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 173, 177,
		181, 185, 189, 193, 197, 201, 205, 209, 213, 217, 221, 225, 229, 233, 237,
		241, 245, 249, 253, 257, 261, 265, 269, 273, 277, 281, 285, 289, 293, 297,
		301, 305, 309, 313, 317, 321, 325, 329, 333, 337, 341, 345, 349, 353, 357,
		361, 365, 369, 373, 377, 381, 385, 389, 393, 397, 401, 405, 409, 413, 417,
		421, 425, 429, 433, 437, 441, 445, 449, 453, 457, 461, 465, 469, 473, 477,
		481, 485, 489, 493, 497, 501, 505, 509, 514, 518, 522, 526, 530, 534, 538,
		542, 546, 550, 554, 558, 562, 566, 570, 574, 578, 582, 586, 590, 594, 598,
		602, 606, 610, 614, 618, 622, 626, 630, 634, 638, 642, 646, 650, 654, 658,
		662, 666, 670, 674, 678, 682, 686, 690, 694, 698, 702, 706, 710, 714, 718,
		722, 726, 730, 734, 738, 742, 746, 750, 754, 758, 762, 766, 770, 774, 778,
		782, 786, 790, 794, 798, 802, 806, 810, 814, 818, 822, 826, 830, 834, 838,
		842, 846, 850, 855, 859, 863, 867, 871, 875, 879, 883, 887, 891, 895, 899,
		903, 907, 911, 915, 919, 923, 927, 931, 935, 939, 943, 947, 951, 955, 959,
		963, 967, 971, 975, 979, 983, 987, 991, 995, 999, 1003, 1007, 1011, 1015, 1019,
		0};

uint32_t triangle_LUT[NS] = {0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112,
		120, 128, 136, 144, 152, 160, 168, 177, 185, 193, 201, 209, 217, 225, 233,
		241, 249, 257, 265, 273, 281, 289, 297, 305, 313, 321, 329, 337, 345, 353,
		361, 369, 377, 385, 393, 401, 409, 417, 425, 433, 441, 449, 457, 465, 473,
		481, 489, 497, 505, 514, 522, 530, 538, 546, 554, 562, 570, 578, 586, 594,
		602, 610, 618, 626, 634, 642, 650, 658, 666, 674, 682, 690, 698, 706, 714,
		722, 730, 738, 746, 754, 762, 770, 778, 786, 794, 802, 810, 818, 826, 834,
		842, 850, 859, 867, 875, 883, 891, 899, 907, 915, 923, 931, 939, 947, 955,
		963, 971, 979, 987, 995, 1003, 1011, 1019, 1019, 1011, 1003, 995, 987, 979, 971,
		963, 955, 947, 939, 931, 923, 915, 907, 899, 891, 883, 875, 867, 859, 850,
		842, 834, 826, 818, 810, 802, 794, 786, 778, 770, 762, 754, 746, 738, 730,
		722, 714, 706, 698, 690, 682, 674, 666, 658, 650, 642, 634, 626, 618, 610,
		602, 594, 586, 578, 570, 562, 554, 546, 538, 530, 522, 514, 505, 497, 489,
		481, 473, 465, 457, 449, 441, 433, 425, 417, 409, 401, 393, 385, 377, 369,
		361, 353, 345, 337, 329, 321, 313, 305, 297, 289, 281, 273, 265, 257, 249,
		241, 233, 225, 217, 209, 201, 193, 185, 177, 168, 160, 152, 144, 136, 128,
		120, 112, 104, 96, 88, 80, 72, 64, 56, 48, 40, 32, 24, 16, 8,
		0};


//TO DO:
//TASK 3
//Calculate TIM2_Ticks
uint32_t TIM2_Ticks = TIM2CLK/NS/F_SIGNAL;// formula was generated from simplifying formulas from tutorial 4


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //TO DO:
  //TASK 4
  //Start TIM3 in PWM mode on channel 1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //start tim3 in PWM mode on channel 1


  //Start TIM2 in Output Compare (OC) mode on channel 1.
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
  //Start the DMA in interrupt (IT) mode.

  //uint32_t DestAddress = (uint32_t) &(TIM3->CCR1);

  EXTI0_1_IRQHandler();// by running the IRQ handler like this, the board does not output anything until the button is pressed.

  //Start the DMA transfer
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


	  //No need to do anything in the main loop for this practical


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1; //To make the frequency what we want it to be
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	//TO DO:
	//TASK 5
	//Disable DMA transfer, start DMA in IT mode with new source and re enable transfer
	//Remember to debounce using HAL_GetTick()
	if(HAL_GetTick()>=10)//used for de-bouncing

	{
		HAL_DMA_Abort_IT(&hdma_tim2_ch1);// this was this function was preffered over the HAL_DMA_disable function.
		//__HAL_TIM_DISABLE_DMA(&htim2,TIM_DMA_CC1);
		//HAL_DMA_Start_IT(&hdma_tim2_ch1, sin_LUT, DestAddress, NS);

		uint32_t DestAddress = (uint32_t) &(TIM3->CCR1);
		if(mode==1)// the mode select is used for switching between different look up tables.
		{
			HAL_DMA_Start_IT(&hdma_tim2_ch1, saw_LUT, DestAddress, NS);// start DMA in interrupt mode
			__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
			mode=2;
		}
		else if(mode==2)
		{
			HAL_DMA_Start_IT(&hdma_tim2_ch1, sin_LUT, DestAddress, NS);// start DMA in interrupt mode
			__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
			mode=3;
		}
		else if(mode==3)
		{
			HAL_DMA_Start_IT(&hdma_tim2_ch1, triangle_LUT, DestAddress, NS);// start DMA in interrupt mode
			__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
			mode=0;
		}
		else if(mode==0)
		{
			mode=1;
		}
	}
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); // Clear interrupt flags
}
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
