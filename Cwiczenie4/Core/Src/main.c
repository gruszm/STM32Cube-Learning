/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	MINUTES_SECONDS,
	HOURS_MINUTES
} ClockModeType;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define JOY_PORT GPIOE
#define JOY_PORT_PUSH GPIO_PIN_15
#define JOY_PUSH_STATE ((~(GPIOE->IDR) & JOY_PORT_PUSH) ? 1 : 0)

#define SEGMENT_A_PIN  GPIO_PIN_0
#define SEGMENT_B_PIN  GPIO_PIN_1
#define SEGMENT_C_PIN  GPIO_PIN_2
#define SEGMENT_D_PIN  GPIO_PIN_3
#define SEGMENT_E_PIN  GPIO_PIN_4
#define SEGMENT_F_PIN  GPIO_PIN_5
#define SEGMENT_G_PIN  GPIO_PIN_6
#define SEGMENT_DP_PIN GPIO_PIN_9

#define SEGMENT_GPIO_PORT GPIOG
#define SEGMENT_MASK SEGMENT_A_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN | SEGMENT_E_PIN | SEGMENT_F_PIN | SEGMENT_G_PIN | SEGMENT_DP_PIN

#define DIGIT_1_PIN GPIO_PIN_2
#define DIGIT_2_PIN GPIO_PIN_3
#define DIGIT_3_PIN GPIO_PIN_4
#define DIGIT_4_PIN GPIO_PIN_5

#define DIGIT_GPIO_PORT GPIOB
#define DIGIT_MASK DIG1_PIN | DIG2_PIN | DIG3_PIN | DIG4_PIN

#define FREQ 50U // 50 Hz
#define DELAY 20U // 20 ms
#define MAX_HOURS 24U
#define MAX_MINUTES_SECONDS 60U
#define MAX_MILISECONDS 1000U
#define RIGHT_DOT_BLINK_INTERVAL 2000U
#define NUMBER_OF_DIGITS 4U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

const uint8_t segments_au8[] = {
    SEGMENT_A_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN | SEGMENT_E_PIN | SEGMENT_F_PIN,                 // 0
    SEGMENT_B_PIN | SEGMENT_C_PIN,                                                                                 // 1
	SEGMENT_A_PIN | SEGMENT_E_PIN | SEGMENT_B_PIN | SEGMENT_D_PIN | SEGMENT_G_PIN,                                 // 2
	SEGMENT_A_PIN | SEGMENT_B_PIN | SEGMENT_G_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN,                                 // 3
	SEGMENT_F_PIN | SEGMENT_G_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN,                                                 // 4
	SEGMENT_A_PIN | SEGMENT_F_PIN | SEGMENT_G_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN,                                 // 5
    SEGMENT_A_PIN | SEGMENT_F_PIN | SEGMENT_G_PIN | SEGMENT_E_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN,                 // 6
	SEGMENT_A_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN,                                                                 // 7
    SEGMENT_A_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN | SEGMENT_E_PIN | SEGMENT_F_PIN | SEGMENT_G_PIN, // 8
	SEGMENT_A_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN | SEGMENT_F_PIN | SEGMENT_G_PIN                  // 9
};

uint16_t miliseconds_u16 = 0U;
uint16_t dotBlinkCounter_u16 = 0U;
uint8_t seconds_u8 = 0U;
uint8_t minutes_u8 = 0U;
uint8_t hours_u8 = 0U;
ClockModeType clockMode_e = MINUTES_SECONDS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);

  GPIO_InitTypeDef gpioInit = {0};
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /* GPIOG config */
  gpioInit.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9;
  gpioInit.Mode  = GPIO_MODE_OUTPUT_PP;
  gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
  gpioInit.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &gpioInit);

  /* GPIOB config */
  gpioInit.Pin   = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &gpioInit);

  /* GPIOE config */
  gpioInit.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_15;
  gpioInit.Mode  = GPIO_MODE_INPUT;
  gpioInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
  gpioInit.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &gpioInit);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000;
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
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  static uint8_t previousCycleJoyPush_u8 = 0U;
	  static uint8_t currentDigit_u8 = 0U;

	  miliseconds_u16++;

	  if (miliseconds_u16 >= MAX_MILISECONDS)
	  {
		  miliseconds_u16 -= MAX_MILISECONDS;
		  seconds_u8++;
	  }

	  if (seconds_u8 >= MAX_MINUTES_SECONDS)
	  {
		  seconds_u8 = 0U;
		  minutes_u8++;
	  }

	  if (minutes_u8 >= MAX_MINUTES_SECONDS)
	  {
		  minutes_u8 = 0U;
		  hours_u8++;
	  }

	  if (hours_u8 >= MAX_HOURS)
	  {
		  miliseconds_u16 = 0U;
		  seconds_u8 = 0U;
		  minutes_u8 = 0U;
		  hours_u8 = 0U;
	  }

	  if (JOY_PUSH_STATE)
	  {
		  if (0U == previousCycleJoyPush_u8)
		  {
			  if (MINUTES_SECONDS == clockMode_e)
			  {
				  clockMode_e = HOURS_MINUTES;
			  }
			  else
			  {
				  clockMode_e = MINUTES_SECONDS;

				  /* On switching to minutes:seconds, right dot should be switched off
				   * and the counter should be reset. */
				  dotBlinkCounter_u16 = 0U;
			  }

			  previousCycleJoyPush_u8 = 1U;
		  }
	  }
	  else
	  {
		  previousCycleJoyPush_u8 = 0U;
	  }

	  /* minutes:seconds */
	  if (MINUTES_SECONDS == clockMode_e)
	  {
		  // Digit 1
		  if (0U == currentDigit_u8)
		  {
			  uint8_t minutesDigitLeft_u8 = (uint8_t)((float)minutes_u8 / 10.0F);

			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_4_PIN, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments_au8[minutesDigitLeft_u8], GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_1_PIN, GPIO_PIN_SET);
		  }

		  // Digit 2
		  if (1U == currentDigit_u8)
		  {
			  uint8_t minutesDigitRight_u8 = minutes_u8 % 10U;

			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_1_PIN, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments_au8[minutesDigitRight_u8] | SEGMENT_DP_PIN, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_2_PIN, GPIO_PIN_SET);
		  }

		  // Digit 3
		  if (2U == currentDigit_u8)
		  {
			  uint8_t secondsDigitLeft_u8 = (uint8_t)((float)seconds_u8 / 10.0F);

			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_2_PIN, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments_au8[secondsDigitLeft_u8], GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_3_PIN, GPIO_PIN_SET);
		  }

		  // Digit 4
		  if (3U == currentDigit_u8)
		  {
			  uint8_t secondsDigitRight_u8 = seconds_u8 % 10U;

			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_3_PIN, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments_au8[secondsDigitRight_u8], GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_4_PIN, GPIO_PIN_SET);
		  }

	  }
	  /* hours:minutes */
	  else if (HOURS_MINUTES == clockMode_e)
	  {
		  // Digit 1
		  if (0U == currentDigit_u8)
		  {
			  uint8_t hoursDigitLeft_u8 = (uint8_t)((float)hours_u8 / 10.0F);

			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_4_PIN, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments_au8[hoursDigitLeft_u8], GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_1_PIN, GPIO_PIN_SET);
		  }

		  // Digit 2
		  if (1U == currentDigit_u8)
		  {
			  uint8_t hoursDigitRight_u8 = hours_u8 % 10U;

			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_1_PIN, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments_au8[hoursDigitRight_u8] | SEGMENT_DP_PIN, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_2_PIN, GPIO_PIN_SET);
		  }

		  // Digit 3
		  if (2U == currentDigit_u8)
		  {
			  uint8_t minutesDigitLeft_u8 = (uint8_t)((float)minutes_u8 / 10.0F);

			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_2_PIN, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments_au8[minutesDigitLeft_u8], GPIO_PIN_SET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_3_PIN, GPIO_PIN_SET);
		  }

		  // Digit 4
		  if (3U == currentDigit_u8)
		  {
			  uint8_t minutesDigitRight_u8 = minutes_u8 % 10U;

			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_3_PIN, GPIO_PIN_RESET);

			  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT,
					                              (dotBlinkCounter_u16 < (RIGHT_DOT_BLINK_INTERVAL / 2U)) ?
					                                    segments_au8[minutesDigitRight_u8] : segments_au8[minutesDigitRight_u8] | SEGMENT_DP_PIN,
							                       GPIO_PIN_SET);

			  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_4_PIN, GPIO_PIN_SET);

		  }
	  }

	  dotBlinkCounter_u16++;
	  if (dotBlinkCounter_u16 >= RIGHT_DOT_BLINK_INTERVAL)
	  {
		  dotBlinkCounter_u16 = 0U;
	  }

	  currentDigit_u8++;
	  if (currentDigit_u8 >= NUMBER_OF_DIGITS)
	  {
		  currentDigit_u8 = 0U;
	  }
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
