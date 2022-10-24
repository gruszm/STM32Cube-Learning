/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

typedef enum
{
	MINUTES_SECONDS,
	HOURS_MINUTES
} ClockModeType;

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define JOY_PORT GPIOE
#define JOY_PORT_PUSH GPIO_PIN_15
#define JOY_PUSH_STATE ((~(GPIOE->IDR) & JOY_PORT_PUSH) ? 1 : 0)

#define SEG_A 0U
#define SEG_B 1U
#define SEG_C 2U
#define SEG_D 3U
#define SEG_E 4U
#define SEG_F 5U
#define SEG_G 6U
#define SEG_DP 9U

#define DIG_1 GPIOB, GPIO_PIN_2
#define DIG_2 GPIOB, GPIO_PIN_3
#define DIG_3 GPIOB, GPIO_PIN_4
#define DIG_4 GPIOB, GPIO_PIN_5

#define FREQ 50U // 50 Hz
#define DELAY 20U // 20 ms
#define MAX_HOURS 24U
#define MAX_MINUTES_SECONDS 60U
#define MAX_MILISECONDS 1000U

#define RESET_SEG 8U

#define RIGHT_DOT_BLINK_INTERVAL 2000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

const uint8_t segments_au8[] = {
    1 << SEG_A | 1 << SEG_B | 1 << SEG_C | 1 << SEG_D | 1 << SEG_E | 1 << SEG_F, // 0
    1 << SEG_B | 1 << SEG_C, // 1
	1 << SEG_A | 1 << SEG_E | 1 << SEG_B | 1 << SEG_D | 1 << SEG_G, // 2
	1 << SEG_A | 1 << SEG_B | 1 << SEG_G | 1 << SEG_C | 1 << SEG_D, // 3
	1 << SEG_F | 1 << SEG_G | 1 << SEG_B | 1 << SEG_C, // 4
	1 << SEG_A | 1 << SEG_F | 1 << SEG_G | 1 << SEG_C | 1 << SEG_D, // 5
    1 << SEG_A | 1 << SEG_F | 1 << SEG_G | 1 << SEG_E | 1 << SEG_C | 1 << SEG_D, // 6
	1 << SEG_A | 1 << SEG_B | 1 << SEG_C, // 7
    1 << SEG_A | 1 << SEG_B | 1 << SEG_C | 1 << SEG_D | 1 << SEG_E | 1 << SEG_F | 1 << SEG_G, // 8
	1 << SEG_A | 1 << SEG_B | 1 << SEG_C | 1 << SEG_D | 1 << SEG_F | 1 << SEG_G // 9
};

uint16_t miliseconds_u16 = 0U;
uint16_t dotBlinkCounter_u16 = 0U;
uint8_t seconds_u8 = 0U;
uint8_t minutes_u8 = 0U;
uint8_t hours_u8 = 0U;
ClockModeType clockMode_e = MINUTES_SECONDS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  static uint8_t previousCycleJoyPush_u8 = 0U;

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
			  HAL_GPIO_WritePin(DIG_4, GPIO_PIN_SET);
		      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(DIG_4, GPIO_PIN_RESET);

			  dotBlinkCounter_u16 = 0U;
		  }

		  previousCycleJoyPush_u8 = 1U;
	  }
  }
  else
  {
	  previousCycleJoyPush_u8 = 0U;
  }

  // Reset all digits
  HAL_GPIO_WritePin(DIG_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOG, segments_au8[RESET_SEG], GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIG_1, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(DIG_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOG, segments_au8[RESET_SEG], GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIG_2, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(DIG_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOG, segments_au8[RESET_SEG], GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIG_3, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(DIG_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOG, segments_au8[RESET_SEG], GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIG_4, GPIO_PIN_RESET);

  /* minutes:seconds */
  if (MINUTES_SECONDS == clockMode_e)
  {
	  // Digit 1
	  uint8_t minutesDigit1_u8 = (uint8_t)((float)minutes_u8 / 10.0F);

	  HAL_GPIO_WritePin(DIG_1, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOG, segments_au8[minutesDigit1_u8], GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DIG_1, GPIO_PIN_RESET);

	  // Digit 2
	  uint8_t minutesDigit2_u8 = minutes_u8 % 10U;

	  HAL_GPIO_WritePin(DIG_2, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOG, segments_au8[minutesDigit2_u8], GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DIG_2, GPIO_PIN_RESET);

	  // Digit 3
	  uint8_t secondsDigit1_u8 = (uint8_t)((float)seconds_u8 / 10.0F);

	  HAL_GPIO_WritePin(DIG_3, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOG, segments_au8[secondsDigit1_u8], GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DIG_3, GPIO_PIN_RESET);

	  // Digit 4
	  uint8_t secondsDigit2_u8 = seconds_u8 % 10U;

	  HAL_GPIO_WritePin(DIG_4, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOG, segments_au8[secondsDigit2_u8], GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DIG_4, GPIO_PIN_RESET);
  }
  /* hours:minutes */
  else if (HOURS_MINUTES == clockMode_e)
  {
	  // Digit 1
	  uint8_t hoursDigit1_u8 = (uint8_t)((float)hours_u8 / 10.0F);

	  HAL_GPIO_WritePin(DIG_1, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOG, segments_au8[hoursDigit1_u8], GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DIG_1, GPIO_PIN_RESET);

	  // Digit 2
	  uint8_t hoursDigit2_u8 = hours_u8 % 10U;

	  HAL_GPIO_WritePin(DIG_2, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOG, segments_au8[hoursDigit2_u8], GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DIG_2, GPIO_PIN_RESET);

	  // Digit 3
	  uint8_t minutesDigit1_u8 = (uint8_t)((float)minutes_u8 / 10.0F);

	  HAL_GPIO_WritePin(DIG_3, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOG, segments_au8[minutesDigit1_u8], GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DIG_3, GPIO_PIN_RESET);

	  // Digit 4
	  uint8_t minutesDigit2_u8 = minutes_u8 % 10U;

	  HAL_GPIO_WritePin(DIG_4, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOG, segments_au8[minutesDigit2_u8], GPIO_PIN_SET);

	  /* Right dot shall blink */
	  if (dotBlinkCounter_u16 < (RIGHT_DOT_BLINK_INTERVAL / 2U))
	  {
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
	  }

	  HAL_GPIO_WritePin(DIG_4, GPIO_PIN_RESET);
  }

  dotBlinkCounter_u16++;

  if (dotBlinkCounter_u16 > RIGHT_DOT_BLINK_INTERVAL)
  {
	  dotBlinkCounter_u16 = 0U;
  }

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
