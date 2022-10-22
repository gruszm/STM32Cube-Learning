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

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define JOY_PORT GPIOE

#define JOY_PORT_RIGHT GPIO_PIN_0
#define JOY_PORT_LEFT GPIO_PIN_1
#define JOY_PORT_UP GPIO_PIN_3
#define JOY_PORT_DOWN GPIO_PIN_2
#define JOY_PORT_PUSH GPIO_PIN_15

#define JOY_LEFT_STATE ((~(GPIOE->IDR) & JOY_PORT_LEFT) ? 1 : 0)
#define JOY_RIGHT_STATE ((~(GPIOE->IDR) & JOY_PORT_RIGHT) ? 1 : 0)
#define JOY_UP_STATE ((~(GPIOE->IDR) & JOY_PORT_UP) ? 1 : 0)
#define JOY_DOWN_STATE ((~(GPIOE->IDR) & JOY_PORT_DOWN) ? 1 : 0)
#define JOY_PUSH_STATE ((~(GPIOE->IDR) & JOY_PORT_PUSH) ? 1 : 0)

#define LED0 GPIOD, GPIO_PIN_12
#define LED1 GPIOD, GPIO_PIN_13
#define LED2 GPIOB, GPIO_PIN_8

#define FREQ 50U // 50 Hz
#define DELAY 20U // 20 ms
#define NUMBER_OF_LEDS 3U
#define MAX_LED_LEVEL DELAY

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

uint8_t counter_u8 = 0U;
uint8_t ledLevels_au8[NUMBER_OF_LEDS] = {0U};
uint8_t currentLed_u8 = 0U;

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

  static uint8_t prevCycleLeftJoy_u8 = 0U;
  static uint8_t prevCycleRightJoy_u8 = 0U;

  if (DELAY == counter_u8)
  {
	  if (JOY_UP_STATE)
	  {
		  if (ledLevels_au8[currentLed_u8] < MAX_LED_LEVEL)
		  {
			  ledLevels_au8[currentLed_u8] += 1U;
		  }
		  else
		  {
			  ledLevels_au8[currentLed_u8] = MAX_LED_LEVEL;
		  }

		  prevCycleLeftJoy_u8 = 0U;
		  prevCycleRightJoy_u8 = 0U;
	  }
	  else if (JOY_DOWN_STATE)
	  {
		  if (ledLevels_au8[currentLed_u8] > 0U)
		  {
			  ledLevels_au8[currentLed_u8] -= 1U;
		  }
		  else
		  {
			  ledLevels_au8[currentLed_u8] = 0U;
		  }

		  prevCycleLeftJoy_u8 = 0U;
		  prevCycleRightJoy_u8 = 0U;
	  }
	  else if (JOY_LEFT_STATE)
	  {
		  if (0U == prevCycleLeftJoy_u8)
		  {
			  if (currentLed_u8 > 0U)
			  {
				  currentLed_u8--;
			  }
			  else
			  {
				  currentLed_u8 = NUMBER_OF_LEDS - 1U;
			  }
		  }

		  prevCycleLeftJoy_u8 = 1U;
		  prevCycleRightJoy_u8 = 0U;
	  }
	  else if (JOY_RIGHT_STATE)
	  {
		  if (0U == prevCycleRightJoy_u8)
		  {
			  if (currentLed_u8 < (NUMBER_OF_LEDS - 1U))
			  {
			      currentLed_u8++;
			  }
			  else
			  {
				  currentLed_u8 = 0U;
			  }
		  }

		  prevCycleLeftJoy_u8 = 0U;
		  prevCycleRightJoy_u8 = 1U;
	  }
	  else if (JOY_PUSH_STATE)
	  {
		  ledLevels_au8[0U] = 0U;
		  ledLevels_au8[1U] = 0U;
		  ledLevels_au8[2U] = 0U;

		  prevCycleLeftJoy_u8 = 0U;
		  prevCycleRightJoy_u8 = 0U;
	  }
	  else
	  {
		  prevCycleLeftJoy_u8 = 0U;
		  prevCycleRightJoy_u8 = 0U;
	  }

	  counter_u8 = 0U;
  }
  else
  {
	  counter_u8++;
  }

  if (counter_u8 < ledLevels_au8[0U])
  {
	  HAL_GPIO_WritePin(LED0, GPIO_PIN_SET);
  }
  else
  {
	  HAL_GPIO_WritePin(LED0, GPIO_PIN_RESET);
  }

  if (counter_u8 < ledLevels_au8[1U])
  {
	  HAL_GPIO_WritePin(LED1, GPIO_PIN_SET);
  }
  else
  {
	  HAL_GPIO_WritePin(LED1, GPIO_PIN_RESET);
  }

  if (counter_u8 < ledLevels_au8[2U])
  {
	  HAL_GPIO_WritePin(LED2, GPIO_PIN_SET);
  }
  else
  {
	  HAL_GPIO_WritePin(LED2, GPIO_PIN_RESET);
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
