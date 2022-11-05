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
#include "kamami_l496_mems.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

typedef enum
{
	ACC_X,
	ACC_Y,
	ACC_Z
} AccelerometerStateType;

/* USER CODE END TD */

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
#define NUMBER_OF_DIGITS 4U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

const static uint8_t segments_au8[] = {
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

static AccelerometerStateType accState_e = ACC_X;

static uint8_t digits_au8[NUMBER_OF_DIGITS] = {0U};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

static void UpdateDigits(void)
{
	struct mems_xyz_res acc;
	float correctedAccValues_f32[3U] = {0.0F};
    float specificAccValue_f32 = 0.0F;
    uint32_t digitValue_u32 = 0U;

	mems_acc_read_xyz(&acc);

	correctedAccValues_f32[0U] = fabsf((float)acc.x * 2.0F / MEMS_ACC_MAXVAL);
    correctedAccValues_f32[1U] = fabsf((float)acc.y * 2.0F / MEMS_ACC_MAXVAL);
	correctedAccValues_f32[2U] = fabsf((float)acc.z * 2.0F / MEMS_ACC_MAXVAL);

	switch (accState_e)
	{
	    case ACC_X:
	    	digits_au8[0U] = SEGMENT_F_PIN | SEGMENT_B_PIN | SEGMENT_G_PIN | SEGMENT_E_PIN | SEGMENT_C_PIN;
		    break;
	    case ACC_Y:
	    	digits_au8[0U] = SEGMENT_F_PIN | SEGMENT_G_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN;
	    	break;
	    case ACC_Z:
	    	digits_au8[0U] = segments_au8[2U];
	    default:
	    	break;
	}

	specificAccValue_f32 = correctedAccValues_f32[accState_e];

	/* Due to safety reasons, check if the acceleration is below 10.0F */
	if (specificAccValue_f32 < 10.0F)
	{
		digitValue_u32 = (uint32_t)specificAccValue_f32;
		digits_au8[1U] = segments_au8[digitValue_u32];
	}

	specificAccValue_f32 *= 10.0F;

	digitValue_u32 = ((uint32_t)specificAccValue_f32) % 10U;
	digits_au8[2U] = segments_au8[digitValue_u32];

	specificAccValue_f32 *= 10.0F;

	digitValue_u32 = ((uint32_t)specificAccValue_f32) % 10U;
	digits_au8[3U] = segments_au8[digitValue_u32];
}

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
  static uint8_t currentDigit_u8 = 0U;

  if (JOY_PUSH_STATE)
  {
	  if (0U == previousCycleJoyPush_u8)
	  {
		  switch (accState_e)
		  {
		  case ACC_X:
		  case ACC_Y:
			  accState_e++;
			  break;
		  case ACC_Z:
		  default:
			  accState_e = ACC_X;
			  break;
		  }

		  previousCycleJoyPush_u8 = 1U;
	  }
  }
  else
  {
	  previousCycleJoyPush_u8 = 0U;
  }

  // Digit 1
  if (0U == currentDigit_u8)
  {
	  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_4_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, digits_au8[0U], GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_1_PIN, GPIO_PIN_SET);
  }

  // Digit 2
  if (1U == currentDigit_u8)
  {
	  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_1_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, digits_au8[1U] | SEGMENT_DP_PIN, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_2_PIN, GPIO_PIN_SET);
  }

  // Digit 3
  if (2U == currentDigit_u8)
  {
	  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_2_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, digits_au8[2U], GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_3_PIN, GPIO_PIN_SET);
  }

  // Digit 4
  if (3U == currentDigit_u8)
  {
	  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_3_PIN, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, digits_au8[3U], GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_4_PIN, GPIO_PIN_SET);
  }

  UpdateDigits();

  currentDigit_u8++;
  if (currentDigit_u8 >= NUMBER_OF_DIGITS)
  {
	  currentDigit_u8 = 0U;
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
