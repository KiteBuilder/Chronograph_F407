/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mb.h"
#include "mbport.h"
#include "mt_port.h"
#include "mbutils.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum { false, true } bool;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define MakeUint16(lsb, msb)    ((uint16_t)((lsb) | ((uint16_t)(msb) << 8)))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RXEN_L_Pin GPIO_PIN_3
#define RXEN_L_GPIO_Port GPIOB
#define UART_MOD_H_Pin GPIO_PIN_7
#define UART_MOD_H_GPIO_Port GPIOD
#define TE_485_L_Pin GPIO_PIN_12
#define TE_485_L_GPIO_Port GPIOC
#define HDPLX_H_Pin GPIO_PIN_15
#define HDPLX_H_GPIO_Port GPIOA
#define DXEN_H_Pin GPIO_PIN_10
#define DXEN_H_GPIO_Port GPIOG
#define OSC_En_Pin GPIO_PIN_15
#define OSC_En_GPIO_Port GPIOC
#define Gate_Out_Pin GPIO_PIN_1
#define Gate_Out_GPIO_Port GPIOB
#define Gate_Out_EXTI_IRQn EXTI1_IRQn
#define Gate_In_Pin GPIO_PIN_0
#define Gate_In_GPIO_Port GPIOB
#define Gate_In_EXTI_IRQn EXTI0_IRQn
#define LedOut_Pin GPIO_PIN_10
#define LedOut_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define TIM10_Period             10000  //timer resolution -  ticks in one second (Hz)
#define TIM10_Prescaler          (HSE_VALUE / TIM10_Period)  //timer prescaler for given resolution

#define ONE_TICK_US              ( (uint32_t)1000000 / TIM10_Period )
#define MAX_CM_PER_SEC           ( ( (uint32_t)REEL_LENGTH_CM * TIM10_Period ) )

#define REEL_LENGTH_CM           100 // Distance between two sensors in centimeters
#define FALSE_START_TIMEOUT      5 // value in seconds to detect the false start
#define FALSE_START_SPEED_VAL    0xFFFF //Error value in the case of the false start

#define MODBUS_SLAVE_ADDRESS     0x02 //Modbus slave address(ID)

#define REG_COILS_START          1
#define REG_COILS_NREGS          2

#define REG_DISCRETE_COILS_START 1
#define REG_DISCRETE_NREGS       1

#define REG_INPUT_START          1
#define REG_INPUT_NREGS          4

#define REG_HOLDING_START        1
#define REG_HOLDING_NREGS        4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
