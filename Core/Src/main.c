/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void Init_SpeedMeasure();
static void Calculate_Speed();
static void CtrlRegister_Handler(uint32_t);

extern void HAL_TIM_PeriodElapsedCallback_Modbus(TIM_HandleTypeDef*);
extern void HAL_UART_RxCpltCallback_modbus(UART_HandleTypeDef*);
extern void HAL_UART_TxCpltCallback_modbus(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t tim10_elapsed_cnt;     //The counter counts overflows during the measurement  process
volatile uint32_t tim10_reminder;        //Ticks amount remained after the measurement stopped
volatile bool f_calc;                    //True if the measurement process finished and result should be calculated
volatile bool f_error;                   //True if error occurred during measurement process
volatile bool f_start;                   //True if the measurement process started - first sensor triggered

uint16_t object_speed;          //Speed in meters per second
uint32_t object_time_us;        //measured time in us
uint16_t measure_cnt;           //measurement counter

//Modbus holding and input registers array
uint16_t *Modbus_Data_Array[] = {&object_speed, (uint16_t*)&object_time_us, (uint16_t*)(&object_time_us) + 1, &measure_cnt};

//Modbus control register
uint8_t Modbus_Ctrl_Reg;
#define MEASURE_FLAG      0x01 //1 - if initiated the measurement process
#define INT_EDGE          0x02 //1-rising edge, 0-falling edge

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    //Initialize all used variables
    tim10_elapsed_cnt = 0;
    tim10_reminder = 0;
    f_calc = false;
    f_error =  false;
    f_start = false;
    object_speed = 0;
    object_time_us = 0;
    measure_cnt = 0;
    Modbus_Ctrl_Reg = 0;
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
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_UART4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
    __disable_irq();

    HAL_GPIO_LockPin(OSC_En_GPIO_Port, OSC_En_Pin); //LOCK this pin settings because they are important - this pin enables HSE generator

    SET_BIT(Modbus_Ctrl_Reg, INT_EDGE); //default edge for the EXTI0/EXTI1 interrupt is rising

    //Disable EXTI0/1 IRQs until measurement process wasn't initiated
    HAL_NVIC_DisableIRQ(Gate_In_EXTI_IRQn);
    HAL_NVIC_DisableIRQ(Gate_Out_EXTI_IRQn);

    //Setup Modbus RTU module and enable it
    MT_PORT_SetTimerModule(&htim3);
    MT_PORT_SetUartModule(&huart4); //use uart1 for debug purposes
    eMBErrorCode eStatus;
    eStatus = eMBInit(MB_RTU, MODBUS_SLAVE_ADDRESS, 0, huart4.Init.BaudRate, MB_PAR_NONE);
    eStatus = eMBEnable();
    if (eStatus != MB_ENOERR)
    {
        // Error handling
    }
    __enable_irq();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      eMBPoll(); //Modbus polling function

      if (f_calc == true)
      {
          f_calc = false;
          Calculate_Speed();
      }

      if (f_error == true)
      {
          f_error = false;
          object_speed = FALSE_START_SPEED_VAL;
      }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 25;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 2599;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RXEN_L_GPIO_Port, RXEN_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UART_MOD_H_GPIO_Port, UART_MOD_H_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TE_485_L_GPIO_Port, TE_485_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HDPLX_H_GPIO_Port, HDPLX_H_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DXEN_H_GPIO_Port, DXEN_H_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OSC_En_GPIO_Port, OSC_En_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LedOut_GPIO_Port, LedOut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RXEN_L_Pin */
  GPIO_InitStruct.Pin = RXEN_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RXEN_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART_MOD_H_Pin */
  GPIO_InitStruct.Pin = UART_MOD_H_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UART_MOD_H_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_485_L_Pin */
  GPIO_InitStruct.Pin = TE_485_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TE_485_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HDPLX_H_Pin */
  GPIO_InitStruct.Pin = HDPLX_H_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HDPLX_H_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DXEN_H_Pin */
  GPIO_InitStruct.Pin = DXEN_H_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DXEN_H_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OSC_En_Pin */
  GPIO_InitStruct.Pin = OSC_En_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OSC_En_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Gate_Out_Pin Gate_In_Pin */
  GPIO_InitStruct.Pin = Gate_Out_Pin|Gate_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LedOut_Pin */
  GPIO_InitStruct.Pin = LedOut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LedOut_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    __disable_irq();
    if (GPIO_Pin == Gate_In_Pin)
    {
       if (READ_BIT(Modbus_Ctrl_Reg, MEASURE_FLAG) && f_start == false)
       {
           HAL_NVIC_DisableIRQ(Gate_In_EXTI_IRQn); //Disable EXTI0 interrupt to exclude debounce

           tim10_elapsed_cnt = 0;
           tim10_reminder = 0;

           f_start = true;
           f_error =  false;

           //Clear all pending EXTI1 interrupts and enable interrupt
           while (HAL_NVIC_GetPendingIRQ(Gate_Out_EXTI_IRQn))
           {
               __HAL_GPIO_EXTI_CLEAR_IT(Gate_Out_Pin);
               HAL_NVIC_ClearPendingIRQ(Gate_Out_EXTI_IRQn);
           }
           HAL_NVIC_EnableIRQ(Gate_Out_EXTI_IRQn);

           __HAL_TIM_SET_COUNTER(&htim10, 0); //Erase timer counter and start it
           //HAL_TIM_Base_Start_IT(&htim10);

           HAL_GPIO_WritePin(LedOut_GPIO_Port, LedOut_Pin, GPIO_PIN_SET); //RED LED turned on
       }
    }
    else if (GPIO_Pin == Gate_Out_Pin)
    {
        if (READ_BIT(Modbus_Ctrl_Reg, MEASURE_FLAG) && f_start == true)
        {
            HAL_NVIC_DisableIRQ(Gate_Out_EXTI_IRQn); //Disable EXTI1 interrupt to exclude debounce

            HAL_TIM_Base_Stop_IT(&htim10);  //Stop timer counter and save the counter value
            tim10_reminder = __HAL_TIM_GET_COUNTER(&htim10);

            CLEAR_BIT(Modbus_Ctrl_Reg, MEASURE_FLAG);
            f_start = false;
            f_calc = true;

            HAL_GPIO_WritePin(LedOut_GPIO_Port, LedOut_Pin, GPIO_PIN_RESET); //RED LED turned off
        }
    }
    __enable_irq();
}

/**
  * @brief UART RX complete callback
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_RxCpltCallback_modbus(huart);
}

/**
  * @brief UART TX complete callback
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_TxCpltCallback_modbus(huart);
}

/**
  * @brief Timer tim10 elapsed period callback
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PeriodElapsedCallback_Modbus(htim);

    __disable_irq();
    if (htim->Instance == TIM10)
    {
        if(f_start == true)
        {
            //check for the false start case
            if (++tim10_elapsed_cnt == FALSE_START_TIMEOUT)
            {
                HAL_TIM_Base_Stop_IT(&htim10);  //Don't need timer any more -  stop it
                HAL_NVIC_DisableIRQ(Gate_Out_EXTI_IRQn); //Disable EXTI1 interrupt - don't need it any more

                f_error =  true;
                CLEAR_BIT(Modbus_Ctrl_Reg, MEASURE_FLAG);
                f_start = false;
                f_calc = false;

                HAL_GPIO_WritePin(LedOut_GPIO_Port, LedOut_Pin, GPIO_PIN_RESET);  //RED LED turned off
            }
            else
            {
                HAL_GPIO_TogglePin(LedOut_GPIO_Port, LedOut_Pin);
            }
        }
    }
    __enable_irq();
}

/**
  * @brief Initialize the speed measurement process
  * @retval None
  */
static void Init_SpeedMeasure()
{
    __disable_irq();
    //Clear all pending EXTI0 interrupts and enable interrupt
    while (HAL_NVIC_GetPendingIRQ(Gate_In_EXTI_IRQn))
    {
        __HAL_GPIO_EXTI_CLEAR_IT(Gate_In_Pin);
        HAL_NVIC_ClearPendingIRQ(Gate_In_EXTI_IRQn);
    }
    HAL_NVIC_EnableIRQ(Gate_In_EXTI_IRQn);

    HAL_TIM_Base_Start_IT(&htim10);
    __enable_irq();
}

/**
  * @brief Calculate the object speed (m/s) and transmit it through the UART interface
  * @retval None
  */
static void Calculate_Speed()
{
    uint32_t time_us;

    /*time_us =  ONE_TICK_US * (tim10_elapsed_cnt * TIM10_Period + tim10_reminder) ;

    object_speed = ((REEL_LENGTH_CM * 1000000) / time_us) / 100; //speed in meters per second*/

    if (tim10_elapsed_cnt == 0 && tim10_reminder == 0) //case if two sensors were triggered simultaneously
    {
        object_speed = FALSE_START_SPEED_VAL - 1; //to identify this situation from others
    }
    else
    {
        time_us = tim10_elapsed_cnt * TIM10_Period + tim10_reminder;

        object_speed = (uint16_t)( ( MAX_CM_PER_SEC / time_us ) / 10); //speed in tenth of centimeters per second

        object_time_us = time_us * ONE_TICK_US; //time in us

        ++measure_cnt;
    }
}

/**
  * @brief
  * @retval eMBErrorCode
  */
eMBErrorCode eMBRegInputCB(uint8_t *pucRegBuffer, uint16_t usAddress, uint16_t usNRegs)
{
    eMBErrorCode eStatus = MB_ENOERR;
    uint32_t iRegIndex;

    if ((usAddress >= REG_INPUT_START) &&
        (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = (uint32_t)(usAddress - REG_INPUT_START);
        while(usNRegs > 0)
        {
            *pucRegBuffer++ = *Modbus_Data_Array[iRegIndex] >> 8;
            *pucRegBuffer++ = *Modbus_Data_Array[iRegIndex] & 0xFF;
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/**
  * @brief
  * @retval eMBErrorCode
  */
eMBErrorCode eMBRegHoldingCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    uint32_t iRegIndex;

    if ( (usAddress >= REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS) )
    {
        iRegIndex = (uint32_t)(usAddress - REG_HOLDING_START);
        switch (eMode)
        {
            /* Pass current register values to the protocol stack. */
            case MB_REG_READ:
                while (usNRegs > 0)
                {
                    *pucRegBuffer++ = *Modbus_Data_Array[iRegIndex] >> 8;
                    *pucRegBuffer++ = *Modbus_Data_Array[iRegIndex] & 0xFF;
                    iRegIndex++;
                    usNRegs--;
                }
                break;

                /* Update current register values with new values from the
                 * protocol stack. */
            case MB_REG_WRITE:
                while (usNRegs > 0)
                {
                    *Modbus_Data_Array[iRegIndex] = *pucRegBuffer++ << 8;
                    *Modbus_Data_Array[iRegIndex] |= *pucRegBuffer++;
                    iRegIndex++;
                    usNRegs--;
                }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/**
  * @brief
  * @retval eMBErrorCode
  */
eMBErrorCode eMBRegCoilsCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNCoils, eMBRegisterMode eMode)
{
     eMBErrorCode eStatus = MB_ENOERR;
     uint32_t iBitIndex;

     if ( (usAddress >= REG_COILS_START) && (usAddress + usNCoils <= REG_COILS_START + REG_COILS_NREGS) )
     {
         iBitIndex = (uint32_t)(usAddress - REG_COILS_START);

         switch (eMode)
         {
             case MB_REG_READ:
             {
                 while (usNCoils > 0)
                 {
                     UCHAR ucResult = xMBUtilGetBits(&Modbus_Ctrl_Reg, iBitIndex, 1);
                     xMBUtilSetBits(pucRegBuffer, iBitIndex - (usAddress - REG_COILS_START), 1, ucResult);
                     iBitIndex++;
                     usNCoils--;
                 }
                 break;
             }
             case MB_REG_WRITE:
             {
                 while (usNCoils > 0)
                 {
                     UCHAR ucResult = xMBUtilGetBits(pucRegBuffer, iBitIndex - (usAddress - REG_COILS_START), 1);
                     xMBUtilSetBits(&Modbus_Ctrl_Reg, iBitIndex, 1, ucResult );

                     CtrlRegister_Handler(iBitIndex); //Control register Bits handler

                     iBitIndex++;
                     usNCoils--;
                 }
                 break;
             }
         }
     }
     else
     {
         eStatus = MB_ENOREG;
     }

     return eStatus;
}

/**
  * @brief
  * @retval eMBErrorCode
  */
eMBErrorCode eMBRegDiscreteCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNDiscrete)
{

    return MB_ENOERR;
}

/**
  * @brief Control register's bits handler
  * @retval None
  */
static void CtrlRegister_Handler(uint32_t bit)
{
    switch(bit)
    {
        case 0: //Bit 0 of Control Register
        {
            if (READ_BIT(Modbus_Ctrl_Reg, MEASURE_FLAG))
            {
                Init_SpeedMeasure();
            }
            break;
        }

        case 1: //Bit 1 of Control Register
        {
            GPIO_InitTypeDef GPIO_InitStruct = {0};

            if (READ_BIT(Modbus_Ctrl_Reg, INT_EDGE))
            {
                GPIO_InitStruct.Pin = Gate_Out_Pin|Gate_In_Pin;
                GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
                GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            }
            else
            {
                GPIO_InitStruct.Pin = Gate_Out_Pin|Gate_In_Pin;
                GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
                GPIO_InitStruct.Pull = GPIO_PULLUP;
            }
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            break;
        }

        default:
            break;
    }
}

/**
  * @brief Enable LTC2870 RX mode
  * @retval
  */
void LTC2870_RX485_En_Rx()
{
    HAL_GPIO_WritePin(RXEN_L_GPIO_Port, RXEN_L_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DXEN_H_GPIO_Port, DXEN_H_Pin, GPIO_PIN_RESET);
}

/**
  * @brief Enable LTC2870 TX mode
  * @retval
  */
void LTC2870_RX485_En_Tx()
{
    HAL_GPIO_WritePin(RXEN_L_GPIO_Port, RXEN_L_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DXEN_H_GPIO_Port, DXEN_H_Pin, GPIO_PIN_SET);
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
