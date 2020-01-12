/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_STEPS (12)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BUTTON_CONTROL 0
#define CONSOLE_CONTROL 1

#define MAX_BUF_LEN 2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
static volatile uint16_t gLastError;
static volatile bool gButtonPressed = FALSE;
static volatile uint8_t gStep = MAX_STEPS;

Stspin240_250_Init_t gStspin240_250InitParams =
{
 {50, // 20000, Frequency of PWM of Input Bridge A in Hz up to 100000Hz
  50}, // 20000, Frequency of PWM of Input Bridge B in Hz up to 100000Hz
 50,                // 20000, Frequency of PWM used for Ref pin in Hz up to 100000Hz
 10,                   //50, Duty cycle of PWM used for Ref pin (from 0 to 100)
 TRUE                  // Dual Bridge configuration  (FALSE for mono, TRUE for dual brush dc)
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void MyFlagInterruptHandler(void);
void ButtonHandler(void);

/*custom system call for printf*/
int _write(int file, char *data, int len);

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
#if CONSOLE_CONTROL
	uint16_t motor_0_dir = 0;
	char buf[MAX_BUF_LEN];
#endif
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
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
#if CONSOLE_CONTROL
  printf("Enter 01 for FWD dir or 02 for REV dir of Motor 0. Anything else to stop v1\r\n");
#endif
  /* Configure KEY Button */
    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);


    /* Set Systick Interrupt to the highest priority to have HAL_Delay working*/
    /* under the user button handler */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0x0, 0x0);

    //----- Init of the Motor control library
    /* Set the Stspin240_250 library to use 1 device */
    BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN240, 1);
    /* When BSP_MotorControl_Init is called with NULL pointer,                  */
    /* the Stspin240_250 library parameters are set with the predefined values from file   */
    /* stspin240_250_target_config.h, otherwise the registers are set using the   */
    /* Stspin240_250_Init_t pointer structure                */
    /* Uncomment the call to BSP_MotorControl_Init below to initialize the      */
    /* device with the structure gStspin240_250InitParams declared in the the main.c file */
    /* and comment the subsequent call having the NULL pointer                   */
    //BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN240, &gStspin240_250InitParams);
    BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN240, NULL);

    /* Set dual bridge enabled as two motors are used*/
    BSP_MotorControl_SetDualFullBridgeConfig(1);

    /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
    BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

    /* Attach the function Error_Handler (defined below) to the error Handler*/
    BSP_MotorControl_AttachErrorHandler(Error_Handler);

    /* Set PWM Frequency of Ref to 15000 Hz */
    BSP_MotorControl_SetRefFreq(0,15000);

    /* Set PWM duty cycle of Ref to 60% */
    BSP_MotorControl_SetRefDc(0,60);

    /* Set PWM Frequency of bridge A inputs to 10000 Hz */
    BSP_MotorControl_SetBridgeInputPwmFreq(0,10000);

    /* Set PWM Frequency of bridge B inputs to 10000 Hz */
    /* On X-NUCLEO-IHM12A1 expansion board PWM_A and PWM_B shares the same */
    /* timer, so frequency must be the same */
    BSP_MotorControl_SetBridgeInputPwmFreq(1,10000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if BUTTON_CONTROL
	  /* Each time the user button is pressed, the step is increased by 1 */
	  if (gButtonPressed)
	  {
		gButtonPressed = FALSE;
		gStep++;
		if (gStep > MAX_STEPS)
		{
		  gStep = 0;
		}

		switch (gStep)
		{
		  case 0:
			/*********** Step 0  ************/
			/* Set speed of motor 0 to 100 % */
			BSP_MotorControl_SetMaxSpeed(0,100);
			/* start motor 0 to run forward*/
			/* if chip is in standby mode */
			/* it is automatically awakened */
			BSP_MotorControl_Run(0, FORWARD);
			break;

		   case 1:
			/*********** Step 1  ************/
			/* Set speed of motor 0 to 75 % */
			BSP_MotorControl_SetMaxSpeed(0,75);
			/* Set speed of motor 1 to 100 % */
			BSP_MotorControl_SetMaxSpeed(1,100);
			/* start motor 1 to run backward */
			BSP_MotorControl_Run(1, BACKWARD);
			break;

		  case 2:
			/*********** Step 2 ************/
			/* Set speed of motor 0 to 50 % */
			BSP_MotorControl_SetMaxSpeed(0,50);
		   /* Set speed of motor 1 to 75% */
			BSP_MotorControl_SetMaxSpeed(1,75);
			break;

		  case 3:
			/*********** Step 3 ************/
			/* Set speed of motor 0 to 25 % */
			BSP_MotorControl_SetMaxSpeed(0,25);
			/* Set speed of motor 1 to 50% */
			BSP_MotorControl_SetMaxSpeed(1,50);
			break;

		  case 4:
			/*********** Step 4 ************/
			/* Stop Motor 0 */
			BSP_MotorControl_HardStop(0);
			/* Set speed of motor 1 to 25% */
			BSP_MotorControl_SetMaxSpeed(1,25);
			break;
		   case 5:
			/*********** Step 5  ************/
			/* Set speed of motor 0 to 25 % */
			BSP_MotorControl_SetMaxSpeed(0,25);
			/* start motor 0 to run backward */
			BSP_MotorControl_Run(0, BACKWARD);
			/* Stop Motor 1 */
			BSP_MotorControl_HardStop(1);
			break;

		   case 6:
			/*********** Step 6  ************/
			/* Set speed of motor 0 to 50 % */
			BSP_MotorControl_SetMaxSpeed(0,50);
			/* Set speed of motor 1 to 25 % */
			BSP_MotorControl_SetMaxSpeed(1,25);
			/* start motor 1 to run backward */
			BSP_MotorControl_Run(1, FORWARD);
			break;

		  case 7:
			/*********** Step 7 ************/
			/* Set speed of motor 0 to 75 % */
			BSP_MotorControl_SetMaxSpeed(0,75);
			/* Set speed of motor 1 to 50 % */
			BSP_MotorControl_SetMaxSpeed(1,50);
			break;

		  case 8:
			/*********** Step 8 ************/
			/* Set speed of motor 0 to 100 % */
			BSP_MotorControl_SetMaxSpeed(0,100);
			/* Set speed of motor 1 to 75 % */
			BSP_MotorControl_SetMaxSpeed(1,75);
			break;

		  case 9:
			/*********** Step 9 ************/
			/* Set speed of motor 1 to 100 % */
			BSP_MotorControl_SetMaxSpeed(1,100);
			break;
		  case 10:
			/*********** Step 10 ************/
			/* Stop both motors and disable bridge */
			BSP_MotorControl_CmdHardHiZ(0);
			BSP_MotorControl_CmdHardHiZ(1);
			break;
		  case 11:
			/*********** Step 10 ************/
			/* Start both motors to go forward*/
			BSP_MotorControl_Run(0,FORWARD);
			BSP_MotorControl_Run(1,FORWARD);
			break;
		  case 12:
		  default:
			/*********** Step 10 ************/
			/* Stop both motors and put chip in standby mode */
			BSP_MotorControl_Reset(0);
			break;
		}
	  }
#endif

#if CONSOLE_CONTROL
	  while(HAL_UART_Receive(&hlpuart1, (uint8_t*)buf, 2, 0xFFFF) != HAL_OK);
	  motor_0_dir = atoi(buf);
	  if(1 == motor_0_dir)
	  {
		  printf("Selected: Motor 0 Fwd \r\n");
		  BSP_MotorControl_SetMaxSpeed(0,100);
		  BSP_MotorControl_Run(0, FORWARD);
	  }
	  else if(2 == motor_0_dir)
	  {
		  printf("Selected: Motor 0 Rev\r\n");
		  BSP_MotorControl_SetMaxSpeed(0,100);
		  BSP_MotorControl_Run(0, BACKWARD);
	  }
	  else
	  {
		  printf("Stop both motors and disable bridge \r\n");
		  /* Stop both motors and disable bridge */
		  BSP_MotorControl_CmdHardHiZ(0);
		  BSP_MotorControl_CmdHardHiZ(1);
	  }
#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
  /* Code to be customised */
  /************************/
  /* Get the state of bridge A */
  uint16_t bridgeState  = BSP_MotorControl_CmdGetStatus(0);

  if (bridgeState == 0)
  {
    if (BSP_MotorControl_GetDeviceState(0) != INACTIVE)
    {
      /* Bridges were disabled due to overcurrent or over temperature */
      /* When  motor was running */
        Error_Handler();
    }
  }
 }

/**
  * @brief  This function is executed when the Nucleo User button is pressed
  * @param  error number of the error
  * @retval None
  */
void ButtonHandler(void)
{
  gButtonPressed = TRUE;

  /* Let 300 ms before clearing the IT for key debouncing */
  HAL_Delay(300);
  __HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
  HAL_NVIC_ClearPendingIRQ(KEY_BUTTON_EXTI_IRQn);
}

int _write(int file, char *data, int len)
{
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
	{
	  errno = EBADF;
	  return -1;
	}

	// arbitrary timeout 1000
	HAL_StatusTypeDef status =
	  HAL_UART_Transmit(&hlpuart1, (uint8_t*)data, len, 1000);

	// return # of bytes written - as best we can tell
	return (status == HAL_OK ? len : 0);
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
	/* Infinite loop */
	  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
