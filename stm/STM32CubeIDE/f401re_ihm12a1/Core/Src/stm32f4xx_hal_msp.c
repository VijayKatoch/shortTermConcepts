/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32f4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
 
/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void BSP_MotorControl_FlagInterruptHandler(void);
extern void ButtonHandler(void);
/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
                        
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                        /**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration    
    PA0-WKUP     ------> TIM2_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }
  else if(htim->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration    
    PB4     ------> TIM3_CH1
    PB5     ------> TIM3_CH2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */
/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief PWM MSP Initialization
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if ((htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_A)&&
      (htim_pwm->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_A))
  {
    /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_A_CLCK_ENABLE();

    /* GPIO configuration */
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_A_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_A;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_A_PORT, &GPIO_InitStruct);
  }
  else if ((htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_B)&&
           (htim_pwm->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_B))
  {
    /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_B_CLCK_ENABLE();

    /* GPIO configuration */
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_B_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_B;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_B_PORT, &GPIO_InitStruct);
   }
  else if ((htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF)&&
           (htim_pwm->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_REF))
  {
    /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF_CLCK_ENABLE();

    /* GPIO configuration */
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_REF_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM_REF;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_REF_PORT, &GPIO_InitStruct);
   }
}

/**
  * @brief PWM MSP De-Initialization
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
   GPIO_InitTypeDef  GPIO_InitStruct;

 if ((htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_A)&&
       (htim_pwm->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_A))
  {
    /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_A_CLCK_DISABLE();

    /* GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PWM_A_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_A_PIN);

    // Reconfigure PWMA pin as output pull down
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_A_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_A_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PWM_A_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_A_PIN, GPIO_PIN_RESET);

  }
  else if ((htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_B)&&
           (htim_pwm->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_B))
  {
    /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_B_CLCK_DISABLE();

    /* GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PWM_B_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_B_PIN);

    // Reconfigure PWMB pin  as output pull down
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PWM_B_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PWM_B_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BSP_MOTOR_CONTROL_BOARD_PWM_B_PORT, BSP_MOTOR_CONTROL_BOARD_PWM_B_PIN, GPIO_PIN_RESET);
  }
  else if ((htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF)&&
           (htim_pwm->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM_REF))
  {
    /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM_REF_CLCK_DISABLE();

    /* GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_REF_PORT, BSP_MOTOR_CONTROL_BOARD_REF_PIN);
  }
}


/**
  * @brief External Line Callback
  * @param[in] GPIO_Pin pin number
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BSP_MOTOR_CONTROL_BOARD_EN_AND_FAULT_PIN)
  {
    BSP_MotorControl_FlagInterruptHandler();
  }
  if (GPIO_Pin == USER_BUTTON_PIN)
  {
    ButtonHandler();
  }
 }

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
