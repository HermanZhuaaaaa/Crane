/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

 __IO bool rxFrameFlag = false;
 __IO uint8_t rxCmd[FIFO_SIZE] = {0};
 __IO uint8_t rxCount = 0;
 uint8_t rxBuffer[200] = {0};
 
/* USER CODE END 0 */

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* UART7 init function */
void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}
/* UART8 init function */
void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspInit 0 */

  /* USER CODE END UART7_MspInit 0 */
    /* UART7 clock enable */
    __HAL_RCC_UART7_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART7 GPIO Configuration
    PE8     ------> UART7_TX
    PE7     ------> UART7_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* UART7 interrupt Init */
    HAL_NVIC_SetPriority(UART7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
  /* USER CODE BEGIN UART7_MspInit 1 */

  /* USER CODE END UART7_MspInit 1 */
  }
  else if(uartHandle->Instance==UART8)
  {
  /* USER CODE BEGIN UART8_MspInit 0 */

  /* USER CODE END UART8_MspInit 0 */
    /* UART8 clock enable */
    __HAL_RCC_UART8_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART8 GPIO Configuration
    PE1     ------> UART8_TX
    PE0     ------> UART8_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* UART8 interrupt Init */
    HAL_NVIC_SetPriority(UART8_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART8_IRQn);
  /* USER CODE BEGIN UART8_MspInit 1 */

  /* USER CODE END UART8_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD9     ------> USART3_RX
    PD8     ------> USART3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspDeInit 0 */

  /* USER CODE END UART7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART7_CLK_DISABLE();

    /**UART7 GPIO Configuration
    PE8     ------> UART7_TX
    PE7     ------> UART7_RX
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_8|GPIO_PIN_7);

    /* UART7 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART7_IRQn);
  /* USER CODE BEGIN UART7_MspDeInit 1 */

  /* USER CODE END UART7_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART8)
  {
  /* USER CODE BEGIN UART8_MspDeInit 0 */

  /* USER CODE END UART8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART8_CLK_DISABLE();

    /**UART8 GPIO Configuration
    PE1     ------> UART8_TX
    PE0     ------> UART8_RX
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_1|GPIO_PIN_0);

    /* UART8 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART8_IRQn);
  /* USER CODE BEGIN UART8_MspDeInit 1 */

  /* USER CODE END UART8_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD9     ------> USART3_RX
    PD8     ------> USART3_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_9|GPIO_PIN_8);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void usart_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
	__IO uint8_t i = 0;
	
	for(i=0; i < len; i++) { usart_SendByte(cmd[i]); }
}


void usart_SendByte(uint16_t data)
{
	uint8_t low;
	low = (uint8_t)data;
	HAL_UART_Transmit(&huart3,&low,sizeof(low),100);

}
/* USER CODE END 1 */
