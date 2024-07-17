/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Emm_V5.h"
#include "stdio.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "PID.h"
#include "centre.h"
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

/* USER CODE BEGIN PV */
Cascade_PID my_pid;

float M3508_speed_set ;
float m3508_angle_set = 30; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void delay_ms(uint16_t cnt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	MOTOR motor[3];
	motor_measure_t motor_chassis[7];
	static unsigned long long sign = 0 ;
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
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	//启动CAN通信
	
	//启动PWM 控制舵机
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1 | TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	
	can_filter_init();
	//初始化轮子
	cheel_init();
	//motor_speed_control(5000 ,10);
		int p1 = 0 ;
		int p2 = 0 ;
		int p3 = 0 ;
			//__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);		//前进方向舵机 占空比85是初始平放位置 120竖直位置
			
//					__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,85);
//		Emm_V5_Pos_Control(2,0,2500,125,8400,1,0); //8400高度足够 到达最大值 7400能够放置到高木上  4000足够放置到低木桩上
//			HAL_Delay(1500);
//Emm_V5_Pos_Control(1,0,2500,125,20000,1,0);
//	HAL_Delay(1500);
//Emm_V5_Pos_Control(2,0,1250,125,1000,1,0); //8400高度足够 到达最大值 7400能够放置到高木上  4000足够放置到低木桩上
//	HAL_Delay(1750);
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);
//	Emm_V5_Pos_Control(2,0,1250,125,8400,1,0); //8400高度足够 到达最大值 7400能够放置到高木上  4000足够放置到低木桩上

			HAL_TIM_Base_Start_IT(&htim6);
//		while(sign <=7000)
//		{
//			p1 = PID_calc(&motor[0].pid_inner,motor[0].speed_rpm,450); 
//			p2 = PID_calc(&motor[1].pid_inner,motor[1].speed_rpm,450); 
//			CAN_cmd_chassis(p1,-p2,0,0);
//		}
//		while(sign >7000 && sign<8000)
//		{
//			p1 = PID_calc(&motor[0].pid_inner,motor[0].speed_rpm,0); 
//			p2 = PID_calc(&motor[1].pid_inner,motor[1].speed_rpm,0); 
//			CAN_cmd_chassis(p1,-p2,0,0);
//		}
//		CAN_cmd_chassis(0,0,0,0);
//		while(sign >0 && sign <7300)
//		{
//			p3 = PID_calc(&motor[2].pid_inner,motor[2].speed_rpm,8000); 
//			CAN_cmd_chassis(0,0,p3,0);
//					printf("%d \n",motor[2].speed_rpm );

//		}
//		while(sign >7300 && sign <10000)
//		{

//		}
//		CAN_cmd_chassis(0,0,0,0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while ( 1 )
  {
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
		
		HAL_Delay(1);
		  M3508_speed_set = PID_calc(&motor[2].pid_outer,motor[2].angle,m3508_angle_set);
			p3 = PID_calc(&motor[2].pid_cloud,motor[2].speed_rpm,M3508_speed_set); 
			CAN_cmd_chassis(0,0,p3,0);
				//	printf("%d \n",motor[2].speed_rpm );

		
			

		
		
		
//		CAN_cmd_chassis(0,0,0,0);
//		for(int i = 0 ;i<100 ;i+=10 )
//		{
//			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,i);
//			HAL_Delay(100 );
//		}
	
	//motor_position_control(190,1);//角度为减速机角度，变成输出角度要除以19
		printf("%d \n",motor[2].speed_rpm );
		//motor_position_control(10,1);
	//motor_position_control(5,1);
		
		
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//一毫秒进入一次中断函数
	if(htim->Instance == TIM6)
	{
		sign++;
	}
}

int fputc(int ch, FILE* f)
{
    uint8_t temp[1] = {ch};
    HAL_UART_Transmit(&huart3,temp,1,2);
    return ch;
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
