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
#include "work.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
  ******************************************************************************
  * @author         : MashiroZZZ
  * @name           : HaoRan Zhu, Xi Cheng(Vision),KuanXi Huang(PUBG Player)
  * @copyright      : RoboMaster-Base
  ******************************************************************************
  * @attention
  * All rights reserved.
  ******************************************************************************
          _____                    _____          
         /\    \                  /\    \         
        /::\    \                /::\____\        
       /::::\    \              /::::|   |        
      /::::::\    \            /:::::|   |        
     /:::/\:::\    \          /::::::|   |        
    /:::/__\:::\    \        /:::/|::|   |        
   /::::\   \:::\    \      /:::/ |::|   |        
  /::::::\   \:::\    \    /:::/  |::|___|______  
 /:::/\:::\   \:::\____\  /:::/   |::::::::\    \ 
/:::/  \:::\   \:::|    |/:::/    |:::::::::\____\
\::/   |::::\  /:::|____|\::/    / ~~~~~/:::/    /
 \/____|:::::\/:::/    /  \/____/      /:::/    / 
       |:::::::::/    /               /:::/    /  
       |::|\::::/    /               /:::/    /   
       |::| \::/____/               /:::/    /    
       |::|  ~|                    /:::/    /     
       |::|   |                   /:::/    /      
       \::|   |                  /:::/    /       
        \:|   |                  \::/    /        
         \|___|                   \/____/         
                                                 
*/

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Motor_Initialization          1
#define Check_Begin_Position          1
#define Check_First_Forward           1
#define Check_First_PullDown          1
#define Check_First_Forward_2_Centre  1
#define Check_Gimbal_CW_First         1
#define Check_First_Centre_Position   1
#define Check_Gimbal_CW_Twice         1
#define Check_Centre_2_Destination    1
#define Check_Twice_PullDown          1
#define Check_Destination_2_Centre    1
#define Check_Gimbal_CW_Third         1
#define Check_Twice_Centre_Position   1
#define Check_Gimbal_CCW_First        1
#define Check_Centre_2_END            1
#define Check_Last_PullDOWN           1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//串级PID结构体
Cascade_PID gimbal_pid,chassis_pid_1,chassis_pid_2; //云台电机与轮子电机
//电机数据结构体 第三个可以作废 最后修正
MOTOR motor[3];
uint8_t avr_flag = 0;  	//平均速度计数标志位 最后要作废

uint8_t Check_Position_A = 0; //A舵机 行进标志位
uint8_t Check_Position_B = 0; //B舵机 行进标志位
uint8_t Check_Position_C = 0; //C底盘电机 行进标志位

uint8_t First_Time = 0;

//三个M3508电机的PID计算值
int p1 = 0 ;
int p2 = 0 ;
int p3 = 0 ;

//数组存储行进位置标记以及钩取砝码位置确认信息

uint8_t rxBuffer_A[2] = {0};	//接收A处舵机下，
uint8_t rxBuffer_B[2] = {0};	//接收B处舵机下，
uint8_t rxBuffer_C[2] = {0};	//接收导轨黑色停止胶带位置数组


//即将废弃
int16_t num[5][2] = {{18,18},{42,42},{60,60},{58,58},{129,129}}; // 计算数据 暂未实践检验
int32_t CurrentAngle[5][2];	//计算电角度

//标志位 
uint16_t Tx_Time = 0;
uint8_t first_flag = 0; //第一次上电标志位 用于记录首次上电电角度

uint8_t SteppingTime = 0;
uint32_t sign,gimbal;		//定时器中断计数值

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_CAN1_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART3_UART_Init();    
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	

	//启动PWM 控制舵机
	
	//A处舵机
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	//B处舵机
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	
	//启动CAN通信	
	can_filter_init();

	//初始化2个位置的M3508
	cheel_init();
	
	//开启接收中断
	HAL_UART_Receive_IT(&huart7,rxBuffer_A,sizeof(uint8_t));	//接受A处舵机有无货物数据
	
	HAL_UART_Receive_IT(&huart8,rxBuffer_B,sizeof(uint8_t));	//接受B处舵机有无货物数据

	HAL_UART_Receive_IT(&huart6,rxBuffer_C,sizeof(uint8_t));	//接收导轨黑色停止胶带位置


	//等待电机初始化
	HAL_Delay(3000);
	
#if Motor_Initialization 
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);	//A处舵机(即一号舵机)	120水平 100最大
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,90);	//B处舵机(即二号舵机) 90水平 120最大
	HAL_Delay(500);
	Work_Up(2,4);	
	HAL_Delay(50);
	Work_Up(4,4);
	HAL_Delay(3000);
  //舵机臂初始位置为最里端  (暂定)

#endif

		//本次代码

  
	//检测开始位置砝码
	//钩取开始位置砝码
#if Check_Begin_Position
  if(Check_Position_A == 0)
  {
      while(rxBuffer_B[0] != '1')   //什么也没收到 一直移动
    {
      Work_Move(3,1,30);
			SteppingTime ++;
      HAL_Delay(160);
    }
    if(rxBuffer_B[0] == '1')    //收到字符1 即为检测到砝码
    {

      Work_Straight(2);
      Check_Position_A = 1;
			if(SteppingTime >28)
			{
				SteppingTime = 0;
				Work_Move(3,0,200);
				HAL_Delay(1500);
			}
			else if (SteppingTime <10)
			{
				SteppingTime = 0;
				Work_Move(3,1,120);
				HAL_Delay(1500);
			}
			else 
			{
				SteppingTime = 0;
			}
      rxBuffer_B[0] = '3';    //修改为无意义标识
    }		
  }

#endif

//云台前进至钩子在中心物块正上方
#if Check_First_Forward
  if(Check_Position_C == 0)   
  {
    while(rxBuffer_C[0] != '1')   //什么也没收到 一直移动
      {
      HAL_Delay(1);
      p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,500);
      p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,-500);
      CAN_cmd_chassis(p1,p2,0,0);	
      }
    if(rxBuffer_C[0] == '1')    //收到字符1 即为检测到停止标识
    {
			Check_Position_C = 1;
        HAL_TIM_Base_Start_IT(&htim6);
      while(sign <1500)
      {
      HAL_Delay(1);
      p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,0);
      p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,-0);
      CAN_cmd_chassis(p1,p2,0,0);	
      }
    }		
  }

#endif
  //放下砝码
#if Check_First_PullDown
	if(Check_Position_A == 1)
	{

		HAL_TIM_Base_Stop_IT(&htim6);
		sign = 0;

		while(rxBuffer_B[0] != '2')
		{
		Work_Move(3,1,20);		
		HAL_Delay(100);	
		}
		if(rxBuffer_B[0] == '2')    //收到字符2 即为检测到木桩
		{
			HAL_Delay(50);
	  	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,90);
      HAL_Delay(300);
			Work_Move(3,1,15);
			HAL_Delay(1500);
			Work_PullDown(2,2);
			Check_Position_A = 2;
			HAL_Delay(1500);
			rxBuffer_B[0] = '3';    //修改为无意义标识
			HAL_TIM_Base_Start_IT(&htim6);
			}
	}
#endif
	//云台前进至云台位于中心物块正上方
#if Check_First_Forward_2_Centre
	if(Check_Position_C == 1)
	{
		
		while(sign<500)
		{
			HAL_Delay(1);
			p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,500);
			p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,-500);
			CAN_cmd_chassis(p1,p2,0,0);
				
			}
			rxBuffer_C[0] = 0;
			HAL_TIM_Base_Stop_IT(&htim6);
			sign = 0;
			while(rxBuffer_C[0] != '1')
			{
				HAL_Delay(1);
				p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,500);
				p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,-500);
				CAN_cmd_chassis(p1,p2,0,0);
				
			}
		if(rxBuffer_C[0] == '1')    //收到字符1 即为检测到停止标识
    {
			Check_Position_C = 2;
      HAL_TIM_Base_Start_IT(&htim6);
       
      while(sign <1500)
      {
      HAL_Delay(1);
      p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,0);
      p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,-0);
      CAN_cmd_chassis(p1,p2,0,0);	
      }
    }		
			
	}
#endif	 
	
	//云台顺时针移动60度
#if Check_Gimbal_CW_First
	if(Check_Position_C == 2)
	{
		HAL_TIM_Base_Stop_IT(&htim6);	//关闭定时器6 停止轮子行进计数
		sign = 0;	
		Check_Position_C = 3;

		Work_Move(3,1,130);
		HAL_Delay(50);
    Work_Move(1,0,480);
		Emm_V5_Pos_Control(5,1,500,175,1068,0,0);	//电机逆时针旋转  云台顺时针旋转  534为30度
		HAL_Delay(4000);
		
	}
#endif	


	// 由外向内检测顺时针30度方向砝码
#if Check_First_Centre_Position 
  if(Check_Position_A == 2)
  {
      while(rxBuffer_B[0] != '1')   //什么也没收到 一直移动
    {
      Work_Move(3,0,30);			
			SteppingTime ++;
      HAL_Delay(160);
    }
    if(rxBuffer_B[0] == '1')    //收到字符1 即为检测到砝码
    {
      Work_Move(3,1,30);
      HAL_Delay(1500);
      Work_Straight(2);
      Check_Position_A = 3;
			if(SteppingTime >28)
			{
				SteppingTime = 0;
				Work_Move(3,1,200);
				HAL_Delay(1500);
			}
			else if (SteppingTime <10)
			{
				SteppingTime = 0;
				Work_Move(3,0,150);
				HAL_Delay(1500);
			}
			else 
			{
				SteppingTime = 0;
			}
      rxBuffer_B[0] = '3';    //修改为无意义标识
    }		
  }

  if(Check_Position_B == 0)
  {
      while(rxBuffer_A[0] != '1')   //什么也没收到 一直移动
    {
      Work_Move(1,1,30);			
			SteppingTime ++;
      HAL_Delay(160);
    }
    if(rxBuffer_A[0] == '1')    //收到字符1 即为检测到砝码
    {
      Work_Move(1,0,30);
      HAL_Delay(1500);
      Work_Straight(1);
      Check_Position_B = 1;
			if(SteppingTime >28)
			{
				SteppingTime = 0;
				Work_Move(1,0,200);
				HAL_Delay(1500);
			}
			else if (SteppingTime <10)
			{
				SteppingTime = 0;
				Work_Move(1,1,150);
				HAL_Delay(1500);
			}
			else 
			{
				SteppingTime = 0;
			}
      rxBuffer_A[0] = '3';    //修改为无意义标识
    }		

  }
#endif

  //云台顺时针旋转至水平
#if Check_Gimbal_CW_Twice
	if(Check_Position_C == 3)
	{
		Check_Position_C = 4;
		Emm_V5_Pos_Control(5,1,500,175,534,0,0);	//电机逆时针旋转 云台顺时针旋转 534为60度 力臂30度
		HAL_Delay(4000);
		HAL_TIM_Base_Start_IT(&htim6);
	}
#endif

  //前进至末端
#if Check_Centre_2_Destination
   if(Check_Position_C == 4)   
  {
		while(sign<6027)
		{
			HAL_Delay(1);
			p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,500);
			p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,-500);
			CAN_cmd_chassis(p1,p2,0,0);
				
			}
			
      while(sign >=5927 &&sign <6927)
      {
      HAL_Delay(1);
      p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,0);
      p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,-0);
      CAN_cmd_chassis(p1,p2,0,0);	
      }
      rxBuffer_C[0] = 0;
			HAL_TIM_Base_Stop_IT(&htim6);
			sign = 0;
			Check_Position_C = 5;
    }		
#endif

    //检查木桩
#if Check_Twice_PullDown
    //清除检查过程中出现的问题
    rxBuffer_A[0] = '3';
    rxBuffer_B[0] = '3';

	 if(Check_Position_A == 3)
  {
      while(rxBuffer_B[0] != '2')   //什么也没收到 一直移动
    {
      Work_Move(3,1,25);			
      HAL_Delay(180);
    }
    if(rxBuffer_B[0] == '2')    //收到字符2 即为检测到木桩
    {
      //往后多走30mm
      Work_Move(3,1,30);
      HAL_Delay(1000);
	  	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,90);
      HAL_Delay(300);
			Work_Move(3,1,15);
			HAL_Delay(1500);
      Work_PullDown(2,3);
      Check_Position_A = 4;
      rxBuffer_B[0] = '3';
     }
  }
  	 if(Check_Position_B == 1)
  {
      while(rxBuffer_A[0] != '2')   //什么也没收到 一直移动
    {
      Work_Move(1,0,25);			
      HAL_Delay(180);
    }
    if(rxBuffer_A[0] == '2')    //收到字符2 即为检测到木桩
    {
      //往后多走30mm
      Work_Move(1,0,30);
      HAL_Delay(1000);
	  	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);
      HAL_Delay(300);
			Work_Move(1,0,15);
			HAL_Delay(1500);
      Work_PullDown(1,3);
      Check_Position_B = 2;
      rxBuffer_A[0] = '3';
     }
  }
#endif

  //返回中心
#if Check_Destination_2_Centre
  if(Check_Position_C == 5)   
  {
    while(rxBuffer_C[0] != '1')   //什么也没收到 一直移动
      {
      HAL_Delay(1);
      p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,-500);    //反转
      p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,500);
      CAN_cmd_chassis(p1,p2,0,0);	
      }
    if(rxBuffer_C[0] == '1')    //收到字符1 即为检测到停止标识
    {
			Check_Position_C = 6;
      HAL_TIM_Base_Start_IT(&htim6);
      while(sign <1500)
      {
      HAL_Delay(1);
      p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,0);
      p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,-0);
      CAN_cmd_chassis(p1,p2,0,0);	
      }
    }		
  }

#endif


  //顺时针旋转30度 
#if Check_Gimbal_CW_Third
  if(Check_Position_C == 6)
	{
    HAL_TIM_Base_Stop_IT(&htim6);
    sign = 0;
		Check_Position_C = 7;
		Emm_V5_Pos_Control(5,1,500,175,534,0,0);	//电机逆时针旋转  534为60度 力臂30度
		HAL_Delay(4000);

  }
#endif

    //检测逆时针30度上的砝码
#if Check_Twice_Centre_Position
  if(Check_Position_A == 4)
    {
        while(rxBuffer_B[0] != '1')   //什么也没收到 一直移动
      {
        Work_Move(3,0,30);			
        SteppingTime ++;
        HAL_Delay(160);
      }
      if(rxBuffer_B[0] == '1')    //收到字符1 即为检测到砝码
      {
         Work_Move(3,1,30);
         HAL_Delay(1500);
        Work_Straight(2);
        Check_Position_A = 5;

        if(SteppingTime >28)
        {
          SteppingTime = 0;
          Work_Move(3,1,200);
          HAL_Delay(1500);
        }

        else if (SteppingTime <10)
        {
          SteppingTime = 0;
          Work_Move(3,0,150);
          HAL_Delay(1500);
        }
        else 
        {
          SteppingTime = 0;
        }
        rxBuffer_B[0] = '3';    //修改为无意义标识
      }		
    }
  if(Check_Position_B == 2)
    {
        while(rxBuffer_A[0] != '1')   //什么也没收到 一直移动
      {
        Work_Move(1,1,30);			
        SteppingTime ++;
        HAL_Delay(160);
      }
      if(rxBuffer_A[0] == '1')    //收到字符1 即为检测到砝码
      {
        Work_Move(1,0,30);
        HAL_Delay(1500);
        Work_Straight(1);
        Check_Position_B = 3;

        if(SteppingTime >28)
        {
          SteppingTime = 0;
          Work_Move(1,0,200);
          HAL_Delay(1500);
        }

        else if (SteppingTime <10)
        {
          SteppingTime = 0;
          Work_Move(1,1,150);
          HAL_Delay(1500);
        }
        else 
        {
          SteppingTime = 0;
        }
        rxBuffer_A[0] = '3';    //修改为无意义标识
      }		
    }
#endif 

    //云台顺时针旋转30度到达水平位置
#if Check_Gimbal_CCW_First
  if(Check_Position_C == 7)
    {
      Check_Position_C = 8;
      Emm_V5_Pos_Control(5,0,500,175,534,0,0);	//电机逆时针旋转  534为60度 力臂30度
      HAL_Delay(4000);
      HAL_TIM_Base_Start_IT(&htim6);
    }
#endif

    //电机出发!!! 到达最终目的地
#if Check_Centre_2_END
 if(Check_Position_C == 8)   
  {
    		
		while(sign<4000)
		{
			HAL_Delay(1);
			p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,-1000);
			p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,1000);
			CAN_cmd_chassis(p1,p2,0,0);
				
			}
			rxBuffer_C[0] = 0;
			HAL_TIM_Base_Stop_IT(&htim6);
			sign = 0;

			while(rxBuffer_C[0] != '1')
			{
				HAL_Delay(1);
				p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,-500);
				p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,500);
				CAN_cmd_chassis(p1,p2,0,0);
				
			}
		if(rxBuffer_C[0] == '1')    //收到字符1 即为检测到停止标识
    {
			Check_Position_C = 9;
      HAL_TIM_Base_Start_IT(&htim6);
      while(sign <1500)
      {
      HAL_Delay(1);
      p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,0);
      p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,-0);
      CAN_cmd_chassis(p1,p2,0,0);	
      }
      HAL_TIM_Base_Stop_IT(&htim6);
			sign = 0;
    }	
    //stop
    
  }
#endif

   //最后的检查----终末的下落检测
#if Check_Last_PullDOWN
    rxBuffer_A[0] = '3';
    rxBuffer_B[0] = '3';

	 if(Check_Position_A == 5)
  {
      while(rxBuffer_B[0] != '2')   //什么也没收到 一直移动
    {
      Work_Move(3,1,25);			
      HAL_Delay(180);
    }
    if(rxBuffer_B[0] == '2')    //收到字符2 即为检测到木桩
    {
      Work_Move(3,1,30);
      HAL_Delay(1000);
	  	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,90);
			Work_Move(3,1,15);
			HAL_Delay(1500);
      Work_PullDown(2,3);
      Check_Position_A = 6;
      rxBuffer_B[0] = '3';
     }
  }
  if(Check_Position_B == 3)
    {
        while(rxBuffer_A[0] != '2')   //什么也没收到 一直移动
      {
        Work_Move(1,0,25);			
        HAL_Delay(180);
      }
      if(rxBuffer_A[0] == '2')    //收到字符2 即为检测到木桩
      {
        Work_Move(1,0,30);
        HAL_Delay(1000);
        __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);
        HAL_Delay(300);
        Work_Move(3,1,15);
        HAL_Delay(1500);
        Work_PullDown(1,3);
        Check_Position_B = 4;
        rxBuffer_A[0] = '3';
      }
    }
#endif

		
	//本次代码


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while ( 1 )
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART8)
	{
		HAL_UART_Receive_IT(&huart8,rxBuffer_B,sizeof(uint8_t));
	}
	if(huart->Instance == UART7)
	{
		HAL_UART_Receive_IT(&huart7,rxBuffer_A,sizeof(uint8_t));
	}
	if(huart->Instance == USART6)
	{
		HAL_UART_Receive_IT(&huart6,rxBuffer_C,sizeof(uint8_t));
	}
}
	
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//一毫秒进入一次中断函数
	if(htim->Instance == TIM6)
	{
		sign++;
	}
	if(htim->Instance == TIM7)
	{
		gimbal++;
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
