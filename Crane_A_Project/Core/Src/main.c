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

//����PID�ṹ��
Cascade_PID gimbal_pid,chassis_pid_1,chassis_pid_2; //��̨��������ӵ��
//������ݽṹ�� �������������� �������
MOTOR motor[3];
uint8_t avr_flag = 0;  	//ƽ���ٶȼ�����־λ ���Ҫ����

uint8_t Check_Position_A = 0; //A��� �н���־λ
uint8_t Check_Position_B = 0; //B��� �н���־λ
uint8_t Check_Position_C = 0; //C���̵�� �н���־λ

uint8_t First_Time = 0;

//����M3508�����PID����ֵ
int p1 = 0 ;
int p2 = 0 ;
int p3 = 0 ;

//����洢�н�λ�ñ���Լ���ȡ����λ��ȷ����Ϣ

uint8_t rxBuffer_A[2] = {0};	//����A������£�
uint8_t rxBuffer_B[2] = {0};	//����B������£�
uint8_t rxBuffer_C[2] = {0};	//���յ����ɫֹͣ����λ������


//��������
int16_t num[5][2] = {{18,18},{42,42},{60,60},{58,58},{129,129}}; // �������� ��δʵ������
int32_t CurrentAngle[5][2];	//�����Ƕ�

//��־λ 
uint16_t Tx_Time = 0;
uint8_t first_flag = 0; //��һ���ϵ��־λ ���ڼ�¼�״��ϵ��Ƕ�

uint8_t SteppingTime = 0;
uint32_t sign,gimbal;		//��ʱ���жϼ���ֵ

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
	

	//����PWM ���ƶ��
	
	//A�����
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	//B�����
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	
	//����CANͨ��	
	can_filter_init();

	//��ʼ��2��λ�õ�M3508
	cheel_init();
	
	//���������ж�
	HAL_UART_Receive_IT(&huart7,rxBuffer_A,sizeof(uint8_t));	//����A��������޻�������
	
	HAL_UART_Receive_IT(&huart8,rxBuffer_B,sizeof(uint8_t));	//����B��������޻�������

	HAL_UART_Receive_IT(&huart6,rxBuffer_C,sizeof(uint8_t));	//���յ����ɫֹͣ����λ��


	//�ȴ������ʼ��
	HAL_Delay(3000);
	
#if Motor_Initialization 
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);	//A�����(��һ�Ŷ��)	120ˮƽ 100���
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,90);	//B�����(�����Ŷ��) 90ˮƽ 120���
	HAL_Delay(500);
	Work_Up(2,4);	
	HAL_Delay(50);
	Work_Up(4,4);
	HAL_Delay(3000);
  //����۳�ʼλ��Ϊ�����  (�ݶ�)

#endif

		//���δ���

  
	//��⿪ʼλ������
	//��ȡ��ʼλ������
#if Check_Begin_Position
  if(Check_Position_A == 0)
  {
      while(rxBuffer_B[0] != '1')   //ʲôҲû�յ� һֱ�ƶ�
    {
      Work_Move(3,1,30);
			SteppingTime ++;
      HAL_Delay(160);
    }
    if(rxBuffer_B[0] == '1')    //�յ��ַ�1 ��Ϊ��⵽����
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
      rxBuffer_B[0] = '3';    //�޸�Ϊ�������ʶ
    }		
  }

#endif

//��̨ǰ��������������������Ϸ�
#if Check_First_Forward
  if(Check_Position_C == 0)   
  {
    while(rxBuffer_C[0] != '1')   //ʲôҲû�յ� һֱ�ƶ�
      {
      HAL_Delay(1);
      p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,500);
      p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,-500);
      CAN_cmd_chassis(p1,p2,0,0);	
      }
    if(rxBuffer_C[0] == '1')    //�յ��ַ�1 ��Ϊ��⵽ֹͣ��ʶ
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
  //��������
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
		if(rxBuffer_B[0] == '2')    //�յ��ַ�2 ��Ϊ��⵽ľ׮
		{
			HAL_Delay(50);
	  	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,90);
      HAL_Delay(300);
			Work_Move(3,1,15);
			HAL_Delay(1500);
			Work_PullDown(2,2);
			Check_Position_A = 2;
			HAL_Delay(1500);
			rxBuffer_B[0] = '3';    //�޸�Ϊ�������ʶ
			HAL_TIM_Base_Start_IT(&htim6);
			}
	}
#endif
	//��̨ǰ������̨λ������������Ϸ�
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
		if(rxBuffer_C[0] == '1')    //�յ��ַ�1 ��Ϊ��⵽ֹͣ��ʶ
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
	
	//��̨˳ʱ���ƶ�60��
#if Check_Gimbal_CW_First
	if(Check_Position_C == 2)
	{
		HAL_TIM_Base_Stop_IT(&htim6);	//�رն�ʱ��6 ֹͣ�����н�����
		sign = 0;	
		Check_Position_C = 3;

		Work_Move(3,1,130);
		HAL_Delay(50);
    Work_Move(1,0,480);
		Emm_V5_Pos_Control(5,1,500,175,1068,0,0);	//�����ʱ����ת  ��̨˳ʱ����ת  534Ϊ30��
		HAL_Delay(4000);
		
	}
#endif	


	// �������ڼ��˳ʱ��30�ȷ�������
#if Check_First_Centre_Position 
  if(Check_Position_A == 2)
  {
      while(rxBuffer_B[0] != '1')   //ʲôҲû�յ� һֱ�ƶ�
    {
      Work_Move(3,0,30);			
			SteppingTime ++;
      HAL_Delay(160);
    }
    if(rxBuffer_B[0] == '1')    //�յ��ַ�1 ��Ϊ��⵽����
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
      rxBuffer_B[0] = '3';    //�޸�Ϊ�������ʶ
    }		
  }

  if(Check_Position_B == 0)
  {
      while(rxBuffer_A[0] != '1')   //ʲôҲû�յ� һֱ�ƶ�
    {
      Work_Move(1,1,30);			
			SteppingTime ++;
      HAL_Delay(160);
    }
    if(rxBuffer_A[0] == '1')    //�յ��ַ�1 ��Ϊ��⵽����
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
      rxBuffer_A[0] = '3';    //�޸�Ϊ�������ʶ
    }		

  }
#endif

  //��̨˳ʱ����ת��ˮƽ
#if Check_Gimbal_CW_Twice
	if(Check_Position_C == 3)
	{
		Check_Position_C = 4;
		Emm_V5_Pos_Control(5,1,500,175,534,0,0);	//�����ʱ����ת ��̨˳ʱ����ת 534Ϊ60�� ����30��
		HAL_Delay(4000);
		HAL_TIM_Base_Start_IT(&htim6);
	}
#endif

  //ǰ����ĩ��
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

    //���ľ׮
#if Check_Twice_PullDown
    //����������г��ֵ�����
    rxBuffer_A[0] = '3';
    rxBuffer_B[0] = '3';

	 if(Check_Position_A == 3)
  {
      while(rxBuffer_B[0] != '2')   //ʲôҲû�յ� һֱ�ƶ�
    {
      Work_Move(3,1,25);			
      HAL_Delay(180);
    }
    if(rxBuffer_B[0] == '2')    //�յ��ַ�2 ��Ϊ��⵽ľ׮
    {
      //�������30mm
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
      while(rxBuffer_A[0] != '2')   //ʲôҲû�յ� һֱ�ƶ�
    {
      Work_Move(1,0,25);			
      HAL_Delay(180);
    }
    if(rxBuffer_A[0] == '2')    //�յ��ַ�2 ��Ϊ��⵽ľ׮
    {
      //�������30mm
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

  //��������
#if Check_Destination_2_Centre
  if(Check_Position_C == 5)   
  {
    while(rxBuffer_C[0] != '1')   //ʲôҲû�յ� һֱ�ƶ�
      {
      HAL_Delay(1);
      p1 = PID_calc(&chassis_pid_1.inner,motor[0].speed_rpm,-500);    //��ת
      p2 = PID_calc(&chassis_pid_2.inner,motor[1].speed_rpm,500);
      CAN_cmd_chassis(p1,p2,0,0);	
      }
    if(rxBuffer_C[0] == '1')    //�յ��ַ�1 ��Ϊ��⵽ֹͣ��ʶ
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


  //˳ʱ����ת30�� 
#if Check_Gimbal_CW_Third
  if(Check_Position_C == 6)
	{
    HAL_TIM_Base_Stop_IT(&htim6);
    sign = 0;
		Check_Position_C = 7;
		Emm_V5_Pos_Control(5,1,500,175,534,0,0);	//�����ʱ����ת  534Ϊ60�� ����30��
		HAL_Delay(4000);

  }
#endif

    //�����ʱ��30���ϵ�����
#if Check_Twice_Centre_Position
  if(Check_Position_A == 4)
    {
        while(rxBuffer_B[0] != '1')   //ʲôҲû�յ� һֱ�ƶ�
      {
        Work_Move(3,0,30);			
        SteppingTime ++;
        HAL_Delay(160);
      }
      if(rxBuffer_B[0] == '1')    //�յ��ַ�1 ��Ϊ��⵽����
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
        rxBuffer_B[0] = '3';    //�޸�Ϊ�������ʶ
      }		
    }
  if(Check_Position_B == 2)
    {
        while(rxBuffer_A[0] != '1')   //ʲôҲû�յ� һֱ�ƶ�
      {
        Work_Move(1,1,30);			
        SteppingTime ++;
        HAL_Delay(160);
      }
      if(rxBuffer_A[0] == '1')    //�յ��ַ�1 ��Ϊ��⵽����
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
        rxBuffer_A[0] = '3';    //�޸�Ϊ�������ʶ
      }		
    }
#endif 

    //��̨˳ʱ����ת30�ȵ���ˮƽλ��
#if Check_Gimbal_CCW_First
  if(Check_Position_C == 7)
    {
      Check_Position_C = 8;
      Emm_V5_Pos_Control(5,0,500,175,534,0,0);	//�����ʱ����ת  534Ϊ60�� ����30��
      HAL_Delay(4000);
      HAL_TIM_Base_Start_IT(&htim6);
    }
#endif

    //�������!!! ��������Ŀ�ĵ�
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
		if(rxBuffer_C[0] == '1')    //�յ��ַ�1 ��Ϊ��⵽ֹͣ��ʶ
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

   //���ļ��----��ĩ��������
#if Check_Last_PullDOWN
    rxBuffer_A[0] = '3';
    rxBuffer_B[0] = '3';

	 if(Check_Position_A == 5)
  {
      while(rxBuffer_B[0] != '2')   //ʲôҲû�յ� һֱ�ƶ�
    {
      Work_Move(3,1,25);			
      HAL_Delay(180);
    }
    if(rxBuffer_B[0] == '2')    //�յ��ַ�2 ��Ϊ��⵽ľ׮
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
        while(rxBuffer_A[0] != '2')   //ʲôҲû�յ� һֱ�ƶ�
      {
        Work_Move(1,0,25);			
        HAL_Delay(180);
      }
      if(rxBuffer_A[0] == '2')    //�յ��ַ�2 ��Ϊ��⵽ľ׮
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

		
	//���δ���


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
	//һ�������һ���жϺ���
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
