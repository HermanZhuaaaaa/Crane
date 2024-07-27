#include "stm32f4xx.h"                  // Device header
#include "tim.h"
#include "Emm_V5.h"
#include "usart.h"


//Length�����ĸ��ȼ� 1 2 3 4 :�ֱ�Ϊ	��͹�ȡλ��2500		��һ����ľ׮λ��4000		��ľ׮λ��9200		���λ��12000		
void Work_Up(uint8_t addr, uint16_t length)
{
	uint16_t sign = 0;
	if(length == 1) sign = 2200;
	if(length == 2) sign = 5600;
	if(length == 3) sign = 9000;
	if(length == 4) sign = 11000; 

	if(addr == 2)
	{
		Emm_V5_Pos_Control(addr,1,50,125,sign,1,0);	
		HAL_Delay(50);
	}
	else if (addr ==4)
	{
		Emm_V5_Pos_Control(addr,0,50,125,sign,1,0);	
		HAL_Delay(50);
	}
}


/**
  * @brief		������µ����ƶ�
  * @param		�����ַaddr,�߶�length
  * @retval		none
  */
void Work_Up_Low_Speed(uint8_t addr, uint16_t length)
{
	uint16_t sign = 0;
	if(length == 1) sign = 2200;
	if(length == 2) sign = 5600;
	if(length == 3) sign = 9000;
	if(length == 4) sign = 11000; 

	if(addr == 2)
	{
		Emm_V5_Pos_Control(addr,1,25,125,sign,1,0);	
		HAL_Delay(50);
	}
	else if (addr ==4)
	{
		Emm_V5_Pos_Control(addr,0,25,125,sign,1,0);	
		HAL_Delay(50);
	}
}


/**
  * @brief		����ƶ�ָ������
  * @param		�����ַaddr,�����ת����dir,����length
  * @retval		none
  */
void Work_Move(uint8_t addr ,uint8_t dir,uint16_t length)
{
	
	//addr Ϊ 1 ���� 3
	uint32_t pulse = length*63;
	Emm_V5_Pos_Control(addr,dir,3500,200,pulse,0,0);	
	HAL_Delay(50);
}


/**
  * @brief		�״�!!! ��Ȧ��⵽��ֱ�ӹ�ȡ
  * @param		none
  * @retval		none
  */
void Work_Straight_FirstTime( void)
{

		Work_Up(2,1);
		HAL_Delay(3000);

		Work_Move(1,1,50);
		HAL_Delay(1200);

		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,100);
		Work_Up(2,4);
		HAL_Delay(50);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,110);
		HAL_Delay(3000);
}



/**
  * @brief		�״�!!!�����Ȧ���ޣ���Ȧû�У�������Ȧ��ȡ
  * @param		none
  * @retval		none
  */
void Work_OuterToInner_FirstTime(void)
{
		//��������ķ���Ҫ�ӵ�����濴
		//1,2�����ǰ�� ��ʱ
		Work_Move(1,1,300);
		HAL_Delay(4000);
		Work_Up(2,1);
		HAL_Delay(3000);
		Work_Move(1,1,70);
		HAL_Delay(1500);

		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,100);	//�������
		Work_Up(2,4);
		HAL_Delay(50);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,110);
		HAL_Delay(3000);
		
		//�����⻷λ��
		Work_Move(1,0,320 );
		HAL_Delay(4000);
}

/**
  * @brief		�����ĺ�!!!��Ȧ��⵽��ֱ�ӹ�ȡ
  * @param		������� Class
  * @retval		none
  */
void Work_Straight(uint8_t Class)
{
	if(Class == 1)		//һ�Ŷ��λ��
	{
		Work_Up(2,1);
		HAL_Delay(3000);

		Work_Move(1,1,30);
		HAL_Delay(1200);

		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,100);
		Work_Up(2,4);
		HAL_Delay(3000);
		// __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);
		// HAL_Delay(100);
	}
	else if(Class == 2)  	//���Ŷ��λ��
	{
		//�󳷲� Ӧ�ò���Ҫ
		Work_Up(4,1);
		HAL_Delay(3000);

		Work_Move(3,0,30);
		HAL_Delay(1200);

		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,120);
		Work_Up(4,4);
		HAL_Delay(3000);	
		// __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,90); 
		// HAL_Delay(100);
	}
}


/**
  * @brief		�����ĺ�!!!�����Ȧ���ޣ���Ȧû�У�������Ȧ��ȡ
  * @param		������� Class
  * @retval		none
  */
void Work_OuterToInner(uint8_t Class)
{
		//��������ķ���Ҫ�ӵ�����濴
	//1,2�����ǰ�� ��ʱ
	if(Class == 1)	//һ�Ŷ����
	{
		Work_Move(1,1,300);
		HAL_Delay(4000);
		Work_Up(2,1);
		HAL_Delay(3000);
		Work_Move(1,1,70);
		HAL_Delay(1500);

		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,100);	//�������
		Work_Up(2,4);
		HAL_Delay(50);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,110);
		HAL_Delay(3000);
		
		Work_Move(1,0,320);
		HAL_Delay(4000);
		
	}
	else if (Class == 2)	//���Ŷ����
	{
		Work_Move(3,0,300);
		HAL_Delay(4000);
		Work_Up(4,1);
		HAL_Delay(3000);
		Work_Move(3,0,70);
		HAL_Delay(1500);

		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,120);
		Work_Up(4,4);
		HAL_Delay(50);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,110);
		HAL_Delay(3000);

		Work_Move(3,1,320);
		HAL_Delay(4000);
		
	}
}


/**
  * @brief		�״�!!!�������붯��
  * @param		none	
  * @retval		none
  */
void Work_PullDown_FirstTime(void)
{
	HAL_Delay(50);
	Work_Up(2,2);		//�����������ڶ��ȼ��߶�
	HAL_Delay(3000);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);
	HAL_Delay(750);
	Work_Move(1,0,40);//�����ƶ�40mm ׼��̧����
	HAL_Delay(1500);
	Work_Up(2,4);		//̧����
	HAL_Delay(2500);
	Work_Move(1,1,40);//�������ԭ���⻷λ��
	HAL_Delay(1500);
	Emm_V5_Reset_CurPos_To_Zero(1);		//����Ƕ�����
	HAL_Delay(50);
}

/**
  * @brief		�����ĺ�!!!�������붯��
  * @param		������Class
  * @retval		none
  */
void Work_PullDown(uint8_t Class,uint8_t length)
{
	if(Class == 1)
	{
		Work_Move(1,1,5);
		HAL_Delay(50);
		Work_Up_Low_Speed(2,length);		//������������len�ȼ��߶�
		HAL_Delay(4000);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);
		HAL_Delay(750);
		Work_Move(1,0,50);//�����ƶ�40mm ׼��̧����
		HAL_Delay(1500);
		Work_Up(2,4);		//̧����
		HAL_Delay(2500);
		Work_Move(1,1,50);//�������ԭ���⻷λ��
		HAL_Delay(1500);
		Emm_V5_Reset_CurPos_To_Zero(1);		//����Ƕ�����
		HAL_Delay(50);
	}
	else if (Class == 2)
	{
		Work_Move(3,0,5);
		HAL_Delay(750);
		Work_Up_Low_Speed(4,length);		//������������len�ȼ��߶�
		HAL_Delay(4000);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,90);
		HAL_Delay(750);
		Work_Move(3,1,50);//�����ƶ�40mm ׼��̧����
		HAL_Delay(1500);
		Work_Up(4,4);		//̧����
		HAL_Delay(2500);
		Work_Move(3,0,50);//�������ԭ���⻷λ��
		HAL_Delay(1500);
		Emm_V5_Reset_CurPos_To_Zero(3);		//����Ƕ�����
		HAL_Delay(50);
	}
}
/**
  * @brief		�״�!!!���Ŀ������
  * @param		none
  * @retval		none
  */
void Work_Check_First(void)
{
	

}

