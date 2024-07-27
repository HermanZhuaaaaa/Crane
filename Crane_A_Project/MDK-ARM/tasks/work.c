#include "stm32f4xx.h"                  // Device header
#include "tim.h"
#include "Emm_V5.h"
#include "usart.h"


//Length划分四个等级 1 2 3 4 :分别为	最低钩取位置2500		第一个低木桩位置4000		高木桩位置9200		最高位置12000		
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
  * @brief		电机上下低速移动
  * @param		电机地址addr,高度length
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
  * @brief		电机移动指定距离
  * @param		电机地址addr,电机旋转方向dir,长度length
  * @retval		none
  */
void Work_Move(uint8_t addr ,uint8_t dir,uint16_t length)
{
	
	//addr 为 1 或者 3
	uint32_t pulse = length*63;
	Emm_V5_Pos_Control(addr,dir,3500,200,pulse,0,0);	
	HAL_Delay(50);
}


/**
  * @brief		首次!!! 外圈检测到，直接钩取
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
  * @brief		首次!!!检测外圈有无，外圈没有，进入内圈钩取
  * @param		none
  * @retval		none
  */
void Work_OuterToInner_FirstTime(void)
{
		//步进电机的方向要从电机后面看
		//1,2舵机在前方 此时
		Work_Move(1,1,300);
		HAL_Delay(4000);
		Work_Up(2,1);
		HAL_Delay(3000);
		Work_Move(1,1,70);
		HAL_Delay(1500);

		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,100);	//舵机勾起
		Work_Up(2,4);
		HAL_Delay(50);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,110);
		HAL_Delay(3000);
		
		//返回外环位置
		Work_Move(1,0,320 );
		HAL_Delay(4000);
}

/**
  * @brief		到中心后!!!外圈检测到，直接钩取
  * @param		电机组数 Class
  * @retval		none
  */
void Work_Straight(uint8_t Class)
{
	if(Class == 1)		//一号舵机位置
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
	else if(Class == 2)  	//二号舵机位置
	{
		//后撤步 应该不需要
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
  * @brief		到中心后!!!检测外圈有无，外圈没有，进入内圈钩取
  * @param		电机组数 Class
  * @retval		none
  */
void Work_OuterToInner(uint8_t Class)
{
		//步进电机的方向要从电机后面看
	//1,2舵机在前方 此时
	if(Class == 1)	//一号舵机臂
	{
		Work_Move(1,1,300);
		HAL_Delay(4000);
		Work_Up(2,1);
		HAL_Delay(3000);
		Work_Move(1,1,70);
		HAL_Delay(1500);

		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,100);	//舵机勾起
		Work_Up(2,4);
		HAL_Delay(50);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,110);
		HAL_Delay(3000);
		
		Work_Move(1,0,320);
		HAL_Delay(4000);
		
	}
	else if (Class == 2)	//二号舵机臂
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
  * @brief		首次!!!放下砝码动作
  * @param		none	
  * @retval		none
  */
void Work_PullDown_FirstTime(void)
{
	HAL_Delay(50);
	Work_Up(2,2);		//放下砝码至第二等级高度
	HAL_Delay(3000);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);
	HAL_Delay(750);
	Work_Move(1,0,40);//向外移动40mm 准备抬起舵机
	HAL_Delay(1500);
	Work_Up(2,4);		//抬起电机
	HAL_Delay(2500);
	Work_Move(1,1,40);//电机返回原来外环位置
	HAL_Delay(1500);
	Emm_V5_Reset_CurPos_To_Zero(1);		//电机角度清零
	HAL_Delay(50);
}

/**
  * @brief		到中心后!!!放下砝码动作
  * @param		舵机组别Class
  * @retval		none
  */
void Work_PullDown(uint8_t Class,uint8_t length)
{
	if(Class == 1)
	{
		Work_Move(1,1,5);
		HAL_Delay(50);
		Work_Up_Low_Speed(2,length);		//放下砝码至第len等级高度
		HAL_Delay(4000);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);
		HAL_Delay(750);
		Work_Move(1,0,50);//向外移动40mm 准备抬起舵机
		HAL_Delay(1500);
		Work_Up(2,4);		//抬起电机
		HAL_Delay(2500);
		Work_Move(1,1,50);//电机返回原来外环位置
		HAL_Delay(1500);
		Emm_V5_Reset_CurPos_To_Zero(1);		//电机角度清零
		HAL_Delay(50);
	}
	else if (Class == 2)
	{
		Work_Move(3,0,5);
		HAL_Delay(750);
		Work_Up_Low_Speed(4,length);		//放下砝码至第len等级高度
		HAL_Delay(4000);
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,90);
		HAL_Delay(750);
		Work_Move(3,1,50);//向外移动40mm 准备抬起舵机
		HAL_Delay(1500);
		Work_Up(4,4);		//抬起电机
		HAL_Delay(2500);
		Work_Move(3,0,50);//电机返回原来外环位置
		HAL_Delay(1500);
		Emm_V5_Reset_CurPos_To_Zero(3);		//电机角度清零
		HAL_Delay(50);
	}
}
/**
  * @brief		首次!!!检测目标砝码
  * @param		none
  * @retval		none
  */
void Work_Check_First(void)
{
	

}

