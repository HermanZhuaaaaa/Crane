#include "stm32f4xx.h"                  // Device header
#include "tim.h"
#include "Emm_V5.h"

void Work_SteppingMotorInit(void)
{

			Emm_V5_Pos_Control(1,1,0,0,0,1,0);		//逆时针往回拉
	
			Emm_V5_Pos_Control(2,0,2500,125,8400,1,0);	//顺时针拉上去
	
			Emm_V5_Pos_Control(3,0,2500,125,4000,1,0);	//顺时针拉 4000脉冲到 750mm外圈处。
	
			Emm_V5_Pos_Control(4,1,2500,125,8400,1,0);	//逆时针拉上去
	
}

void Work_pulldown(uint8_t addr,uint8_t servo)
{
	if(servo == 1)
	{
		//接在 A号PWM上的舵机
		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,85);		//前进方向舵机 占空比85是初始平放位置 120竖直位置
	Emm_V5_Pos_Control(2,1,2500,125,1000,1,0); 
	HAL_Delay(1500);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);
	HAL_Delay(100);
	Emm_V5_Pos_Control(2,0,1250,125,8400,1,0); 
	}
	
	if(servo == 2)
	{
		//接在 B号PWM上的舵机
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,85);		//前进方向舵机 占空比85是初始平放位置 120竖直位置
	Emm_V5_Pos_Control(addr,0,2500,125,1000,1,0); 
	HAL_Delay(1500);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);
	HAL_Delay(100);
	Emm_V5_Pos_Control(2,0,1250,125,8400,1,0); 

	}
}

void Work_FromInnerToOuter(void)
{
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,85);		//前进方向舵机 占空比85是初始平放位置 120竖直位置
			
	Emm_V5_Pos_Control(2,1,2500,125,8400,1,0); //8400高度足够 到达最大值 7400能够放置到高木上  4000足够放置到低木桩上
	HAL_Delay(1800);
	Emm_V5_Pos_Control(1,0,2500,125,20100,1,0);
	HAL_Delay(1800);
	Emm_V5_Pos_Control(2,0,1250,125,1700,1,0); 
	HAL_Delay(1750);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);
	Emm_V5_Pos_Control(2,1,1250,125,8400,1,0); 
}

void Work_FromOuterToInner(void)
{
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,85);		//前进方向舵机 占空比85是初始平放位置 120竖直位置
			
	Emm_V5_Pos_Control(2,0,2500,125,8400,1,0); //8400高度足够 到达最大值 7400能够放置到高木上  4000足够放置到低木桩上
	HAL_Delay(1500);
	Emm_V5_Pos_Control(1,0,2500,125,20000,1,0);
	HAL_Delay(1700);
	Emm_V5_Pos_Control(2,0,1250,125,1000,1,0); 
	HAL_Delay(750);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,120);
	Emm_V5_Pos_Control(2,0,1250,125,8400,1,0); 
}
