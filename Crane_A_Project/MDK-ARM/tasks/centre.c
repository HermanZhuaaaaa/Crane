#include "stm32f4xx.h"                  // Device header
#include "stdio.h"
#include "centre.h"
#include "PID.h"
#include "tim.h"


//定义三个电机对象,其中0、1为轮子，2为云台
extern MOTOR motor[3];
//串级PID
extern Cascade_PID gimbal_pid,chassis_pid_1,chassis_pid_2; //云台电机与轮子电机

//计算平均转速
extern uint8_t avr_flag;
extern uint32_t gimbal;
static uint8_t sign_flag = 1;

/**
  * @brief		清除MOTOR数据
  * @param
  * @retval
  */

void Cale_angle_clear(MOTOR *motor,Cascade_PID *mypid)
{
	motor->angle = 0;
	motor->circle = 0;
	motor->ecd = 0;
	motor ->last_ecd = 0;
	motor ->total_circle = 0;
	mypid->inner.dcur[0] = 	mypid->inner.dcur[1] = 	mypid->inner.dcur[2] = 0;
	mypid->inner.error[0] =	mypid->inner.error[1] =	mypid->inner.error[2] = 0;
}


//对轮子进行初始化
void cheel_init(void)
{
	
	Cale_angle_clear(&motor[0],&chassis_pid_1);
	Cale_angle_clear(&motor[1],&chassis_pid_2);
	Cale_angle_clear(&motor[2],&gimbal_pid);
	//const 	float init_pid_inner[3] = {13, 0.5, 0.8	};//设定P、I、D三个参数
	const 	float init_chassis_pid_inner[3] = {28	,0.4	,0	}; 
	const float init_chassis_pid_outer[3] = {28,0,0.01};

	const float init_gimbal_pid_inner[3] = {20,0.045,0};
	const 	float init__gimbal_pid_outer[3] = {43	,0	, 0.5	}; 
	int now = 0, set = 0;
	
	int max_out_inner = 4000, max_iout_inner = 4000;	// 8000 8000
	int max_out_outer = 350, max_iout_outer = 350;	// 1000 1000调整
	int max_out_cloud = 700, max_iout_cloud = 700;	
	float DeadZone_Outer = 2500;
	float DeadZone_Inner = 0;
	
//串级PID初始化
	PID_init(&chassis_pid_1.inner,init_chassis_pid_inner,DeadZone_Inner,max_out_inner,max_iout_inner);
	PID_init(&chassis_pid_1.outer,init_chassis_pid_outer,DeadZone_Outer,max_iout_outer,max_iout_outer);
	
	PID_init(&chassis_pid_2.inner,init_chassis_pid_inner,DeadZone_Inner,max_out_inner,max_iout_inner);
	PID_init(&chassis_pid_2.outer,init_chassis_pid_outer,DeadZone_Outer,max_iout_outer,max_iout_outer);
	
	PID_init(&gimbal_pid.inner,init_gimbal_pid_inner,DeadZone_Inner,4000,4000);
	PID_init(&gimbal_pid.outer,init__gimbal_pid_outer,DeadZone_Outer,max_out_cloud,max_iout_cloud);
	
}

/**
  * @brief		角度递增到达
  * @param		
  * @retval
  */

void Cale_Angle_Arrive(MOTOR *motor,int32_t angle)
{
	HAL_TIM_Base_Start_IT(&htim7);
	uint8_t flag = 0;
	int32_t slice_angle = angle*8192*19/360 ; 
	slice_angle = (slice_angle - motor->m3508_total_circle_zero)/10;
	uint16_t i = 0;	
	if(flag == 0)
	{
	if(gimbal <=970)
	{
	float p3 = Cascade_PID_calc(&gimbal_pid,slice_angle,motor[2].total_circle,motor[2].speed_rpm);
	CAN_cmd_chassis(0,0,p3,0);
		HAL_Delay(1);
	}
	if(gimbal > 970 && gimbal <1030)
	{
		gimbal = 0;
		i++;
		if(i == 10)
		{
			HAL_TIM_Base_Stop_IT(&htim7);
			flag = 1;
		}
	}
}
}


/**
  * @brief		计算平均转速
  * @param
  * @retval
  */

void Cale_speed_average(MOTOR *motor)
{
	static uint8_t num = 0;
	float avr = 0;
	int avr_speed = 0;

	motor->average_speed[num] = motor->speed_rpm;
	num++;
	if(num == 10)
	{
		num = 0;
		for(int i =0;i<10;i++)                          
		{                           
			avr+=motor->average_speed[i];
		}
		avr_speed = avr/10 ;
		motor->avr_speed = avr_speed;
		avr_flag = 1;
	}
}


/**
  * @brief		计算电角度
  * @param
  * @retval
  */

void Cale_angle_t(MOTOR *motor)
{
	
	if(motor->ecd-motor->last_ecd>4096) motor->circle--;
	else if(motor->ecd-motor->last_ecd<-4096) motor->circle++;
	motor->total_circle = motor->circle * 8192 + motor->ecd ; 
	
	//if (motor->total_circle >= 155648)	motor->total_circle =motor->total_circle - 155648;
	//if (motor ->total_circle <=0 ) motor->total_circle = motor->total_circle + 155648;
	motor->angle = motor->total_circle/19.0f/8192*360;
	//第一次跳变消除
	if(sign_flag)
	{
		motor->circle = 0;
		sign_flag = 0;
	}
	
}


