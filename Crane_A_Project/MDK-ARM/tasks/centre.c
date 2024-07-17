#include "stdio.h"
#include "centre.h"
#include "PID.h"
#include "time.h"

//定义三个电机对象,其中0、1为轮子,3为云台
extern MOTOR motor[3];
extern Cascade_PID my_pid;
float pos;


//对轮子进行初始化
void cheel_init(void)
{
	const 	float init_pid_inner[3] = {13, 0.5, 0.8};//设定P、I、D三个参数 2.5 0.09 0.005
	
	const float init_pid_cloud[3] = {40	,0.4,0}; //40 0.4 0 out:3k
	const 	float init_pid_outer[3] = {50	, 0, 0.3}; // 50 0 0.3

	int now = 0, set = 0;
	int max_out_inner = 2000, max_iout_inner = 1000;
	int max_out_outer = 2000, max_iout_outer = 2500;	
	int max_out_cloud = 4500, max_iout_cloud = 4500;	
//	
	PID_init(&my_pid.inner,init_pid_inner, max_out_inner, max_iout_inner);
	PID_init(&my_pid.outer,init_pid_outer, max_out_outer, max_iout_outer);
//	PID_init(&my_pid.cloud,init_pid_cloud, max_out_cloud, max_iout_cloud);

	for(int i = 0; i <= 2; i ++){
		//PID初始化函数
		PID_init(&motor[i].pid_inner, init_pid_inner, max_out_inner, max_iout_inner);
		PID_init(&motor[i].pid_outer, init_pid_outer, max_out_outer, max_iout_outer);
		PID_init(&motor[i].pid_cloud, init_pid_cloud, max_out_cloud, max_iout_cloud);
	}
	
}

/**
  * @brief		计算电角度
  * @param
  * @retval
  */
/**
  * @brief		??????
  * @param
  * @retval
  */

void Cale_angle_t(MOTOR *motor)
{
	if(motor->ecd-motor->last_ecd>4096) motor->circle--;
	else if(motor->ecd-motor->last_ecd<-4096) motor->circle++;
	motor->total_circle = motor->circle * 8192 + motor->ecd ; 
	
	if (motor->total_circle >= 155648)	motor->total_circle =motor->total_circle - 155648;
	if (motor ->total_circle <0 ) motor->total_circle = motor->total_circle + 155648;
	motor->angle = motor->total_circle/19.0f/8192*360;
	//第一次跳变消除
			static uint8_t sign_flag = 1;
if(sign_flag)
{
	motor->circle = 0;
	sign_flag = 0;
}
	
}
//uint16_t motor_angle_calc(uint8_t addr)
//{
//	if((motor[addr].ecd - motor[addr].last_ecd > 5000) || (motor[addr].ecd - motor[addr].last_ecd < -5000))
//		circle +=1;
//}

void motor_position_control(float set_pos,uint8_t addr)
{
	pos = Cascade_PID_calc(&my_pid,set_pos,motor[addr].circle,motor[addr].speed_rpm);
	if(addr == 1) CAN_cmd_chassis(pos,0,0,0);
	if(addr == 2) CAN_cmd_chassis(0,pos,0,0);
	if(addr == 3) CAN_cmd_chassis(0,0,pos,0);
	
}
//两轮子进行控制,其中set是预定的速度,time是前进时间

//void motor_speed_control(int set, int time)
//{
//	time = clock() + time;
//	while(clock() <= time){
//			int p1 = PID_calc(&motor[0].pid, motor[0].speed_rpm, set);
//			int p2 = PID_calc(&motor[1].pid, motor[1].speed_rpm, set);
//			CAN_cmd_chassis(p1, p2, 0, 0);
//		
//	}
//	CAN_cmd_chassis(0, 0, 0, 0);
//	for(int i = 0; i < 2; i ++){
//		PID_clear(&motor[i].pid);
//	}
//}


//初始化云台
//void gimabal_init(void)
//{
//	const float init_pid[3] = {0, 0, 0};//设定P、I、D三个参数
//	int now = 0, setmax = 0;
//	int max_out = 0, max_iout = 0;
//	//PID初始化函数
//	PID_init(&motor[2]., init_pid, max_out, max_iout);
//}

//对云台进行角度控制
void gimbal_angle_control(int angle)
{
	
}

