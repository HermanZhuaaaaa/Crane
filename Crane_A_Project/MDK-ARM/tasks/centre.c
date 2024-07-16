#include "stdio.h"
#include "centre.h"
#include "PID.h"
#include "time.h"

//定义三个电机对象,其中0、1为轮子,3为云台
extern MOTOR motor[3];

//对轮子进行初始化
void cheel_init(void)
{
	const float init_pid[3] = {1, 0.5, 0.1};//设定P、I、D三个参数
	int now = 0;
	int max_out = 2500, max_iout = 1250;
	for(int i = 0; i < 2; i ++){
		//PID初始化函数
		PID_init(&motor[i].pid, init_pid, max_out, max_iout);
	}
}

//两轮子进行控制,其中set是预定的速度,time是前进时间
void motor_speed_control(int set, int time)
{
	time = clock() + time;
	while(1){
		if (clock() != time)
		{
			int p1 = PID_calc(&motor[0].pid, motor[0].speed_rpm, set);
			int p2 = PID_calc(&motor[1].pid, motor[1].speed_rpm, set);
			CAN_cmd_chassis(p1, p2, 0, 0);
		}
		else if (clock() >= time)
			break;
	}
	CAN_cmd_chassis(0, 0, 0, 0);
	for(int i = 0; i < 2; i ++){
		PID_clear(&motor[i].pid);
	}
}

//初始化云台
void gimabal_init(void)
{
	const float init_pid[3] = {0, 0, 0};//设定P、I、D三个参数
	int now = 0, setmax = 0;
	int max_out = 0, max_iout = 0;
	//PID初始化函数
	PID_init(&motor[2].pid, init_pid, max_out, max_iout);
}

//对云台进行角度控制
void gimbal_angle_control(int angle)
{
	
}

