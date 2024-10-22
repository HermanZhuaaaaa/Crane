#ifndef CENTRE_H
#define CENTRE_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "PID.h"

//定义电机数据的结构体
typedef struct{
	uint16_t ecd;//角度
  int16_t speed_rpm;//转每分
  int16_t given_current;//电流
  uint8_t temperate;//温度
  int16_t last_ecd;//原角度
	
	pid_type_def pid;//该电机的PID结构体
	
}MOTOR;

extern MOTOR motor[3];

//对轮子进行初始化
extern void cheel_init(void);

//两轮子进行速度控制
extern void motor_speed_control(int set, int time);

//对云台进行角度控制
extern void gimbal_angle_control(int angle);

#endif