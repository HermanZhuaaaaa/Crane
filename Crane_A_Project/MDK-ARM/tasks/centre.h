#ifndef CENTRE_H
#define CENTRE_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "PID.h"

//定义电机数据的结构体
typedef struct{
	uint16_t ecd;//角度
  int16_t speed_rpm;//转每分
	float average_speed[10];
	int	avr_speed;
  int16_t given_current;//电流
  uint8_t temperate;//温度
  int16_t last_ecd;//原角度
	int32_t total_circle; //转过整体的电角度
	int16_t circle;	//圈数
	float angle;			//转换成角度
	
	//起始角度
	float m3508_angle_zero;
	uint16_t m3508_circle_zero;
	uint32_t m3508_total_circle_zero;
	
	pid_type_def pid_inner;//该电机的PID结构体
	pid_type_def pid_outer;
	pid_type_def pid_cloud;
}MOTOR;

extern MOTOR motor[3];

//对轮子进行初始化
extern void cheel_init(void);

//计算角度值
extern void Cale_angle_t(MOTOR *motor);
 
//清除结构体数值
extern void Cale_angle_clear(MOTOR *motor,Cascade_PID *mypid);

//角度递增到达      
extern void Cale_Angle_Arrive(MOTOR *motor,int32_t angle);

//计算平均转速
extern void Cale_speed_average(MOTOR *motor);
#endif
