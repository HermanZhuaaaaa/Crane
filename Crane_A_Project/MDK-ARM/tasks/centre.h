#ifndef CENTRE_H
#define CENTRE_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "PID.h"

//���������ݵĽṹ��
typedef struct{
	uint16_t ecd;//�Ƕ�
  int16_t speed_rpm;//תÿ��
  int16_t given_current;//����
  uint8_t temperate;//�¶�
  int16_t last_ecd;//ԭ�Ƕ�
	int32_t total_circle; //ȫ����
	int16_t circle;	//Ȧ��
	float angle;
	
	pid_type_def pid_inner;//�õ����PID�ṹ��
	pid_type_def pid_outer;
	pid_type_def pid_cloud;
}MOTOR;

extern MOTOR motor[3];

//�����ӽ��г�ʼ��
extern void cheel_init(void);

//�����ӽ����ٶȿ���
//extern void motor_speed_control(int set, int time);
//�����ӽ���λ�ÿ���
extern void motor_position_control(float set_pos,uint8_t addr);
//����̨���нǶȿ���
extern void gimbal_angle_control(int angle);
void Cale_angle_t(MOTOR *motor);
#endif