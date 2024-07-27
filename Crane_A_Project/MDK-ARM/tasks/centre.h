#ifndef CENTRE_H
#define CENTRE_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "PID.h"

//���������ݵĽṹ��
typedef struct{
	uint16_t ecd;//�Ƕ�
  int16_t speed_rpm;//תÿ��
	float average_speed[10];
	int	avr_speed;
  int16_t given_current;//����
  uint8_t temperate;//�¶�
  int16_t last_ecd;//ԭ�Ƕ�
	int32_t total_circle; //ת������ĵ�Ƕ�
	int16_t circle;	//Ȧ��
	float angle;			//ת���ɽǶ�
	
	//��ʼ�Ƕ�
	float m3508_angle_zero;
	uint16_t m3508_circle_zero;
	uint32_t m3508_total_circle_zero;
	
	pid_type_def pid_inner;//�õ����PID�ṹ��
	pid_type_def pid_outer;
	pid_type_def pid_cloud;
}MOTOR;

extern MOTOR motor[3];

//�����ӽ��г�ʼ��
extern void cheel_init(void);

//����Ƕ�ֵ
extern void Cale_angle_t(MOTOR *motor);
 
//����ṹ����ֵ
extern void Cale_angle_clear(MOTOR *motor,Cascade_PID *mypid);

//�Ƕȵ�������      
extern void Cale_Angle_Arrive(MOTOR *motor,int32_t angle);

//����ƽ��ת��
extern void Cale_speed_average(MOTOR *motor);
#endif
