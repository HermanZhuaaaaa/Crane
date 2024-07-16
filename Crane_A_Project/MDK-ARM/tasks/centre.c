#include "stdio.h"
#include "centre.h"
#include "PID.h"
#include "time.h"

//���������������,����0��1Ϊ����,3Ϊ��̨
extern MOTOR motor[3];
extern Cascade_PID my_pid;
float pos;

//�����ӽ��г�ʼ��
void cheel_init(void)
{
	const 	float init_pid_inner[3] = {2, 0, 0};//�趨P��I��D��������
	const 	float init_pid_outer[3] = {3, 0, 0};

	int now = 0;
	int max_out_inner = 1000, max_iout_inner = 700;
	int max_out_outer = 200, max_iout_outer = 250;	
	PID_init(&my_pid.inner,init_pid_inner, max_out_inner, max_iout_inner);
	PID_init(&my_pid.outer,init_pid_outer, max_out_outer, max_iout_outer);
	for(int i = 0; i < 2; i ++){
		//PID��ʼ������
		PID_init(&motor[i].pid_inner, init_pid_inner, max_out_inner, max_iout_inner);
		PID_init(&motor[i].pid_outer, init_pid_outer, max_out_outer, max_iout_outer);
		
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
//�����ӽ��п���,����set��Ԥ�����ٶ�,time��ǰ��ʱ��

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


//��ʼ����̨
//void gimabal_init(void)
//{
//	const float init_pid[3] = {0, 0, 0};//�趨P��I��D��������
//	int now = 0, setmax = 0;
//	int max_out = 0, max_iout = 0;
//	//PID��ʼ������
//	PID_init(&motor[2]., init_pid, max_out, max_iout);
//}

//����̨���нǶȿ���
void gimbal_angle_control(int angle)
{
	
}

