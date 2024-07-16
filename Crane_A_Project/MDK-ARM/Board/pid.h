//��ͷ�ļ�������PID�㷨�����м�����Լ����������÷�(�÷����ʹ�����)
#ifndef PID_H
#define PID_H
//����PID������ݽṹ��
typedef struct{
	//PID���ϵ��
	float Kp;
	float Ki;
	float Kd;
	
	//Ԥ��������
	float max_out; //�����
	float max_iout; //�������
	
	//�ﵽĿ��ֵ����ǰֵ
	float set;
	float now;
	
	//���ֵ
	float out;		//���������
  float pout;	//���������
  float iout;	//���������
  float dout;	//΢�������
	
	//΢�����������ֵ0����1��һ��2���ϴ�
  float dcur[3];  
  //������������ֵ0����1��һ��2���ϴ�
  float error[3];  
} pid_type_def;


typedef struct
{
	pid_type_def inner;
	pid_type_def outer;
	float out;
}Cascade_PID;

//ǰ���޷�����
extern float limit(int now, int setmax);

//PID��ʼ������
extern void PID_init(pid_type_def *pid, const float PID[3], float max_out, float max_iout);

//PID���㺯��(λ��)
extern float PID_calc(pid_type_def *pid, float now, float set);

//����м��������
extern void PID_clear(pid_type_def *pid);

//���㴮��PID
extern float Cascade_PID_calc(Cascade_PID *pid,float outer_set,float outer_now,float inner_now);

#endif
