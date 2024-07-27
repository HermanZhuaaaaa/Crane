//PID��غ��������ļ�
#include "main.h"
#include "PID.h"
//�޷��ú���
float limit(int now, int setmax)
{ 
	if(now < - setmax) return -setmax;
	if(now > setmax) return setmax;
	else
			return now;
}

//PID��ʼ������
void PID_init(pid_type_def *pid, const float PID[3],float deadzone, float max_out, float max_iout)
{
		//�ж��Ƿ���PID���Ƶ����Ҫ��ʼ��
    if (pid == NULL || PID == NULL)
    {
        return;
    }
		//��ֵ�����ϵ�����г�ʼ��
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
		//��ʼ�����ֵ
    pid->max_out = max_out;
    pid->max_iout = max_iout;
		
		pid->DeadZone = deadzone;
		
		//��ʼ����ֵ�����ֵ
    pid->dcur[0] = pid->dcur[1] = pid->dcur[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->pout = pid->iout = pid->dout = pid->out = 0.0f;
}


//PID���㺯��(λ��)
float PID_calc(pid_type_def *pid, float now, float set)
{
    //�жϴ����PIDָ�벻Ϊ��
    if (pid == NULL)
    {
        return 0.0f;
    }
		//��Ź�ȥ���μ�������ֵ
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
		//�趨Ŀ��ֵ�͵�ǰֵ���ṹ���Ա
    pid->set = set;
    pid->now = now;
		//�������µ����ֵ
    pid->error[0] = set - now;
		
				
		//��������
		if( (pid->error[0] < pid->DeadZone)&&(pid->error[0]> -pid->DeadZone))
		{
			pid->error[0] = 0;
		}
		
		
		//���ֵ���㶨�ٶ�ת�����������λ��ʽPID
		//������������
		pid->pout = pid->Kp * pid->error[0];
		//������������
		pid->iout += pid->Ki * pid->error[0];
		//΢����������(��ѡ)
		//��Ź�ȥ���μ����΢�����ֵ
    pid->dcur[2] = pid->dcur[1];
    pid->dcur[1] = pid->dcur[0];
    //��ǰ����΢���ñ�������ȥ��һ�����������
    pid->dcur[0] = (pid->error[0] - pid->error[1]);
    //΢�������
    pid->dout = pid->Kd * pid->dcur[0];
		
		//�Ի���������޷�
		pid->iout = limit(pid->iout, pid->max_iout);
		//�������ֵ���
		pid->out = pid->pout + pid->iout + pid->dout;
		//����ֵ�����޷�
		pid->out = limit(pid->out, pid->max_out);
		//������ֵ
		return pid->out;
}

//����м��������
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }
	//��ǰ�������
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    //΢��������
    pid->dcur[0] = pid->dcur[1] = pid->dcur[2] = 0.0f;
    //�������
    pid->out = pid->pout = pid->iout = pid->dout = 0.0f;
    //Ŀ��ֵ�͵�ǰֵ����
    pid->now = pid->set = 0.0f;
}

//����PID���
float Cascade_PID_calc(Cascade_PID *pid,float outer_set,float outer_now,float inner_now)
{
  PID_calc(&pid->outer,outer_now,outer_set);
  PID_calc(&pid->inner,inner_now,pid->outer.out);
  pid->out = pid->inner.out;
	return pid->out;
}





