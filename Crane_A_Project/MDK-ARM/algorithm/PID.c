//PID相关函数定义文件
#include "main.h"
#include "PID.h"
//限幅用函数
float limit(int now, int setmax)
{ 
	if(now < - setmax) return -setmax;
	if(now > setmax) return setmax;
	else
			return now;
}

//PID初始化函数
void PID_init(pid_type_def *pid, const float PID[3],float deadzone, float max_out, float max_iout)
{
		//判断是否有PID控制电机需要初始化
    if (pid == NULL || PID == NULL)
    {
        return;
    }
		//传值给相关系数进行初始化
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
		//初始化最大值
    pid->max_out = max_out;
    pid->max_iout = max_iout;
		
		pid->DeadZone = deadzone;
		
		//初始化现值与误差值
    pid->dcur[0] = pid->dcur[1] = pid->dcur[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->pout = pid->iout = pid->dout = pid->out = 0.0f;
}


//PID运算函数(位置)
float PID_calc(pid_type_def *pid, float now, float set)
{
    //判断传入的PID指针不为空
    if (pid == NULL)
    {
        return 0.0f;
    }
		//存放过去两次计算的误差值
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
		//设定目标值和当前值到结构体成员
    pid->set = set;
    pid->now = now;
		//计算最新的误差值
    pid->error[0] = set - now;
		
				
		//设置死区
		if( (pid->error[0] < pid->DeadZone)&&(pid->error[0]> -pid->DeadZone))
		{
			pid->error[0] = 0;
		}
		
		
		//保持电机恒定速度转动，这里采用位置式PID
		//比例项计算输出
		pid->pout = pid->Kp * pid->error[0];
		//积分项计算输出
		pid->iout += pid->Ki * pid->error[0];
		//微分项计算输出(可选)
		//存放过去两次计算的微分误差值
    pid->dcur[2] = pid->dcur[1];
    pid->dcur[1] = pid->dcur[0];
    //当前误差的微分用本次误差减去上一次误差来计算
    pid->dcur[0] = (pid->error[0] - pid->error[1]);
    //微分项输出
    pid->dout = pid->Kd * pid->dcur[0];
		
		//对积分项进行限幅
		pid->iout = limit(pid->iout, pid->max_iout);
		//对总输出值求和
		pid->out = pid->pout + pid->iout + pid->dout;
		//对总值进行限幅
		pid->out = limit(pid->out, pid->max_out);
		//返回总值
		return pid->out;
}

//清空中间变量函数
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }
	//当前误差清零
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    //微分项清零
    pid->dcur[0] = pid->dcur[1] = pid->dcur[2] = 0.0f;
    //输出清零
    pid->out = pid->pout = pid->iout = pid->dout = 0.0f;
    //目标值和当前值清零
    pid->now = pid->set = 0.0f;
}

//串级PID输出
float Cascade_PID_calc(Cascade_PID *pid,float outer_set,float outer_now,float inner_now)
{
  PID_calc(&pid->outer,outer_now,outer_set);
  PID_calc(&pid->inner,inner_now,pid->outer.out);
  pid->out = pid->inner.out;
	return pid->out;
}





