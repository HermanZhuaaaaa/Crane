//本头文件定义了PID算法所需中间变量以及函数及其用法(用法解释待补充)
#ifndef PID_H
#define PID_H
//定义PID相关数据结构体
typedef struct{
	//PID相关系数
	float Kp;
	float Ki;
	float Kd;
	
	//预设最大输出
	float max_out; //总输出
	float max_iout; //积分输出
	
	//达到目标值，当前值
	float set;
	float now;
	
	float DeadZone;
	float lastout;
	
	//输出值
	float out;		//三项总输出
  float pout;	//比例项输出
  float iout;	//积分项输出
  float dout;	//微分项输出
	
	//微分项最近三个值0最新1上一次2上上次
  float dcur[3];  
  //误差项最近三个值0最新1上一次2上上次
  float error[3];  
} pid_type_def;


typedef struct
{
	pid_type_def inner;
	pid_type_def outer;
	float out;
}Cascade_PID;

//前置限幅函数
extern float limit(int now, int setmax);

//PID初始化函数
extern void PID_init(pid_type_def *pid, const float PID[3],float deadzone, float max_out, float max_iout);

//PID运算函数(位置)
extern float PID_calc(pid_type_def *pid, float now, float set);

//清空中间变量函数
extern void PID_clear(pid_type_def *pid);

//计算串级PID
extern float Cascade_PID_calc(Cascade_PID *pid,float outer_set,float outer_now,float inner_now);

#endif
