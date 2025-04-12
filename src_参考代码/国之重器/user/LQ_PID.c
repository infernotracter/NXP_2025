/*!
  * @file     LQ_PID.c
  *
  * @brief    PID控制
  *
  * @company  北京龙邱智能科技
  *
  * @author   LQ-005
  *
  * @note     Tab键 4个空格
  *
  * @version  V1.0
  *
  * @par URL  http://shop36265907.taobao.com
  *           http://www.lqist.cn
  *
  * @date     2020年6月5日
  */ 

	

#include "LQ_PID.h"
#include "include.h"
int speed_jifen;
float speed_p=12,speed_i=0.05;  //速度pid参数     12  0.05
float dir_p=160,dir_d=80;     //方向外环pid参数     12  0.12(纯陀螺仪参数）    
float dirn_p=0.28,dirn_d=0.08;       //方向内环pid参数      0.28 0.08

/*!
  * @brief    限幅函数
  *
  * @param    amt   ： 参数
  * @param    low   ： 最低值
  * @param    high  ： 最高值
  *
  * @return   无
  *
  * @note     无
  *
  * @see      无
  *
  * @date     2020/6/8
  */
float constrain_float(float amt, float low, float high)
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}




/*!
  * @brief    pid参数初始化函数
  *
  * @param    无
  *
  * @return   无
  *
  * @note     无
  *
  * @see      无
  *
  * @date     2020/6/8
  */
void PidInit(pid_param_t * pid)
{
	pid->kp        = 0;
	pid->ki        = 0;
	pid->kd        = 0;
	pid->imax      = 0;
	pid->out_p     = 0;
	pid->out_i     = 0;
	pid->out_d     = 0;
	pid->out       = 0;
	pid->integrator= 0;
	pid->last_error= 0;
	pid->last_derivative   = 0;
	pid->last_t    = 0;
}


/*!
  * @brief    pid位置式控制器输出
  *
  * @param    pid     pid参数
  * @param    error   pid输入误差
  *
  * @return   PID输出结果
  *
  * @note     无
  *
  * @see      无
  *
  * @date     2020/6/8
  */
float PidLocCtrl(pid_param_t * pid, float error)
{
	/* 累积误差 */
	pid->integrator += error;

	/* 误差限幅 */
	constrain_float(pid->integrator, -pid->imax, pid->imax);


	pid->out_p = pid->kp * error;
	pid->out_i = pid->ki * pid->integrator;
	pid->out_d = pid->kd * (error - pid->last_error);

	pid->last_error = error;

	pid->out = pid->out_p + pid->out_i + pid->out_d;

	return pid->out;
}


/*!
  * @brief    pid增量式控制器输出
  *
  * @param    pid     pid参数
  * @param    error   pid输入误差
  *
  * @return   PID输出结果   注意输出结果已经包涵了上次结果
  *
  * @note     无
  *
  * @see      无
  *
  * @date     2020/6/8
  */
float PidIncCtrl(pid_param_t * pid, float error)
{

	pid->out_p = pid->kp * (error - pid->last_error);
	pid->out_i = pid->ki * error;
	pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);

	pid->last_derivative = error - pid->last_error;
	pid->last_error = error;

	pid->out += pid->out_p + pid->out_i + pid->out_d;
	return pid->out;
}

int speedcontrol(int ECPULSE,int aim_speed)
{
   int out,Error;
	Error=aim_speed-ECPULSE;
	speed_jifen+=Error;
	if(speed_jifen>300)speed_jifen=300;
	if(speed_jifen<-300)speed_jifen=-300;
out=(int)(Error*speed_p+speed_jifen*speed_i);
return out;
}
int dircontrol(int chazhi)
{
	int out_dw;
	static int last_chazhi;
	out_dw=(int)(chazhi*dir_p+(chazhi-last_chazhi)*dir_d);
	last_chazhi=chazhi;
  return out_dw;
}
int diranglecontrol(int out_dw)
{
	int out_dn,error;
	static int last_error;
	error=(out_dw-gyroz1);
	out_dn=(int)(error*dirn_p+(error-last_error)*dirn_d);
	last_error=error;
	return out_dn;
}

//int dircontrol(int chazhi)
//{
//	int out;
//	static int last_chazhi;
//	out=(int)(chazhi*dir_p-gyroz1*dir_d);
//	last_chazhi=chazhi;
//	return out;
//}
	
	
	
	
	
	
	


