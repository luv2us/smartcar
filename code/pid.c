/*
 * pid.c
 *
 *  Created on: 2024年2月27日
 *      Author: 凌纪哲
 */


/*
 * ljz_pid.c
 *
 *  Created on: 2024年2月3日
 *      Author: 凌纪哲
 */
#include "pid.h"

/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：限幅函数
 *  参数说明：
  * @param    amt   ： 参数
  * @param    low   ： 最低值
  * @param    high  ： 最高值
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
// pid参数值修改
pid_param_t LSpeed_PID;
pid_param_t RSpeed_PID;
void Pid_Value(void)
{
    //速度V                  //50(/5)    50(/7)      60(/7)       70(/7)
    //左轮电机PID参数
    LSpeed_PID.kp =42;//12.5;//205;    //235       230         205             215
    LSpeed_PID.ki= 0.95;//0.4;//1.0;    //1.0       1.0         1.0             1.0
    LSpeed_PID.kd = 3.3;//0;//0.5;    //0.5       0.5         0.5             0.5
    //右轮电机PID参数
    RSpeed_PID.kp =42;// 12.5;// 205;    //235       230         205             215
    RSpeed_PID.ki =0.95;//0.4;//1.0;    //1.0       1.0         1.0             1.0
    RSpeed_PID.kd = 3.3;//0;//0.5;    //0.5       0.5         0.5             0.5
}
void Pid_Value_stop(void)
{
    //速度V                  //50(/5)    50(/7)      60(/7)       70(/7)
    //左轮电机PID参数
    LSpeed_PID.kp = 0;//205;    //235       230         205             215
    LSpeed_PID.ki =0;//1.0;    //1.0       1.0         1.0             1.0
    LSpeed_PID.kd = 0;//0.5;    //0.5       0.5         0.5             0.5
    //右轮电机PID参数
    RSpeed_PID.kp = 0;// 205;    //235       230         205             215
    RSpeed_PID.ki =0;//1.0;    //1.0       1.0         1.0             1.0
    RSpeed_PID.kd = 0;//0.5;    //0.5       0.5         0.5             0.5
}
// pid参数初始化函数
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

/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：pid位置式控制器输出
 *  参数说明：
  * @param    pid     pid参数
  * @param    error   pid输入误差
 *  函数返回：PID输出结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
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
/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：pid增量式控制器输出
 *  参数说明：
  * @param    pid     pid参数
  * @param    error   pid输入误差
 *  函数返回：PID输出结果   注意输出结果已经包涵了上次结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
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



