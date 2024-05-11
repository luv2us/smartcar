/*
 * pid.h
 *
 *  Created on: 2024年2月27日
 *      Author: 凌纪哲
 */

#ifndef APP_PID_H_
#define APP_PID_H_


#include "zf_common_headfile.h"

typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D
    float                imax;       //积分限幅

    float                out_p;  //KP输出
    float                out_i;  //KI输出
    float                out_d;  //KD输出
    float                out;    //pid输出

    float                integrator; //< 积分值
    float                last_error; //< 上次误差
    float                last_derivative;//< 上次误差与上上次误差之差
    unsigned long        last_t;     //< 上次时间
}pid_param_t;



extern pid_param_t LSpeed_PID;
extern pid_param_t RSpeed_PID;




void PidInit(pid_param_t * pid);

float constrain_float(float amt, float low, float high);

float PidLocCtrl(pid_param_t * pid, float error);

float PidIncCtrl(pid_param_t * pid, float error);

void Pid_Value(void);

void Pid_Value_stop(void);

#endif /* APP_PID_H_ */
