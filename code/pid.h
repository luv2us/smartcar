/*
 * pid.h
 *
 *  Created on: 2024��2��27��
 *      Author: �����
 */

#ifndef APP_PID_H_
#define APP_PID_H_


#include "zf_common_headfile.h"

typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D
    float                imax;       //�����޷�

    float                out_p;  //KP���
    float                out_i;  //KI���
    float                out_d;  //KD���
    float                out;    //pid���

    float                integrator; //< ����ֵ
    float                last_error; //< �ϴ����
    float                last_derivative;//< �ϴ���������ϴ����֮��
    unsigned long        last_t;     //< �ϴ�ʱ��
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
