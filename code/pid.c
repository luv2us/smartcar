/*
 * pid.c
 *
 *  Created on: 2024��2��27��
 *      Author: �����
 */


/*
 * ljz_pid.c
 *
 *  Created on: 2024��2��3��
 *      Author: �����
 */
#include "pid.h"

/*************************************************************************
 *  �������ƣ�float constrain_float(float amt, float low, float high)
 *  ����˵�����޷�����
 *  ����˵����
  * @param    amt   �� ����
  * @param    low   �� ���ֵ
  * @param    high  �� ���ֵ
 *  �������أ���
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
// pid����ֵ�޸�
pid_param_t LSpeed_PID;
pid_param_t RSpeed_PID;
void Pid_Value(void)
{
    //�ٶ�V                  //50(/5)    50(/7)      60(/7)       70(/7)
    //���ֵ��PID����
    LSpeed_PID.kp =42;//12.5;//205;    //235       230         205             215
    LSpeed_PID.ki= 0.95;//0.4;//1.0;    //1.0       1.0         1.0             1.0
    LSpeed_PID.kd = 3.3;//0;//0.5;    //0.5       0.5         0.5             0.5
    //���ֵ��PID����
    RSpeed_PID.kp =42;// 12.5;// 205;    //235       230         205             215
    RSpeed_PID.ki =0.95;//0.4;//1.0;    //1.0       1.0         1.0             1.0
    RSpeed_PID.kd = 3.3;//0;//0.5;    //0.5       0.5         0.5             0.5
}
void Pid_Value_stop(void)
{
    //�ٶ�V                  //50(/5)    50(/7)      60(/7)       70(/7)
    //���ֵ��PID����
    LSpeed_PID.kp = 0;//205;    //235       230         205             215
    LSpeed_PID.ki =0;//1.0;    //1.0       1.0         1.0             1.0
    LSpeed_PID.kd = 0;//0.5;    //0.5       0.5         0.5             0.5
    //���ֵ��PID����
    RSpeed_PID.kp = 0;// 205;    //235       230         205             215
    RSpeed_PID.ki =0;//1.0;    //1.0       1.0         1.0             1.0
    RSpeed_PID.kd = 0;//0.5;    //0.5       0.5         0.5             0.5
}
// pid������ʼ������
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
 *  �������ƣ�float constrain_float(float amt, float low, float high)
 *  ����˵����pidλ��ʽ���������
 *  ����˵����
  * @param    pid     pid����
  * @param    error   pid�������
 *  �������أ�PID������
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
float PidLocCtrl(pid_param_t * pid, float error)
{
    /* �ۻ���� */
    pid->integrator += error;

    /* ����޷� */
    constrain_float(pid->integrator, -pid->imax, pid->imax);


    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}
/*************************************************************************
 *  �������ƣ�float constrain_float(float amt, float low, float high)
 *  ����˵����pid����ʽ���������
 *  ����˵����
  * @param    pid     pid����
  * @param    error   pid�������
 *  �������أ�PID������   ע���������Ѿ��������ϴν��
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
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



