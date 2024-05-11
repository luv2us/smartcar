/*
 * servo_pid.h
 *
 *  Created on: 2024��3��2��
 *      Author: �����
 */

#ifndef APP_SERVO_PID_H_
#define APP_SERVO_PID_H_

#include "zf_common_headfile.h"


extern float Angle_kp;    //ת��PID���⻷��20.5
extern float Angle_ki;
extern float Angle_kd;
extern float value;

void Angle_out(void);
void AngleControl(void);
#endif /* APP_SERVO_PID_H_ */
