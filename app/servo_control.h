/*
 * servo_control.h
 *
 *  Created on: 2024年3月4日
 *      Author: 凌纪哲
 */

#ifndef APP_SERVO_CONTROL_H_
#define APP_SERVO_CONTROL_H_

#include "zf_common_headfile.h"

#define SERVO_MID 675

void servo_set(int angle);
void servo_init(void);
void encoder_init(void);
void encoder_get();
int PD_Camera(float expect_val, float err);


#endif /* APP_SERVO_CONTROL_H_ */
