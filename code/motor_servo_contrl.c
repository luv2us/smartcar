/*
 * motor_servo_cotrl.c
 *
 *  Created on: 2024年3月1日
 *      Author: 凌纪哲
 */
#include "motor_servo_contrl.h"

void motor_control_dir(sint32 motorR,sint32 motorL)//差速轮控制
{

if(motorR>=0)
    {
        gpio_set_level(P02_6,1);           //右轮正转
        pwm_set_duty(ATOM1_CH5_P02_5,motorR);
    }
else
    {
        gpio_set_level(P02_6,0);           //右轮反转
        pwm_set_duty(ATOM1_CH5_P02_5,-motorR);
    }
if(motorL>=0)
    {
        gpio_set_level(P02_4,1);           //左轮正转
        pwm_set_duty(ATOM1_CH7_P02_7,motorL);
    }
else
    {
        gpio_set_level(P02_4,0);           //左轮反转
        pwm_set_duty(ATOM1_CH7_P02_7,-motorL);
    }
}

void motor_init(void)    //电机初始化
{
pwm_init(ATOM1_CH5_P02_5,17000,0);
pwm_init(ATOM1_CH7_P02_7,17000,0);

gpio_init(P02_4,GPO,0,GPO_PUSH_PULL);
gpio_init(P02_6,GPO,0,GPO_PUSH_PULL);


}
