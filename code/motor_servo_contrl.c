/*
 * motor_servo_cotrl.c
 *
 *  Created on: 2024��3��1��
 *      Author: �����
 */
#include "motor_servo_contrl.h"

void motor_control_dir(sint32 motorR,sint32 motorL)//�����ֿ���
{

if(motorR>=0)
    {
        gpio_set_level(P02_6,1);           //������ת
        pwm_set_duty(ATOM1_CH5_P02_5,motorR);
    }
else
    {
        gpio_set_level(P02_6,0);           //���ַ�ת
        pwm_set_duty(ATOM1_CH5_P02_5,-motorR);
    }
if(motorL>=0)
    {
        gpio_set_level(P02_4,1);           //������ת
        pwm_set_duty(ATOM1_CH7_P02_7,motorL);
    }
else
    {
        gpio_set_level(P02_4,0);           //���ַ�ת
        pwm_set_duty(ATOM1_CH7_P02_7,-motorL);
    }
}

void motor_init(void)    //�����ʼ��
{
pwm_init(ATOM1_CH5_P02_5,17000,0);
pwm_init(ATOM1_CH7_P02_7,17000,0);

gpio_init(P02_4,GPO,0,GPO_PUSH_PULL);
gpio_init(P02_6,GPO,0,GPO_PUSH_PULL);


}
