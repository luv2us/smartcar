/*
 * servo_control.c
 *
 *  Created on: 2024��3��4��
 *      Author: �����
 */

#include "servo_control.h"

volatile int16 encoder_R = 0;
volatile int16 encoder_L = 0;

sint16 MotorDuty1 = 500; // �������ռ�ձ���ֵ
sint16 MotorDuty2 = 500; // �������ռ�ձ���ֵ
sint16 MotorDuty_L = 0;  // ������PWM
sint16 MotorDuty_R = 0;  // ������PWM
pid_servo_t servo={1.085,0.085};

//float P =  1;//1.98;
//float D =  0  ;//1.632;

volatile sint16 Target_Speed1 =80; // Ŀ���ٶ�ȫ�ֱ���
volatile sint16 Target_Speed2 =80; // Ŀ���ٶ�ȫ�ֱ���
void encoder_init(void)
{
    encoder_quad_init(ENCODER_QUADDEC, ENCODER_QUADDEC_A, ENCODER_QUADDEC_B); // ��ʼ��������ģ�������� �������������ģʽ
    encoder_dir_init(ENCODER_DIR, ENCODER_DIR_PULSE, ENCODER_DIR_DIR);        // ��ʼ��������ģ�������� ����������������ģʽ
}
void encoder_get()
{
    encoder_L = encoder_get_count(ENCODER_QUADDEC); // ��ȡ����������
    encoder_R = encoder_get_count(ENCODER_DIR);     // ��ȡ����������

    encoder_clear_count(ENCODER_QUADDEC); // ��ձ���������
    encoder_clear_count(ENCODER_DIR);     // ��ձ���������
}
//void servo_set(int angle)
//{
//    if (angle >= 125) // �޷�����
//        angle = 125;
//    else if (angle <= -125)
//        angle = -125;
//
//    pwm_set_duty(ATOM1_CH1_P33_9, SERVO_MID + angle);
//}

void servo_init(void)
{
    pwm_init(ATOM1_CH1_P33_9, 50, SERVO_MID);
}

int PD_Camera(float expect_val, float err) // ���PD����
{
    float u;

    volatile static float error_current, error_last;
    float ek, ek1;
    error_current = err - expect_val;
    ek = error_current;
    ek1 = error_current - error_last;
    u = servo.kp * ek + servo.kd * ek1;
    error_last = error_current;

    return (int)u;
}
