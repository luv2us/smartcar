/*
 * servo_control.c
 *
 *  Created on: 2024年3月4日
 *      Author: 凌纪哲
 */

#include "servo_control.h"

volatile int16 encoder_R = 0;
volatile int16 encoder_L = 0;

sint16 MotorDuty1 = 500; // 电机驱动占空比数值
sint16 MotorDuty2 = 500; // 电机驱动占空比数值
sint16 MotorDuty_L = 0;  // 左轮总PWM
sint16 MotorDuty_R = 0;  // 右轮总PWM
pid_servo_t servo={1.085,0.085};

//float P =  1;//1.98;
//float D =  0  ;//1.632;

volatile sint16 Target_Speed1 =80; // 目标速度全局变量
volatile sint16 Target_Speed2 =80; // 目标速度全局变量
void encoder_init(void)
{
    encoder_quad_init(ENCODER_QUADDEC, ENCODER_QUADDEC_A, ENCODER_QUADDEC_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_dir_init(ENCODER_DIR, ENCODER_DIR_PULSE, ENCODER_DIR_DIR);        // 初始化编码器模块与引脚 带方向增量编码器模式
}
void encoder_get()
{
    encoder_L = encoder_get_count(ENCODER_QUADDEC); // 获取编码器计数
    encoder_R = encoder_get_count(ENCODER_DIR);     // 获取编码器计数

    encoder_clear_count(ENCODER_QUADDEC); // 清空编码器计数
    encoder_clear_count(ENCODER_DIR);     // 清空编码器计数
}
//void servo_set(int angle)
//{
//    if (angle >= 125) // 限幅处理
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

int PD_Camera(float expect_val, float err) // 舵机PD调节
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
