/*
 * servo_pid.c
 *
 *  Created on: 2024年3月2日
 *      Author: 凌纪哲
 */
#include "servo_pid.h"


int16 OUT_PWM=0;       //转向输出PWM                                                                       90                 95       93       80
float Angle_kp=31.3;    //转向环PID（外环）32.4     31.5    32.1     31.9   31.7    31.3    32.2    30.5    29.5    29.5        30       31.2      31.3
float value = 3;     //循迹P的变化大小                                              5       4     3.5       3     2.4         2.3        3         3
float Angle_ki=0;
float Angle_kd=0;
float Gro3_p = 0.3;   //陀螺仪的乘积系数//0.19     0.75    0.75    0.585 0.545    0.445   0.415     0.4    0.31    0.15       0.21      0.35       0.3
uint8_t Number=0;       //内外环计数器
float Angle=0;          //目标角度
float Angle_last=0;
int ServoDuty_Last=0;
int Servoduty=0;        //外环偏差


void AngleControl(void)
{
//  Servoduty = (int)ServoDuty; //传递偏差
////三次内环，执行一次内环
//  if(Number==3){
//    Number=0;
//    Angle_Get();
//  }
//  Number++;
//  Angle=Angle_last+(Angle-Angle_last)*Number/3;
  Angle_out();
}


void Angle_out(void)
{
  float error;
  static  float last_error;
  float price_Differention;
  float price_Proportion;
  static float Pre1_Error[4];
  float Direct_Parameter;
//  if(gyro[2] > 2500) gyro[2] = 2500;
//  if(gyro[2] < -2500) gyro[2] = -2500;
  error = Angle*8.0 ;//+ gyro[2]*Gro3_p;  //转向内环参数，一般不用改，改变后面的参数，改变阻尼大小
  price_Proportion=0.35 * error;
  price_Differention= 0.35 * (error - last_error);

  Direct_Parameter =price_Proportion  + price_Differention ;//

  last_error =  error;
  Pre1_Error[3]=Pre1_Error[2];
  Pre1_Error[2]=Pre1_Error[1];
  Pre1_Error[1]=Pre1_Error[0];
  Pre1_Error[0]=Direct_Parameter;
  Direct_Parameter=Pre1_Error[0]*0.8+Pre1_Error[1]*0.1+Pre1_Error[2]*0.06+Pre1_Error[3]*0.04 ;
  OUT_PWM=(sint16)(Direct_Parameter);
}
