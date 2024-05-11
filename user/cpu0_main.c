/*********************************************************************************************************************
 * TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC264 开源库的一部分
 *
 * TC264 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          cpu0_main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.9.4
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
extern uint8_t Crossroad_Flag;
extern uint8_t r_up_guaiflag;
extern uint8_t l_up_guaiflag;
extern uint8_t r_down_guaiflag;
extern uint8_t l_down_guaiflag;
extern unsigned char Image_Use[LCDH][LCDW];
extern unsigned char Bin_Image[LCDH][LCDW];
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
int i,j=0;
uint8 x1_boundary[LCDH], x2_boundary[LCDH], x3_boundary[LCDH];
uint8 y1_boundary[LCDW],y2_boundary[LCDW],y3_boundary[LCDW];
float testtunner1=0;
float testtunner2=0;
// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init(); // 获取时钟频率<务必保留>
    debug_init(); // 初始化默认调试串口  //更改至无线串口串口2
    /*****************************///初始化
    ips200_init(IPS200_TYPE_PARALLEL8); // 屏幕
    mt9v03x_init();

    PidInit(&LSpeed_PID);
    PidInit(&RSpeed_PID);


    encoder_init();
    servo_init();
    motor_init();

    pit_ms_init(CCU60_CH0, 100);
    //////无线串口与逐飞助手初始化
    wireless_uart_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    //////
    seekfree_assistant_oscilloscope_struct oscilloscope_data;
    oscilloscope_data.data[0] =0;
    oscilloscope_data.channel_num = 5;

//    //配置发送边线信息
//        seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, NULL, LCDW, LCDH);
//
//        seekfree_assistant_camera_boundary_config(Y_BOUNDARY, LCDW, NULL, NULL, NULL, y1_boundary, NULL, y3_boundary);
//    seekfree_assistant_camera_boundary_config(X_BOUNDARY, LCDH, x1_boundary, NULL, x3_boundary, NULL, NULL, NULL);
    //////
   // int counter = 0;
    // 等待所有核心初始化完毕
    cpu_wait_event_ready();
    while (TRUE)
    {

        Show_Camera_Info();//屏幕显示
 /*
         ////用于调速度环
//       i++;
//       if(i>=800){i=0;j++;}
//       if(j%4==1){Target_Speed1=180;Target_Speed2=180;}
//       if(j%4==2){Target_Speed1=250;Target_Speed2=250;}
//       if(j%4==3){Target_Speed1=300;Target_Speed2=300;}
//       if(j%4==0){Target_Speed1=480;Target_Speed2=480;}
//        seekfree_assistant_oscilloscope_send(&oscilloscope_data);//虚拟示波器显示波形
//        oscilloscope_data.data[0] =MotorDuty1;
//        oscilloscope_data.data[1] =encoder_L;
//        oscilloscope_data.data[2] =MotorDuty2;
//        oscilloscope_data.data[3] =encoder_R;
//        oscilloscope_data.data[4] =150;
        //////
*/

 /*
        ///可发送左右边线至逐飞助手
//        for (i = 0; i < LCDH; i++)
//        {
//            x1_boundary[i] = ImageSide[i][0];
//            x3_boundary[i] = ImageSide[i][1];
//        }
//                for (i = 0; i < LCDW; i++)
//                {
//                    y1_boundary[i] = UpdowmSide[0][i];
//                    y3_boundary[i] = UpdowmSide[1][i];
//                }
//        seekfree_assistant_camera_send();
        ///可用于调参//seekfree_assistant_parameter[i]均为float类型
        seekfree_assistant_data_analysis();
        for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
        {
            // 更新标志位
            if(seekfree_assistant_parameter_update_flag[i])
            {
                seekfree_assistant_parameter_update_flag[i] = 0;
                if(i%8==0){servo.kp=seekfree_assistant_parameter[i];printf("方向环kp : %f", servo.kp);printf("\r\n");}//方向环kp
                if(i%8==1){servo.kd=seekfree_assistant_parameter[i];printf("方向环kd : %f ", servo.kd);printf("\r\n");}//方向环kd
                if(i%8==2){targetspeedL=seekfree_assistant_parameter[i];targetspeedR=targetspeedL;printf("targetspeed : %f ", targetspeedL);printf("\r\n");}//
//                if(i%8==3){targetspeedR=seekfree_assistant_parameter[i];printf("targetspeedR : %f ", targetspeedR);printf("\r\n");}//
                if(i%8==4){motor_flag=seekfree_assistant_parameter[i];printf("motor : %d ", motor_flag);printf("\r\n");}//注意！0才能停

            }
        }
*/
    }
}

#pragma section all restore

// **************************** 代码区域 ****************************
