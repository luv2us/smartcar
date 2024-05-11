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

// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
int i,j=0;
uint8 x1_boundary[LCDH], x2_boundary[LCDH], x3_boundary[LCDH];
uint8 y1_boundary[LCDW],y2_boundary[LCDW],y3_boundary[LCDW];
float testtunner1=0;
float testtunner2=0;
//extern float previous_yaw;
extern float relative_difference;
extern uint8 detect_round;
extern unsigned char motor_flag;
//extern float error_servo;
//extern unsigned char Bin_Image[LCDH][LCDW];
extern uint8 round_found_flag;
// **************************** 代码区域 ****************************
uint8 printtime=1;
uint8 showimageflag=1;
uint8 key_x_state[KEY_NUMBER];
uint8 pointx=0,pointy=0;

int core0_main(void)
{
    clock_init(); // 获取时钟频率<务必保留>
    debug_init(); // 初始化默认调试串口  //更改至无线串口串口2
    /*****************************///初始化
    ips200_init(IPS200_TYPE_PARALLEL8); // 屏幕
    mt9v03x_init();

    PidInit(&LSpeed_PID);
    PidInit(&RSpeed_PID);

    imu660ra_init();
    encoder_init();
    servo_init();
    motor_init();
    key_init(10);
    wireless_uart_init();
    pit_ms_init(CCU60_CH0, 5);
    //////无线串口与逐飞助手初始化

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    //////
    seekfree_assistant_oscilloscope_struct oscilloscope_data;
    oscilloscope_data.data[0] =0;
    oscilloscope_data.channel_num = 5;
    // 等待所有核心初始化完毕
    cpu_wait_event_ready();
    ips200_show_string(110,240,"gyro");
    ips200_show_string(110,260,"track");
    ips200_show_string(110,280,"circle");
    while (TRUE)
    {

        key_scanner();
        key_x_state[0]=key_get_state(KEY_1);
        key_x_state[1]=key_get_state(KEY_2);
            if(key_x_state[0]!=0)
            {
                if(motor_flag==1)
                {
                      showimageflag=1;
                      motor_flag=0;
                      ips200_clear();
                }
                else
                {
                    showimageflag=0;
                    motor_flag=1;
                ips200_clear();
                }

                key_clear_state(KEY_1);
            }
            if(key_x_state[1]!=0)
            {
                if(detect_round==0)
                      {
                        detect_round=1;
                      }
                else
                {
                    detect_round=0;
                }
                key_clear_state(KEY_2);
            }
        if(showimageflag)
        {
           Show_Camera_Info();//屏幕显示图像
        }
        //显示陀螺仪差值
        ips200_show_float(160,240,relative_difference,3,3);
        ips200_show_uint(160,260,seektype,2);
        ips200_show_uint(160,280,round_found_flag,2);

//左
//测试环岛上拐点
//        uint8 uppointy=0,uppointx=0;
//        UpSideErr(UpdowmSide,2,1,&uppointx);
//        uppointy=UpdowmSide[0][uppointx];
// ips200_draw_line(uppointx,uppointy,uppointx,120,RGB565_BLUE);
//        ips200_show_uint(100,280,uppointy,3);
//        ips200_show_uint(130,280,uppointx,3);
//////测试环岛判断 1直线且丢线行少于10 2下拐点 3中拐点
//      uint8 leftState=0,rightState=0,miss_number=0,miss_number1=0,left_down_y=0,errL=0,l_py=0;
//
//            leftState=RoadImageSide_Mono(ImageSide, 0);//左边界
//            rightState=RoadImageSide_Mono(ImageSide, 1);//右边界
//
//            ips200_show_char(40,130,'r');
//            ips200_show_uint(50,130,rightState,2);
//
//                miss_number=line_miss(ImageSide, 0,30);
//                miss_number1=line_miss(ImageSide, 1,30);
////                ips200_show_char(40,150,'m');
//                //ips200_show_uint(50,150,miss_number1,3);
//                ips200_show_uint(80,150,miss_number,3);
//
//                left_down_y=(uint8)Find_Down_Point(1);
//                //ips200_draw_line(ImageSide[left_down_y][0],left_down_y,160,left_down_y,RGB565_BLUE);
//
//                errL = RoundaboutGetArc(ImageSide, 1, 5, &l_py,left_down_y-5);
//                //ips200_draw_line(ImageSide[l_py][0],l_py,160,l_py,RGB565_GREEN);
//
//                ips200_show_char(40,170,'d');
//                ips200_show_uint(50,170,left_down_y,3);
//
//                ips200_show_char(40,190,'e');
//                            ips200_show_uint(50,190,l_py,3);
//                            ips200_show_uint(70,190,errL,3);

//右
//测试环岛上拐点
//                uint8 uppointy=0,uppointx=0;
//                UpSideErr(UpdowmSide,2,1,&uppointx);
//                uppointy=UpdowmSide[0][uppointx];
//                ips200_draw_line(uppointx,uppointy,uppointx,120,RGB565_BLUE);
//                ips200_show_uint(100,280,uppointy,3);
//                ips200_show_uint(130,280,uppointx,3);
//        uint8 leftState=0,rightState=0,miss_number=0,right_down_y=0,errR=0,r_py=0;
//        leftState=RoadImageSide_Mono(ImageSide, 0);//左边界
//        rightState=RoadImageSide_Mono(ImageSide, 1);//右边界
//        ips200_show_char(40,130,'l');
//        ips200_show_uint(50,130,leftState,2);
//        miss_number=line_miss(ImageSide, 0,30);
//        ips200_show_char(40,150,'m');
//        ips200_show_uint(50,150,miss_number,3);
//        right_down_y=(uint8)Find_Down_Point(2);
//       // ips200_draw_line(ImageSide[right_down_y][0],right_down_y,160,right_down_y,RGB565_BLUE);
//        errR = RoundaboutGetArc(ImageSide, 2, 5, &r_py,right_down_y-5);
//        //ips200_draw_line(ImageSide[r_py][1],r_py,160,r_py,RGB565_GREEN);
//        ips200_show_char(40,170,'d');
//        ips200_show_uint(50,170,right_down_y,3);
//        ips200_show_char(40,190,'e');
//        ips200_show_uint(50,190,r_py,3);
//        ips200_show_uint(70,190,errR,3);







//        if(printtime)//打印一次
//        {
//            for(uint8 z=LCDH-5;z>=10;z--)
//        {
//            printf("%d\n",ImageSide[0][z]);
//        }
//            printtime=0;
//        }
   //
        /*
         //用于调速度环
//       i++;
//       if(i>=400){i=0;j++;}
//       if(j%4==1){Target_Speed1=0;Target_Speed2=80;}
//       if(j%4==2){Target_Speed1=0;Target_Speed2=150;}
//       if(j%4==3){Target_Speed1=0;Target_Speed2=110;}
//       if(j%4==0){Target_Speed1=0;Target_Speed2=60;}
//        seekfree_assistant_oscilloscope_send(&oscilloscope_data);//虚拟示波器显示波形
//        oscilloscope_data.data[0] =encoder_L;
//        oscilloscope_data.data[1] =encoder_R;
//        oscilloscope_data.data[2] =Target_Speed1;
//        oscilloscope_data.data[3] =Target_Speed2;
       // oscilloscope_data.data[4] =Target_Speed1;
        */
        ////
        /*
        //发送左右边线至逐飞助手
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
*/

        /*
        ///可用于调参//seekfree_assistant_parameter[i]均为float类型
//        seekfree_assistant_data_analysis();
//        for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
//        {
//            // 更新标志位
//            if(seekfree_assistant_parameter_update_flag[i])
//            {
//                seekfree_assistant_parameter_update_flag[i] = 0;
//                if(i%8==0){servo.kp=seekfree_assistant_parameter[i];printf("方向环kp : %f", servo.kp);printf("\r\n");}//方向环kp
//                if(i%8==1){servo.kd=seekfree_assistant_parameter[i];printf("方向环kd : %f ", servo.kd);printf("\r\n");}//方向环kd
//                if(i%8==2){Target_Speed1=seekfree_assistant_parameter[i];Target_Speed2=Target_Speed1;printf("targetspeed : %d ", Target_Speed1);printf("\r\n");}//
////                if(i%8==3){targetspeedR=seekfree_assistant_parameter[i];printf("targetspeedR : %f ", targetspeedR);printf("\r\n");}//
//                if(i%8==4){motor_flag=seekfree_assistant_parameter[i];printf("motor : %d ", motor_flag);printf("\r\n");}//注意！0才能停
//
//            }
//        } */

    }
}

#pragma section all restore

// **************************** 代码区域 ****************************
