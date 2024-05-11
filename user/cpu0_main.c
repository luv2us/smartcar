/*********************************************************************************************************************
 * TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� TC264 ��Դ���һ����
 *
 * TC264 ��Դ�� ��������
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 *
 * ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
 * ����û�������������Ի��ʺ��ض���;�ı�֤
 * ����ϸ����μ� GPL
 *
 * ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
 * ���û�У������<https://www.gnu.org/licenses/>
 *
 * ����ע����
 * ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
 * �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          cpu0_main
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          ADS v1.9.4
 * ����ƽ̨          TC264D
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
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
// **************************** �������� ****************************
uint8 printtime=1;
uint8 showimageflag=1;
uint8 key_x_state[KEY_NUMBER];
uint8 pointx=0,pointy=0;

int core0_main(void)
{
    clock_init(); // ��ȡʱ��Ƶ��<��ر���>
    debug_init(); // ��ʼ��Ĭ�ϵ��Դ���  //���������ߴ��ڴ���2
    /*****************************///��ʼ��
    ips200_init(IPS200_TYPE_PARALLEL8); // ��Ļ
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
    //////���ߴ�����������ֳ�ʼ��

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    //////
    seekfree_assistant_oscilloscope_struct oscilloscope_data;
    oscilloscope_data.data[0] =0;
    oscilloscope_data.channel_num = 5;
    // �ȴ����к��ĳ�ʼ�����
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
           Show_Camera_Info();//��Ļ��ʾͼ��
        }
        //��ʾ�����ǲ�ֵ
        ips200_show_float(160,240,relative_difference,3,3);
        ips200_show_uint(160,260,seektype,2);
        ips200_show_uint(160,280,round_found_flag,2);

//��
//���Ի����Ϲյ�
//        uint8 uppointy=0,uppointx=0;
//        UpSideErr(UpdowmSide,2,1,&uppointx);
//        uppointy=UpdowmSide[0][uppointx];
// ips200_draw_line(uppointx,uppointy,uppointx,120,RGB565_BLUE);
//        ips200_show_uint(100,280,uppointy,3);
//        ips200_show_uint(130,280,uppointx,3);
//////���Ի����ж� 1ֱ���Ҷ���������10 2�¹յ� 3�йյ�
//      uint8 leftState=0,rightState=0,miss_number=0,miss_number1=0,left_down_y=0,errL=0,l_py=0;
//
//            leftState=RoadImageSide_Mono(ImageSide, 0);//��߽�
//            rightState=RoadImageSide_Mono(ImageSide, 1);//�ұ߽�
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

//��
//���Ի����Ϲյ�
//                uint8 uppointy=0,uppointx=0;
//                UpSideErr(UpdowmSide,2,1,&uppointx);
//                uppointy=UpdowmSide[0][uppointx];
//                ips200_draw_line(uppointx,uppointy,uppointx,120,RGB565_BLUE);
//                ips200_show_uint(100,280,uppointy,3);
//                ips200_show_uint(130,280,uppointx,3);
//        uint8 leftState=0,rightState=0,miss_number=0,right_down_y=0,errR=0,r_py=0;
//        leftState=RoadImageSide_Mono(ImageSide, 0);//��߽�
//        rightState=RoadImageSide_Mono(ImageSide, 1);//�ұ߽�
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







//        if(printtime)//��ӡһ��
//        {
//            for(uint8 z=LCDH-5;z>=10;z--)
//        {
//            printf("%d\n",ImageSide[0][z]);
//        }
//            printtime=0;
//        }
   //
        /*
         //���ڵ��ٶȻ�
//       i++;
//       if(i>=400){i=0;j++;}
//       if(j%4==1){Target_Speed1=0;Target_Speed2=80;}
//       if(j%4==2){Target_Speed1=0;Target_Speed2=150;}
//       if(j%4==3){Target_Speed1=0;Target_Speed2=110;}
//       if(j%4==0){Target_Speed1=0;Target_Speed2=60;}
//        seekfree_assistant_oscilloscope_send(&oscilloscope_data);//����ʾ������ʾ����
//        oscilloscope_data.data[0] =encoder_L;
//        oscilloscope_data.data[1] =encoder_R;
//        oscilloscope_data.data[2] =Target_Speed1;
//        oscilloscope_data.data[3] =Target_Speed2;
       // oscilloscope_data.data[4] =Target_Speed1;
        */
        ////
        /*
        //�������ұ������������
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
        ///�����ڵ���//seekfree_assistant_parameter[i]��Ϊfloat����
//        seekfree_assistant_data_analysis();
//        for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
//        {
//            // ���±�־λ
//            if(seekfree_assistant_parameter_update_flag[i])
//            {
//                seekfree_assistant_parameter_update_flag[i] = 0;
//                if(i%8==0){servo.kp=seekfree_assistant_parameter[i];printf("����kp : %f", servo.kp);printf("\r\n");}//����kp
//                if(i%8==1){servo.kd=seekfree_assistant_parameter[i];printf("����kd : %f ", servo.kd);printf("\r\n");}//����kd
//                if(i%8==2){Target_Speed1=seekfree_assistant_parameter[i];Target_Speed2=Target_Speed1;printf("targetspeed : %d ", Target_Speed1);printf("\r\n");}//
////                if(i%8==3){targetspeedR=seekfree_assistant_parameter[i];printf("targetspeedR : %f ", targetspeedR);printf("\r\n");}//
//                if(i%8==4){motor_flag=seekfree_assistant_parameter[i];printf("motor : %d ", motor_flag);printf("\r\n");}//ע�⣡0����ͣ
//
//            }
//        } */

    }
}

#pragma section all restore

// **************************** �������� ****************************
