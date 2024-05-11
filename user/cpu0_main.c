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
extern uint8_t Crossroad_Flag;
extern uint8_t r_up_guaiflag;
extern uint8_t l_up_guaiflag;
extern uint8_t r_down_guaiflag;
extern uint8_t l_down_guaiflag;
extern unsigned char Image_Use[LCDH][LCDW];
extern unsigned char Bin_Image[LCDH][LCDW];
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
int i,j=0;
uint8 x1_boundary[LCDH], x2_boundary[LCDH], x3_boundary[LCDH];
uint8 y1_boundary[LCDW],y2_boundary[LCDW],y3_boundary[LCDW];
float testtunner1=0;
float testtunner2=0;
// **************************** �������� ****************************
int core0_main(void)
{
    clock_init(); // ��ȡʱ��Ƶ��<��ر���>
    debug_init(); // ��ʼ��Ĭ�ϵ��Դ���  //���������ߴ��ڴ���2
    /*****************************///��ʼ��
    ips200_init(IPS200_TYPE_PARALLEL8); // ��Ļ
    mt9v03x_init();

    PidInit(&LSpeed_PID);
    PidInit(&RSpeed_PID);


    encoder_init();
    servo_init();
    motor_init();

    pit_ms_init(CCU60_CH0, 100);
    //////���ߴ�����������ֳ�ʼ��
    wireless_uart_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    //////
    seekfree_assistant_oscilloscope_struct oscilloscope_data;
    oscilloscope_data.data[0] =0;
    oscilloscope_data.channel_num = 5;

//    //���÷��ͱ�����Ϣ
//        seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, NULL, LCDW, LCDH);
//
//        seekfree_assistant_camera_boundary_config(Y_BOUNDARY, LCDW, NULL, NULL, NULL, y1_boundary, NULL, y3_boundary);
//    seekfree_assistant_camera_boundary_config(X_BOUNDARY, LCDH, x1_boundary, NULL, x3_boundary, NULL, NULL, NULL);
    //////
   // int counter = 0;
    // �ȴ����к��ĳ�ʼ�����
    cpu_wait_event_ready();
    while (TRUE)
    {

        Show_Camera_Info();//��Ļ��ʾ
 /*
         ////���ڵ��ٶȻ�
//       i++;
//       if(i>=800){i=0;j++;}
//       if(j%4==1){Target_Speed1=180;Target_Speed2=180;}
//       if(j%4==2){Target_Speed1=250;Target_Speed2=250;}
//       if(j%4==3){Target_Speed1=300;Target_Speed2=300;}
//       if(j%4==0){Target_Speed1=480;Target_Speed2=480;}
//        seekfree_assistant_oscilloscope_send(&oscilloscope_data);//����ʾ������ʾ����
//        oscilloscope_data.data[0] =MotorDuty1;
//        oscilloscope_data.data[1] =encoder_L;
//        oscilloscope_data.data[2] =MotorDuty2;
//        oscilloscope_data.data[3] =encoder_R;
//        oscilloscope_data.data[4] =150;
        //////
*/

 /*
        ///�ɷ������ұ������������
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
        ///�����ڵ���//seekfree_assistant_parameter[i]��Ϊfloat����
        seekfree_assistant_data_analysis();
        for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
        {
            // ���±�־λ
            if(seekfree_assistant_parameter_update_flag[i])
            {
                seekfree_assistant_parameter_update_flag[i] = 0;
                if(i%8==0){servo.kp=seekfree_assistant_parameter[i];printf("����kp : %f", servo.kp);printf("\r\n");}//����kp
                if(i%8==1){servo.kd=seekfree_assistant_parameter[i];printf("����kd : %f ", servo.kd);printf("\r\n");}//����kd
                if(i%8==2){targetspeedL=seekfree_assistant_parameter[i];targetspeedR=targetspeedL;printf("targetspeed : %f ", targetspeedL);printf("\r\n");}//
//                if(i%8==3){targetspeedR=seekfree_assistant_parameter[i];printf("targetspeedR : %f ", targetspeedR);printf("\r\n");}//
                if(i%8==4){motor_flag=seekfree_assistant_parameter[i];printf("motor : %d ", motor_flag);printf("\r\n");}//ע�⣡0����ͣ

            }
        }
*/
    }
}

#pragma section all restore

// **************************** �������� ****************************
