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
 * �ļ�����          cpu1_main
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
#pragma section all "cpu1_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��

extern unsigned char Image_Use[LCDH][LCDW];
extern unsigned char Bin_Image[LCDH][LCDW];
int a,b;
// **************************** �������� ****************************
/*
core1������������ͷ���ݣ��ݲ��ڴ���ʾͼ��
*/

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
#define use8neighbor 1
void core1_main(void)
{
    disable_Watchdog();         // �رտ��Ź�
    interrupt_global_enable(0); // ��ȫ���ж�
    // ��ʼ��
    system_start();
    pit_init(CCU60_CH1, 10000); // ����ͷԪ�ش���10ms��ʱ�ж�
    cpu_wait_event_ready();     // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {

        if (mt9v03x_finish_flag)
        {

//            a = system_getval();
            Get_Use_Image();
            Get_Bin_Image(1);
            //Bin_Image_Filter();
            image_filter();
//            b = system_getval();
            imageside(2);
         //   ImageGetSide(Bin_Image, ImageSide, ImageSide_last,1); //
            UpdownSideGet(Bin_Image, UpdowmSide); //
            GetRoadWide(ImageSide, Roadwide); // �������
            mt9v03x_finish_flag = 0;
        }
    }
}
#pragma section all restore
// **************************** �������� ****************************
