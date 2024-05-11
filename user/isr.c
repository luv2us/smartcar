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
 * �ļ�����          isr
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

#include "isr_config.h"
#include "isr.h"
float error_servo = 0;
unsigned char motor_flag=0;
int angle_servo=0;

extern volatile sint16 encoder_R;
extern volatile sint16 encoder_L;
extern int16 OUT_PWM;
float targetspeedL=60;
float targetspeedR=60;
extern sint16 MotorDuty1; // �������ռ�ձ���ֵ
extern sint16 MotorDuty2; // �������ռ�ձ���ֵ
uint8_t gyro_flag=0;
extern unsigned char Bin_Image[LCDH][LCDW];
extern uint8_t Crossroad_Flag;
// ����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ�� interrupt_global_enable(0); �������ж�Ƕ��
// �򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ������� interrupt_global_disable(); ���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ����� interrupt_global_enable(0); �������жϵ���Ӧ��
uint16_t time;
uint8_t num;
extern euler_param_t eulerAngle;
extern uint8 detect_round;
extern uint8 round_found_flag;
// **************************** PIT�жϺ��� ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH0);

    encoder_R = encoder_get_count(ENCODER_DIR);
    encoder_L = -encoder_get_count(ENCODER_QUADDEC);

    angle_servo = PD_Camera(0,error_servo);//error_servo
    if(gyro_flag)//��ȡ����������
    {
        ICM_getEulerianAngles();
    }
    pwm_set_duty(ATOM1_CH1_P33_9, SERVO_MID + angle_servo);//������
    if(motor_flag==1)
    {
        Pid_Value();
        MotorDuty1 = (sint16)PidIncCtrl(&LSpeed_PID, (float)(Target_Speed1 - encoder_L)); // �ٶȱջ�
        MotorDuty2 = (sint16)PidIncCtrl(&RSpeed_PID, (float)(Target_Speed2- encoder_R)); // �ٶȱջ�

    if (MotorDuty1 > 5000)
            MotorDuty1 = 5000;
    else if (MotorDuty1 < -5000)
            MotorDuty1 = 5000;
    if (LSpeed_PID.out > 5000)
            LSpeed_PID.out = 5000;
    else if (LSpeed_PID.out < -5000)
            LSpeed_PID.out = -5000;
    if (MotorDuty2 >5000)
            MotorDuty2 = 5000;
    else if (MotorDuty2 < -5000)
            MotorDuty2 = -5000;
    if (RSpeed_PID.out >5000)
            RSpeed_PID.out = 5000;
    else if (RSpeed_PID.out < -5000)
            RSpeed_PID.out = -5000;
    }
    else if(motor_flag==0)                          //����رգ����ٶȻ���Ϊ0����pwm���0
    {
     Pid_Value_stop();
     MotorDuty1=0;
     MotorDuty2=0;
    }
  motor_control_dir(MotorDuty2,  MotorDuty1 );

    encoder_clear_count(ENCODER_QUADDEC);           //��ձ���������
    encoder_clear_count(ENCODER_DIR);               //��ձ���������
}

IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH1);

    Crossroad_Find(UpdowmSide, ImageSide, Roadwide, &Crossroad_Flag);
        if(detect_round)
        {

        if(round_found_flag)
        {
            roundabout(Bin_Image,ImageSide,UpdowmSide,&round_found_flag);
        }
        if(round_found_flag==0)
        {
             RoadIsRoundabout(UpdowmSide,ImageSide,&round_found_flag);
        }
        }
    zebra_panduan(Bin_Image);
        if(motor_flag==2)
            {
                ImageAddingLine(ImageSide,1,(uint16)ImageSide[100][0],   100,90, 10);
                ImageAddingLine(ImageSide,2,(uint16)ImageSide[100][1],   100,90, 10);
                Pid_Value_stop();
                static int hi = 0;
                hi++;
                if(hi==90)
                    {
                        MotorDuty1=0;
                        MotorDuty2=0;
                    }
            }
    get_midline();
    Get_Errand();
}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH0);
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH1);
}
// **************************** PIT�жϺ��� ****************************

// **************************** �ⲿ�жϺ��� ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);            // �����ж�Ƕ��
    if (exti_flag_get(ERU_CH0_REQ0_P15_4)) // ͨ��0�ж�
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
    }

    if (exti_flag_get(ERU_CH4_REQ13_P15_5)) // ͨ��4�ж�
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��

    if (exti_flag_get(ERU_CH1_REQ10_P14_3)) // ͨ��1�ж�
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler(); // ToF ģ�� INT �����ж�
    }

    if (exti_flag_get(ERU_CH5_REQ1_P15_8)) // ͨ��5�ж�
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);

        wireless_module_spi_handler(); // SPI WIFI �жϻص�����
    }
}

// ��������ͷpclk����Ĭ��ռ���� 2ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
// IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // �����ж�Ƕ��
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // ͨ��2�ж�
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // ͨ��6�ж�
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }

IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);            // �����ж�Ƕ��
    if (exti_flag_get(ERU_CH3_REQ6_P02_0)) // ͨ��3�ж�
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler(); // ����ͷ�����ɼ�ͳһ�ص�����
    }
    if (exti_flag_get(ERU_CH7_REQ16_P15_1)) // ͨ��7�ж�
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
    }
}
// **************************** �ⲿ�жϺ��� ****************************

// **************************** DMA�жϺ��� ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    camera_dma_handler();       // ����ͷ�ɼ����ͳһ�ص�����
}
// **************************** DMA�жϺ��� ****************************

// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��

#if DEBUG_UART_USE_INTERRUPT   // ������� debug �����ж�
    debug_interrupr_handler(); // ���� debug ���ڽ��մ����� ���ݻᱻ debug ���λ�������ȡ
#endif                         // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
}

// ����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    camera_uart_handler();      // ����ͷ��������ͳһ�ص�����
}

// ����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);     // �����ж�Ƕ��
    wireless_module_uart_handler(); // ����ģ��ͳһ�ص�����
}
// ����3Ĭ�����ӵ�GPS��λģ��
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    gps_uart_callback();        // GPS���ڻص�����
}

// ����ͨѶ�����ж�
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0); // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}
