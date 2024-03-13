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

// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init(); // 获取时钟频率<务必保留>
    debug_init(); // 初始化默认调试串口
    /*****************************///初始化
    ips200_init(IPS200_TYPE_PARALLEL8); // 屏幕初始化
    mt9v03x_init();

    PidInit(&LSpeed_PID);
    PidInit(&RSpeed_PID);
    Pid_Value();

    encoder_init();
    servo_init();
    motor_init();

    pit_ms_init(CCU60_CH0, 5);

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_oscilloscope_struct oscilloscope_data;
    oscilloscope_data.data[0] =0;
    oscilloscope_data.channel_num = 2;
    // 等待所有核心初始化完毕
    cpu_wait_event_ready();
    while (TRUE)
    {
        Show_Camera_Info();

        MotorDuty1 = (sint32)PidIncCtrl(&LSpeed_PID, (float)(500 - encoder_L)); // 速度闭环
        MotorDuty2 = (sint32)PidIncCtrl(&RSpeed_PID, (float)(500 - encoder_R)); // 速度闭环

        seekfree_assistant_oscilloscope_send(&oscilloscope_data);

        if (MotorDuty1 > 9999)
                MotorDuty1 = 9999;
        else if (MotorDuty1 < -9999)
                MotorDuty1 = 9999;
        if (LSpeed_PID.out > 9999)
            LSpeed_PID.out = 9999;
        else if (LSpeed_PID.out < -9999)
            LSpeed_PID.out = -9999;


        if (MotorDuty2 > 9999)
            MotorDuty2 = 9999;
        else if (MotorDuty2 < -9999)
            MotorDuty2 = -9999;
        if (RSpeed_PID.out > 9999)
            RSpeed_PID.out = 9999;
        else if (RSpeed_PID.out < -9999)
            RSpeed_PID.out = -9999;

        motor_control_dir(MotorDuty2,  MotorDuty1 );
        system_delay_ms(5);
    }
}

#pragma section all restore

// **************************** 代码区域 ****************************
//      encoder_R = encoder_get_count(ENCODER_DIR);
//      encoder_L = -encoder_get_count(ENCODER_QUADDEC);
/*
        MotorDuty1 = (int)PidIncCtrl(&LSpeed_PID, (float)(((sint16)400) - encoder_L  ));
        motor_control_dir(0,(sint32)MotorDuty1);

        seekfree_assistant_oscilloscope_send(&oscilloscope_data);

        oscilloscope_data.data[0]=encoder_L;
        printf("encoder_R=%d\n ",encoder_R);*/

//        system_delay_ms (100);
//        encoder_R = encoder_get_count(ENCODER_DIR);
//         encoder_clear_count(ENCODER_DIR); // 清空编码器计数
