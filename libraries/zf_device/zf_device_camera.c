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
 * 文件名称          zf_device_camera
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.9.4
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-15       pudding           first version
 * 2023-04-25       pudding           增加中文注释说明
 ********************************************************************************************************************/

#include "zf_common_debug.h"
#include "zf_common_interrupt.h"
#include "zf_driver_gpio.h"
#include "zf_driver_dma.h"
#include "zf_driver_exti.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_ov7725.h"
#include "zf_device_scc8660.h"
#include "zf_device_camera.h"
#include "zf_device_ips200.h"
#include "zf_common_font.h"
#include "zf_device_imu660ra.h"
#include "zf_driver_delay.h"
#include "ENA.h"

fifo_obj_struct camera_receiver_fifo;                               // 定义摄像头接收数据fifo结构体
uint8 camera_receiver_buffer[CAMERA_RECEIVER_BUFFER_SIZE];          // 定义摄像头接收数据缓冲区
uint8 camera_send_image_frame_header[4] = {0x00, 0xFF, 0x01, 0x01}; // 定义摄像头数据发送到上位机的帧头

/* 图像*/
unsigned char Image_Use[LCDH][LCDW];
unsigned char Bin_Image[LCDH][LCDW];
uint8_t ImageSide[LCDH][2];      // 左右边线数组
uint8_t ImageSide_last[LCDH][2]; // 左右边线数组副本
uint8_t UpdowmSide[2][LCDW];     // 上下边线数组
uint8_t Roadwide[LCDH];          // 赛道宽度

seektypedef seektype=normal;
/**  @brief    主跑行  */
#define ROAD_MAIN_ROW 60//50//44
/**  @brief    使用起始行  */
#define ROAD_START_ROW 115
/**  @brief    使用结束行  */
#define ROAD_END_ROW 10


extern unsigned char motor_flag;
uint8_t Crossroad_Flag = 0;
unsigned char Threshold = 0;
uint8_t R_CircleFlag = 0; // 右环岛标志位
uint8_t L_CircleFlag = 0; // 左环岛标志位
uint8_t miss_Flag = 0;    // 丢线标志位 1左丢 2右丢

uint8_t leftup[2];
uint8_t rightup[2];

uint8_t turnonpoint[2][3];
uint8_t Lpointx = 0, Lpointy = 0, Rpointx = 0, Rpointy = 0;
//uint8 lastpointy=0;
sint16 g_sSteeringError = 0;
uint8_t Servo_P = 11;
uint8 found_round_flag=0;
extern float error_servo;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       摄像头二进制图像数据解压为十六进制八位数据 小钻风用
// 参数说明       *data1          摄像头图像数组
// 参数说明       *data2          存放解压数据的地址
// 参数说明       image_size      图像的大小
// @return      void
// Sample usage:   camera_binary_image_decompression(&ov7725_image_binary[0][0], &data_buffer[0][0], OV7725_SIZE);
//-------------------------------------------------------------------------------------------------------------------
void camera_binary_image_decompression(const uint8 *data1, uint8 *data2, uint32 image_size)
{
    zf_assert(NULL != data1);
    zf_assert(NULL != data2);
    uint8 i = 8;

    while (image_size--)
    {
        i = 8;
        while (i--)
        {
            *data2++ = (((*data1 >> i) & 0x01) ? 255 : 0);
        }
        data1++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       摄像头图像发送至上位机查看图像
// 参数说明       uartn           使用的串口号
// 参数说明       *image_addr     需要发送的图像地址
// 参数说明       image_size      图像的大小
// @return      void
// Sample usage:                camera_send_image(DEBUG_UART_INDEX, &mt9v03x_image[0][0], MT9V03X_IMAGE_SIZE);
//-------------------------------------------------------------------------------------------------------------------
void camera_send_image(uart_index_enum uartn, const uint8 *image_addr, uint32 image_size)
{
    zf_assert(NULL != image_addr);
    // 发送命令
    uart_write_buffer(uartn, camera_send_image_frame_header, 4);

    // 发送图像
    uart_write_buffer(uartn, (uint8 *)image_addr, image_size);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     摄像头串口 FIFO 初始化
// 参数说明     void
// 返回参数     void
// 使用示例     camera_fifo_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void camera_fifo_init(void)
{
    fifo_init(&camera_receiver_fifo, FIFO_DATA_8BIT, camera_receiver_buffer, CAMERA_RECEIVER_BUFFER_SIZE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       摄像头采集初始化
// 参数说明       image_size      图像的大小
// @return      void
// 参数说明       image_size      图像的大小
// 参数说明       data_addr       数据来源外设地址
// 参数说明       buffer_addr     图像缓冲区地址
// @return      void
// Sample usage:                camera_init();
//-------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
float my_abs(float x)
{
    if (x >= 0)
        return x;
    else
        return -x;
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
uint8 camera_init(uint8 *source_addr, uint8 *destination_addr, uint16 image_size)
{
    uint8 num;
    uint8 link_list_num;
    switch (camera_type)
    {
    case CAMERA_BIN_IIC:  // IIC 小钻风
    case CAMERA_BIN_UART: // UART 小钻风
        for (num = 0; num < 8; num++)
        {
            gpio_init((gpio_pin_enum)(OV7725_DATA_PIN + num), GPI, GPIO_LOW, GPI_FLOATING_IN);
        }
        link_list_num = dma_init(OV7725_DMA_CH,
                                 source_addr,
                                 destination_addr,
                                 OV7725_PCLK_PIN,
                                 EXTI_TRIGGER_FALLING,
                                 image_size);
        exti_init(OV7725_VSYNC_PIN, EXTI_TRIGGER_FALLING); // 初始化场中断，并设置为下降沿触发中断
        break;
    case CAMERA_GRAYSCALE: // 总钻风
        for (num = 0; num < 8; num++)
        {
            gpio_init((gpio_pin_enum)(MT9V03X_DATA_PIN + num), GPI, GPIO_LOW, GPI_FLOATING_IN);
        }
        link_list_num = dma_init(MT9V03X_DMA_CH,
                                 source_addr,
                                 destination_addr,
                                 MT9V03X_PCLK_PIN,
                                 EXTI_TRIGGER_RISING,
                                 image_size); // 如果超频到300M 倒数第二个参数请设置为FALLING

        exti_init(MT9V03X_VSYNC_PIN, EXTI_TRIGGER_FALLING); // 初始化场中断，并设置为下降沿触发中断
        break;
    case CAMERA_COLOR: // 凌瞳
        for (num = 0; num < 8; num++)
        {
            gpio_init((gpio_pin_enum)(SCC8660_DATA_PIN + num), GPI, GPIO_LOW, GPI_FLOATING_IN);
        }

        link_list_num = dma_init(SCC8660_DMA_CH,
                                 source_addr,
                                 destination_addr,
                                 SCC8660_PCLK_PIN,
                                 EXTI_TRIGGER_RISING,
                                 image_size); // 如果超频到300M 倒数第二个参数请设置为FALLING

        exti_init(SCC8660_VSYNC_PIN, EXTI_TRIGGER_FALLING); // 初始化场中断，并设置为下降沿触发中断
        break;
    default:
        break;
    }
    return link_list_num;
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void Get_Use_Image(void)
{
    short i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < LCDH; i++)
    {
        for (j = 0; j <= LCDW; j++)
        {
            Image_Use[row][line] = mt9v03x_image[i][j + 14];//原本160 现在变180
            line++;
        }
        line = 0;
        row++;
    }
}



/*!
 * @brief    基于soble边沿检测算子的一种自动阈值边沿检测
 *
 * @param    imageIn    输入数组
 *           imageOut   输出数组      保存的二值化后的边沿信息
 *
 * @return
 *
 * @note
 *
 * @example
 *
 * @date     2020/5/15
 */
void lq_sobelAutoThreshold(unsigned char imageIn[LCDH][LCDW], unsigned char imageOut[LCDH][LCDW])
{
    /** 卷积核大小 */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = LCDW - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = LCDH - KERNEL_SIZE / 2;
    short i, j, k;
    short temp[4];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* 计算不同方向梯度幅值  */
            temp[0] = -(short)imageIn[i - 1][j - 1] + (short)imageIn[i - 1][j + 1]   //{{-1, 0, 1},
                      - (short)imageIn[i][j - 1] + (short)imageIn[i][j + 1]          // {-1, 0, 1},
                      - (short)imageIn[i + 1][j - 1] + (short)imageIn[i + 1][j + 1]; // {-1, 0, 1}};

            temp[1] = -(short)imageIn[i - 1][j - 1] + (short)imageIn[i + 1][j - 1]   //{{-1, -1, -1},
                      - (short)imageIn[i - 1][j] + (short)imageIn[i + 1][j]          // { 0,  0,  0},
                      - (short)imageIn[i - 1][j + 1] + (short)imageIn[i + 1][j + 1]; // { 1,  1,  1}};

            temp[2] = -(short)imageIn[i - 1][j] + (short)imageIn[i][j - 1]           //  0, -1, -1
                      - (short)imageIn[i][j + 1] + (short)imageIn[i + 1][j]          //  1,  0, -1
                      - (short)imageIn[i - 1][j + 1] + (short)imageIn[i + 1][j - 1]; //  1,  1,  0

            temp[3] = -(short)imageIn[i - 1][j] + (short)imageIn[i][j + 1]           // -1, -1,  0
                      - (short)imageIn[i][j - 1] + (short)imageIn[i + 1][j]          // -1,  0,  1
                      - (short)imageIn[i - 1][j - 1] + (short)imageIn[i + 1][j + 1]; //  0,  1,  1

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);

            /* 找出梯度幅值最大值  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
            temp[3] = (short)imageIn[i - 1][j - 1] + (short)imageIn[i - 1][j] + (short)imageIn[i - 1][j + 1] + (short)imageIn[i][j - 1] + (short)imageIn[i][j] + (short)imageIn[i][j + 1] + (short)imageIn[i + 1][j - 1] + (short)imageIn[i + 1][j] + (short)imageIn[i + 1][j + 1];

            if (temp[0] > temp[3] / 12.0f)
            {
                imageOut[i][j] = 0;
            }
            else
            {
                imageOut[i][j] = 1;
            }
        }
    }
}
#define GrayScale 256

int pixelCount[GrayScale];
float pixelPro[GrayScale];


#define HISTOGRAM_SIZE 256
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void computeHistogram(unsigned char image[LCDH][LCDW], int histogram[])
{
    for (int i = 0; i < LCDH; i++)
    {
        for (int j = 0; j < LCDW; j++)
        {
            histogram[image[i][j]]++;
        }
    }
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
//int otsuThreshold(unsigned char image[LCDH][LCDW])
//{
//    int histogram[HISTOGRAM_SIZE] = {0};
//    computeHistogram(image, histogram);
//
//    int totalPixels = LCDH * LCDW;
//    double sum = 0.0, sumB = 0.0;
//    double varMax = 0.0;
//    int threshold = 0;
//
//    // 计算灰度平均值
//    for (int t = 0; t < HISTOGRAM_SIZE; t++)
//    {
//        sum += t * histogram[t];
//    }
//
//    for (int t = 0; t < HISTOGRAM_SIZE; t++)
//    {
//        double wB = 0, wF = 0;
//        double varBetween, meanB, meanF;
//
//        wB += histogram[t];
//        wF = totalPixels - wB;
//        if (wF == 0)
//            break;
//
//        sumB += (double)(t * histogram[t]);
//
//        meanB = sumB / wB;
//        meanF = (sum - sumB) / wF;
//
//        varBetween = (double)wB * (double)wF * (meanB - meanF) * (meanB - meanF);
//
//        if (varBetween > varMax)
//        {
//            varMax = varBetween;
//            threshold = t;
//        }
//    }
//
//    return threshold;
//}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
int otsuThreshold1(unsigned char image[LCDH][LCDW])
{
    int histogram[HISTOGRAM_SIZE] = {0};
    computeHistogram(image, histogram);

    int totalPixels = LCDH * LCDW;
    double sum = 0.0, sumB = 0.0;
    double varMax = 0.0;
    int threshold = 0;

    // 计算灰度平均值和总和
    for (int t = 0; t < HISTOGRAM_SIZE; t++)
    {
        sum += t * histogram[t];
    }

    // 计算阈值
    for (int t = 0; t < HISTOGRAM_SIZE; t++)
    {
        double wB = 0, wF = 0;
        double varBetween, meanB, meanF;

        wB += histogram[t];
        wF = totalPixels - wB;
        if (wF == 0)
            break;

        sumB += (double)(t * histogram[t]);

        meanB = sumB / wB;
        meanF = (sum - sumB) / wF;

        varBetween = wB * wF * (meanB - meanF) * (meanB - meanF);

        if (varBetween > varMax)
        {
            varMax = varBetween;
            threshold = t;
        }
    }

    return threshold;
}

#define white 255
#define black 0
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void Get_Bin_Image(uint8 mode) // 对图像二值化
{

    // unsigned short i = 0, j = 0;
    switch (mode)
    {
    case 1: // 大津法阈值

        Threshold = (unsigned char)otsuThreshold1(Image_Use);
        for (int i = 1; i < LCDH - 1; i++)
        {
            for (int j = 1; j < LCDW - 1; j++)
            {
                if (Image_Use[i][j] > (Threshold)) // 数值越大，显示的内容越多，较浅的图像也能显示出来
                    Bin_Image[i][j] = white;
                else
                    Bin_Image[i][j] = black;
            }
        }
        // Threshold = (unsigned char) kapurThreshold(Image_Use);
        // ips200_show_char(200, 200, Threshold);

        break;
    case 2:
        lq_sobelAutoThreshold(Image_Use, Bin_Image);

        break;

    default:
        break;
    }

    /*
    for (i = 0; i < LCDH; i++)
    {
        for (j = 0; j < LCDW; j++)
        {
            if (Image_Use[i][j] > Threshold) // 数值越大，显示的内容越多，较浅的图像也能显示出来
                Bin_Image[i][j] = 1;
            else
                Bin_Image[i][j] = 0;
        }
    }
    */
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void Bin_Image_Filter(void) // 对图像进行滤波
{
    sint16 nr; // 行
    sint16 nc; // 列
    for (nr = 1; nr < LCDH - 1; nr++)
    {
        for (nc = 1; nc < LCDW - 1; nc = nc + 1)
        {
            if ((Bin_Image[nr][nc] == 0) && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] > 2))
            {
                Bin_Image[nr][nc] = 1;
            }
            else if ((Bin_Image[nr][nc] == 1) && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] < 2))
            {
                Bin_Image[nr][nc] = 0;
            }
        }
    }
}
#define threshold_max   255*6//5此参数可根据自己的需求调节
#define threshold_min   255*3//2此参数可根据自己的需求调节
void image_filter(void)//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;


    for (i = 1; i < LCDH - 1; i++)
    {
        for (j = 1; j < (LCDW - 1); j++)
        {
            //统计八个方向的像素值
            num =
                    Bin_Image[i - 1][j - 1] + Bin_Image[i - 1][j] + Bin_Image[i - 1][j + 1]
                + Bin_Image[i][j - 1] + Bin_Image[i][j + 1]
                + Bin_Image[i + 1][j - 1] + Bin_Image[i + 1][j] + Bin_Image[i + 1][j + 1];


            if (num >= threshold_max && Bin_Image[i][j] == 0)
            {

                Bin_Image[i][j] = 255;//白  可以搞成宏定义，方便更改

            }
            if (num <= threshold_min && Bin_Image[i][j] == 255)
            {

                Bin_Image[i][j] = 0;//黑

            }

        }
    }

}
/*!
 * @brief    判断是否丢线
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageOut   ： 边线数组
 * @param    lineIndex  ： 行
 *
 * @return   0：没有丢线   1:左边丢线  2：右边丢线  3： 左右都丢线   4：错误
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/24 星期三
 */
uint8_t RoadIsNoSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t lineIndex)
{
    uint8_t state = 0;
    uint8_t i = 0;
    static uint8_t last = 78;
    imageOut[lineIndex][0] = 0;
    imageOut[lineIndex][1] =LCDW-1 ;
    /* 用距离小车比较近的行 判断是否丢线 */
    for (i = last; i > 1; i--)
    {
        if (!imageInput[lineIndex][i])
        {
            imageOut[lineIndex][0] = i;
            break;
        }
    }
    if (i == 1)
    {
        /* 左边界丢线 */
        state = 1;
    }

    for (i = last; i < LCDW-1; i++)
    {
        if (!imageInput[lineIndex][i])
        {
            imageOut[lineIndex][1] = i;
            break;
        }
    }
    if (i == LCDW-1)
    {
        /* 左右边界丢线 */
        if (state == 1)
        {
            state = 3;
        }
        /* 右边界丢线 */
        else
        {
            state = 2;
        }
    }
    if (imageOut[lineIndex][1] <= imageOut[lineIndex][0])
    {
        state = 4;
    }
    return state;
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void RoadNoSideProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t mode, uint8_t lineIndex)
    {
        uint8_t i = 0, j = 0, count = 0;
        switch (mode)
        {
        case 1:
            for (i = imageOut[lineIndex][1]; i > 1; i--)
            {
                count++;
                for (j = lineIndex; j > ROAD_END_ROW && lineIndex > count; j--)
                {
                    if (imageInput[j][i])
                    {
                        imageOut[lineIndex - count][0] = 0;
                        imageOut[lineIndex - count][1] = i;
                        break;
                    }
                }
            }
            break;
        case 2:
            for (i = imageOut[lineIndex][0]; i < LCDW-1; i++)
            {
                count++;
                for (j = lineIndex; j > ROAD_END_ROW && lineIndex > count; j--)
                {
                    if (imageInput[j][i])
                    {
                        imageOut[lineIndex - count][0] = i;
                        imageOut[lineIndex - count][1] = LCDW-1;
                        break;
                    }
                }
            }
            break;
        }
    }

uint8_t right_line_lost,left_line_lost;
uint8_t Right_Lost_Flag[LCDH]={0};
uint8_t Left_Lost_Flag[LCDH]={0};
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
//void Find_Boundry(uint8_t imageOut[LCDH][2])//从中间往两边搜索中线
//{
//    int i,j;
//    static int left_border=0,right_border=0,mid=LCDW/2,last_mid=LCDW/2;
//    right_line_lost=0;
//    left_line_lost=0;
//    //起始点合理性判断
//    if(Bin_Image[MT9V03X_H-1][LCDW/2]==0x00)//屏幕中线是黑的话
//    {
//        if(Bin_Image[MT9V03X_H-1][LCDW/4]==0xff)//看看左1/4是不是白
//        {
//            last_mid=LCDW/4;//更改搜索起始点
//        }
//        else if(Bin_Image[MT9V03X_H-1][LCDW/4*3]==0xff)//看看右1/4是不是白
//        {
//            last_mid=LCDW/4*3;//更改搜索起始点
//        }
//    }
//    //开始巡边
//    for(i=MT9V03X_H-1;i>=0;i--)//从最底下往上扫描
//    {
//        for(j=last_mid;j<LCDW-3;j++)//往右扫描
//        {
//            if(Bin_Image[i][j]==0xff&&Bin_Image[i][j+1]==0x00&&Bin_Image[i][j+2]==0x00)//白黑黑，找到右边界
//            {
//                right_border=j;
//                Right_Lost_Flag[i]=0; //右丢线数组，丢线置1，不丢线置0
//                break;//跳出，找到本行边界就没必要循环下去了
//            }
//            else
//            {
//                right_border=j;//没找到右边界，把屏幕最右赋值给右边界
//                Right_Lost_Flag[i]=1; //右丢线数组，丢线置1，不丢线置0
//            }
//        }
//        right_line_lost+=Right_Lost_Flag[i];
//        for(j=last_mid;j>1;j--)//往左边扫描
//        {
//            if(Bin_Image[i][j]==0xff&&Bin_Image[i][j-1]==0x00&&Bin_Image[i][j-2]==0x00)//黑黑白认为到达左边界
//            {
//                left_border=j;
//                Left_Lost_Flag[i]=0; //左丢线数组，丢线置1，不丢线置0
//                break;//跳出，找到本行边界就没必要循环下去了
//            }
//            else
//            {
//                left_border=j;//找到头都没找到边，就把屏幕最左右当做边界
//                Left_Lost_Flag[i]=1; //左丢线数组，丢线置1，不丢线置0
//            }
//        }
//        left_line_lost+=Left_Lost_Flag[i];
//        mid=(left_border+right_border)/2;//中线坐标
//        last_mid=mid;//中线查找开始点，方便下一次找中线
//        imageOut[i][0]= (uint8_t)left_border ;      //左边线线数组
//        imageOut[i][1]= (uint8_t)right_border;      //右边线线数组
//        Roadwide[i]=imageOut[i][1]-imageOut[i][0];
//    }
//}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
uint8_t ImageGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t imageOut_last[LCDH][2],uint8 mode) // 获取左右边线
{
    uint8_t i = 0, j = 0;
    uint8 centerline=0;
    switch(mode)
    {
        case 1:
        RoadIsNoSide(imageInput, imageOut, ROAD_START_ROW); //
/* 离车头近的40行 寻找边线 */
for (i = ROAD_START_ROW -1; i > ROAD_END_ROW; i--)
{
    imageOut[i][0] = 0;
    imageOut[i][1] = LCDW-1;

    // 根据边界连续特性 寻找边界
    for (j = imageOut[i + 1][0] + 10; j > 0; j--)
    {
        if (!imageInput[i][j])
        {
            imageOut[i][0] = j;
            break;
        }
    }
    for (j = imageOut[i + 1][1] - 10; j < LCDW-1; j++)
    {
        if (!imageInput[i][j])
        {
            imageOut[i][1] = j;
            break;
        }
    }


    /* 如果左边界 即将超出中线 则检查是否右丢线 */

    if (imageOut[i][0] > (LCDW / 2 - 10) && imageOut[i][1] > (LCDW - 5))
    {
        // 右丢线处理
        RoadNoSideProcess(imageInput, imageOut, 2, i); // ？存疑

        if (i > 70)
        {
            imageOut[i][0] += 50;
        }
        return 1;
    }

    /* 如果右边界 即将超出中线 则检查是否左丢线 */

    if (imageOut[i][1] < (LCDW / 2 + 10) && imageOut[i][0] < (5))
    {
        // 左丢线处理
        RoadNoSideProcess(imageInput, imageOut, 1, i);

        if (i > 70)
        {
            imageOut[i][1] -= 50;
        }
        return 2;
    }


}
        break;
        case 2:

            for(i=LCDW/2;i>1;i--)
            {
                if (!imageInput[115][i])
                {
                    imageOut[115][0] = i;
                    break;
                }
            }
            for(i=LCDW/2;i<LCDW-1;i++)
            {
            if (!imageInput[115][i])
               {
                   imageOut[115][1] = i;
                   break;
               }
            }
            centerline=(imageOut[115][1]+ imageOut[115][0])>>1;
            for(j=114;j>10;j--)
            {
                imageOut[j][0] = 0;
                imageOut[j][1] = LCDW-1;
                for(i=centerline;i>1;i--)
                {
                    if (!imageInput[j][i])
                               {
                                   imageOut[j][1] = i;
                                   break;
                               }
                   }

                for(i=centerline;i<LCDW-1;i++)
                {
                    if (!imageInput[j][i])
                           {
                               imageOut[j][1] = i;
                               break;
                           }
                 }
                centerline=(imageOut[j][1]+ imageOut[j][0])>>1;
              }
            break;
            }

    return 0;
}

void imageside(uint8 mode)
{
    switch (mode)
    {
    case 1:
        ImageGetSide(Bin_Image, ImageSide, ImageSide_last,1);
        break;
    case 2:
        enh_process();
        break;
    default:
        break;
    }

}
//----------------------------------------------------------
//
int windowSize = 3;
//
//
//----------------------------------------------------------
void triangularFilter(uint8_t edge[LCDH][2], uint8_t filteredEdge[LCDH][2], int windowSize) {
    // 确保窗口大小是奇数
    if (windowSize % 2 == 0) {
        windowSize++;
    }

    int halfWindowSize = windowSize / 2;
    for (int i = 0; i < LCDH; i++) {
        int sum = 0;
        int weightSum = 0;

        // 应用三角权重
        for (int j = -halfWindowSize; j <= halfWindowSize; j++) {
            int index = i + j;
            if (index >= 0 && index < LCDH) {
                int weight = abs(j) + 1; // 权重从1到windowSize递增
                sum += edge[index][0] * weight;
                weightSum += weight;
            }
        }

        // 计算加权平均值
        filteredEdge[i][0] = sum / weightSum;
    }
}



//#define LCDH 120  // 边线数据的高度
//#define LCDW 160  // 边线数据的宽度
//#define SIGMA_SPATIAL 2.0  // 空间域的标准差
//#define SIGMA_INTENSITY 0.1  // 强度域的标准差
//
//// 计算高斯权重
//double gaussian(double x, double sigma) {
//    return exp(-x * x / (2 * sigma * sigma));
//}

// 双边滤波
//void bilateralFilterEdge(uint8_t edge[LCDH][2], uint8_t filteredEdge[LCDH][2])
//{
////    double weightSum;
////    int left, right;
////
////    // 对每个边线点进行双边滤波
////    for (int i = 0; i < LCDH; i++) {
////        // 初始化累积变量
////        weightSum = 0.0;
////        int filteredValue = 0;
////
////        // 遍历边线点的邻域
////        for (int j = 0; j < LCDH; j++) {
////            // 计算空间域的高斯权重
////            double spatialWeight = gaussian(fabs(i - j), SIGMA_SPATIAL);
////
////            // 计算强度域的高斯权重
////            double intensityDifference = (double)edge[j][0] - (double)edge[i][0];
////            double intensityWeight = gaussian(intensityDifference * intensityDifference, SIGMA_INTENSITY);
////
////            // 计算总权重
////            double totalWeight = spatialWeight * intensityWeight;
////
////            // 累加加权的强度值
////            filteredValue += totalWeight * (double)edge[j][0];
////            weightSum += totalWeight;
////        }
////
////        // 归一化得到最终的滤波值
////        filteredEdge[i][0] = (uint8_t)(filteredValue / weightSum);
////    }
//    const int windowSize = 3; // 定义一个较小的窗口大小
//       double weightSum, totalWeight;
//       int filteredValue;
//       int left, right;
//
//       for (int i = 0; i < LCDH; i++) {
//           weightSum = 0.0;
//           filteredValue = 0;
//
//           for (int j = i - windowSize; j <= i + windowSize; j++) {
//               if (j < 0 || j >= LCDH) continue; // 边界检查
//
//               double spatialDistance = fabs(i - j);
//               double spatialWeight = gaussian(spatialDistance, SIGMA_SPATIAL > spatialDistance ? SIGMA_SPATIAL : spatialDistance);
//
//               double intensityDifference = (double)edge[j][0] - (double)edge[i][0];
//               double intensityWeight = gaussian(intensityDifference * intensityDifference, SIGMA_INTENSITY);
//
//               totalWeight = spatialWeight * intensityWeight;
//
//               filteredValue += totalWeight * (double)edge[j][0];
//               weightSum += totalWeight;
//           }
//
//           filteredEdge[i][0] = (uint8_t)(filteredValue / (weightSum > 0.0 ? weightSum : 1.0));
//       }
//}


//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void medianFilterEdge(uint8_t imagein[LCDH][2], uint8_t imageOut[LCDH][2])
{
    uint8_t window[3][2];
    for (int i = 1; i < LCDH - 1; i++) {
        // 选择当前点及左右点作为滤波窗口
        window[0][0] = imagein[i - 1][0];
        window[1][0] = imagein[i][0];
        window[2][0] = imagein[i + 1][0];

        // 对窗口内的点进行排序
        if (window[0][0] > window[1][0]) { uint8_t temp = window[0][0]; window[0][0] = window[1][0]; window[1][0] = temp; }
        if (window[1][0] > window[2][0]) { uint8_t temp = window[1][0]; window[1][0] = window[2][0]; window[2][0] = temp; }
        if (window[0][0]> window[1][0]) { uint8_t temp = window[0][0]; window[0][0] = window[1][0]; window[1][0] = temp; }

        // 取中值作为滤波后的结果
        imageOut[i][0] = window[1][0];


               window[0][1] = imagein[i - 1][1];
               window[1][1]= imagein[i][1];
               window[2][1] = imagein[i + 1][1];

               // 对窗口内的点进行排序
               if (window[0][1] > window[1][1]) { uint8_t temp = window[0][1]; window[0][1] = window[1][1]; window[1][1] = temp; }
               if (window[1][1] > window[2][1]) { uint8_t temp = window[1][1]; window[1][1] = window[2][1]; window[2][1] = temp; }
               if (window[0][1]> window[1][1]) { uint8_t temp = window[0][1]; window[0][1] = window[1][1]; window[1][1] = temp; }

               // 取中值作为滤波后的结果
               imageOut[i][1] = window[1][1];
    }
    // 边界处理（第一个和最后一个点）
    imageOut[0][0] = imagein[0][0];
    imageOut[0][1] = imagein[0][1];
    imageOut[LCDH - 1][0] = imagein[LCDH - 1][0];
    imageOut[LCDH - 1][1] = imagein[LCDH - 1][1];
}

//----------------------------------------------------------
//
//
//
//----------------------------------------------------------

uint8_t UpdownSideGet(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[2][LCDW]) // 获取上下边线
{
    uint8_t i = 0, j = 0;
//    uint8_t last = 70;
//
//    imageOut[0][LCDW-1] = 0;
//    imageOut[1][LCDW-1] = 119;
//    /* 用中线比较近的行 判断是否丢线 */
//    for (i = last; i >= 0; i--)
//    {
//        if (!imageInput[i][80])
//        {
//            imageOut[0][80] = i;
//            break;
//        }
//    }
//
//    for (i = last; i < 140; i++)
//    {
//        if (!imageInput[i][80])
//        {
//            imageOut[1][80] = i;
//            break;
//        }
//    }
//



       // imageOut[0][LCDW-1] = 0;
        //imageOut[1][LCDW-1] = 119;
        /* 用中线比较近的行 判断是否丢线 */
        for (i = 70; i >= 0; i--)//5.6 六十变七十
        {
            if (!imageInput[i][0])
            {
                imageOut[0][0] = i;
                break;
            }
        }
        for (i = 70; i >= 0; i--)//5.6 六十变七十
               {
                   if (!imageInput[i][LCDW-1])
                   {
                       imageOut[0][LCDW-1] = i;
                       break;
                   }
               }
//        for (i = 60; i < 120; i++)//5.6注释掉，似乎为下边线
//        {
//            if (!imageInput[i][0])
//            {
//                imageOut[1][0] = i;
//                break;
//            }
//        }
        for(i=1;i<LCDW/2;i++)
        {
            for (j = imageOut[0][i - 1] + 10; j > 0; j--)
               {
                   if (!imageInput[j][i])
                   {
                       imageOut[0][i] = j;
                       break;
                   }
               }
        }
        for(i=LCDW-2;i>=LCDW/2;i--)
                {
                    for (j = imageOut[0][i + 1] + 10; j > 0; j--)
                       {
                           if (!imageInput[j][i])
                           {
                               imageOut[0][i] = j;
                               break;
                           }
                       }
                }
//    /* 中线往左 寻找边线 */
//    for (i = 90 - 1; i > 0; i--)//原本是90
//    {
//        imageOut[0][i] = 0;
//        imageOut[1][i] = 119;
//
//        /* 根据边界连续特性 寻找边界 */
//        for (j = imageOut[0][i + 1] + 10; j > 0; j--)
//        {
//            if (!imageInput[j][i])
//            {
//                imageOut[0][i] = j;
//                break;
//            }
//        }
//        for (j = imageOut[1][i + 1] - 10; j < 120; j++)
//        {
//            if (!imageInput[j][i])
//            {
//                imageOut[1][i] = j;
//                break;
//            }
//        }
//    }
//    /*中线往右 寻找边线*/
//    for (i = 90 + 1; i < LCDW-1; i++)
//    {
//        imageOut[0][i] = 0;
//        imageOut[1][i] = 119;
//
//        /* 根据边界连续特性 寻找边界 */
//        for (j = imageOut[0][i - 1] + 10; j > 0; j--)
//        {
//            if (!imageInput[j][i])
//            {
//                imageOut[0][i] = j;
//                break;
//            }
//        }
//        for (j = imageOut[1][i - 1] - 10; j < 120; j++)
//        {
//            if (!imageInput[j][i])
//            {
//                imageOut[1][i] = j;
//                break;
//            }
//        }
//    }



    return 0;
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
uint8_t GetRoadWide(uint8_t imageInput[LCDH][2], uint8_t imageOut[LCDH])
{
    uint8_t i = 0;
    for (i = 10; i <= LCDH - 5; i++)
    {
        imageOut[i] = 0;
        if (imageInput[i][1] > imageInput[i][0])
        {
            imageOut[i] = imageInput[i][1] - imageInput[i][0];
        }
        else
        {
            imageOut[i] = 160;
        }
//        printf("%d\n",imageOut[i]);
    }
    return 0;
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void ips200_bin_roadside(uint8_t imageOut[LCDH][2])
{

    uint8_t i = 0;
    for (i = 0; i < LCDH; i++)
    {
        ips200_draw_point(imageOut[i][0], i, RGB565_RED);
        ips200_draw_point( imageOut[i][1],i, RGB565_RED);
    }
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void ips200_bin_updownside(uint8_t imageOut[2][LCDW])
{

    uint8_t i = 0;
    for (i = 0; i < LCDW; i++)
    {
        ips200_draw_point(i, imageOut[0][i], RGB565_BLACK);
        //ips200_draw_point(i, imageOut[1][i], RGB565_GREEN);
    }
}

extern float error_servo;
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void Show_Camera_Info(void)
{

//    ips200_show_gray_image(0, 0, Image_Use[0], LCDW, LCDH, LCDW, LCDH,Threshold);//Threshold
    ips200_bin_roadside(ImageSide);
//    ips200_bin_updownside(UpdowmSide);
    // ips200_show_side_image(0,0,LCDW, LCDH);





}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
//void FindCarbarn(uint8_t imageInput[LCDH][LCDW], uint8_t *Flag) // 斑马线
//{
//    uint8_t i, Flag_number;
//    Flag_number = 0;
//    for (i = 5; i <= LCDW - 5; i++)
//    {
//        if (imageInput[50][i] != imageInput[50][i + 1])
//        {
//            Flag_number++;
//        }
//    }
//    if (Flag_number > 10)
//    {
//        *Flag = 1;
//    }
//}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
//void ZebraProcess(uint8_t imageSide[LCDH][2], uint8_t state, uint8_t *motor)
//{
//    static uint16_t count = 0;
//    count++;
//    if (state == 1)
//    {
//        imageSide[ROAD_MAIN_ROW][0] = 0;
//        imageSide[ROAD_MAIN_ROW][1] = LCDW / 2;
//    }
//    else
//    {
//        imageSide[ROAD_MAIN_ROW][0] = LCDW / 2;
//        imageSide[ROAD_MAIN_ROW][1] = LCDW - 1;
//    }
//    if (count > 100)
//    {
//        *motor = 0;
//        while (1)
//            ;
//    }
//}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
//void Carbarn(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t *motor_flag, uint8_t *Flag)
//{
//    switch (*Flag)
//    {
//    case 0:
//        FindCarbarn(imageInput, &*Flag); // 车库识别
//        break;
//    case 1:
//        ZebraProcess(imageSide, 1, &*motor_flag); // 进入车库
//        break;
//    }
//}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------

/*!
 * @brief    判断边线是否存在弧形
 *
 * @param    imageInput ： 二值图像信息
 * @param    imageOut   ： 边线数组
 * @param    status     ： 1：左边线  2：右边线
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2020/6/23 星期二
 */
uint8_t RoundaboutGetArc(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t num, uint8_t *index,uint8 start)
{
//    uint8 i = 0;
//    uint8_t inc = 0, dec = 0, n = 0;
    uint8 first_catch=0;
    switch (status)
    {
    case 1:
//        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW;i--)
//        {
//            if (imageSide[i][0] != 0 && imageSide[i + 1][0] != 0)
//            {
//                if (imageSide[i][0] == imageSide[i + 1][0])
//                {
//                    n++;
//                    continue;
//                }
//                if (imageSide[i][0] > imageSide[i + 1][0])
//                {
//                    inc++;
//                    inc += n;
//                    n = 0;
//                }
//                else
//                {
//                    dec++;
//                    dec += n;
//                    n = 0;
//                }
//                /* 有弧线 */
//                if (inc > num && dec > num)
//                {
//                    *index = i + num;
//                    return 1;
//                }
//            }
//            else
//            {
//                inc = 0;
//                dec = 0;
//                n = 0;
//            }
//        }
        for (uint8 i =start ;i >10;  i--)
                       {
                                if(first_catch==0&&imageSide[i][0] >imageSide[i+5][0]&&imageSide[i][0]>imageSide[i-5][0]&&
                                   imageSide[i][0]>imageSide[i+4][0]&&imageSide[i] [0]>imageSide[i-4][0]&&
                                   imageSide[i][0]>=imageSide[i+3][0]&&imageSide[i][0]>=imageSide[i-3][0]&&
                                   imageSide[i][0]>=imageSide[i+2][0]&&imageSide[i][0]>=imageSide[i-2][0]&&
                                   imageSide[i][0]>=imageSide[i+1][0]&&imageSide[i][0]>=imageSide[i-1][0]&&
                                   imageSide[i][0]>30&&imageSide[i-1][0]>25&&imageSide[i+1][0]>25)

                           {
//                                    ips200_draw_line(ImageSide[i][0], i, LCDW-1,i, RGB565_GREEN);
                                    //ips200_show_uint(60,180,i,3);
                                    first_catch=1;
                               *index = i;
                               return 1;
                           }
                       }
        break;

    case 2:
//        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW;i--)
//        {
//            if (imageSide[i][1] != LCDW-1 && imageSide[i + 1][1] != LCDW-1)
//            {
//                if (imageSide[i][1] == imageSide[i + 1][1])
//                {
//                    n++;
//                    continue;
//                }
//                if (imageSide[i][1] > imageSide[i + 1][1])
//                {
//                    inc++;
//                    inc += n;
//                    n = 0;
//                }
//                else
//                {
//                    dec++;
//                    dec += n;
//                    n = 0;
//                }
//                /* 有弧线 */
//                if (inc > num && dec > num)
//                {
//                    *index = i + num;
//                    return 1;
//                }
//            }
//            else
//            {
//                inc = 0;
//                dec = 0;
//                n = 0;
//            }
//        }
//        break;
//    }
        for (uint8 i =start; i >10 ;i--)
                              {
                                       if(
                                               first_catch==0&&imageSide[i][1] <imageSide[i+5][1]&&imageSide[i][1]<imageSide[i-5][1]&&
                                               imageSide[i][1] <imageSide[i+4][1]&&imageSide[i][1] <imageSide[i-4][1]&&
                                               imageSide[i][1]<=imageSide[i+3][1]&&imageSide[i][1]<=imageSide[i-3][1]&&
                                               imageSide[i][1]<=imageSide[i+2][1]&&imageSide[i][1]<=imageSide[i-2][1]&&
                                               imageSide[i][1]<=imageSide[i+1][1]&&imageSide[i][1]<=imageSide[i-1][1]&&
                                               imageSide[i][1]<140&&imageSide[i-1][1]<145&&imageSide[i+1][1]<145 )//

                                  {
                                           //ips200_draw_line(ImageSide[i][1], i, LCDW-1,i, RGB565_GREEN);
                                           //ips200_show_uint(50,220,ImageSide[i][1],3);
                                           first_catch=1;
                                      *index = i;
                                      return 1;
                                  }


                              }


        break;
    }
    return 0;
}
/*!
 * @brief    判断边线是否存在弧形
 *
 * @param    SideInput ： 上边线数组
 * @param    num       ： 检测幅度
 * @param    index     ： 最低点
 *
 * @return
 *
 * @note
 *
 * @see
 *
 * @date     2021/12/01 星期三
 */
uint8_t UpSideErr(uint8_t SideInput[2][LCDW], uint8_t status, uint8_t num, uint8_t *index)
{
//    uint8_t dec = 0, inc = 0, i;
//    // 上线是否右突起
//    switch (status)
//    {
//    case 1:
//        for (i = LCDW-1 - 1; i > 0; i--)
//        {
//            if (UpdowmSide[0][i] > 1 && UpdowmSide[0][i + 1] > 1)
//            {
//                if (UpdowmSide[0][i] >= UpdowmSide[0][i + 1])
//                    inc++;
//                else
//                    dec++;
//                /* 有弧线 */
//                if (inc > num && dec > num)
//                {
//                    *index = i + num;
//                    return 1;
//                }
//            }
//            else
//            {
//                inc = 0;
//                dec = 0;
//            }
//        }
//        break;
//    // 下边线
//    case 2:
//        for (i = LCDW-1 - 1; i > 0; i--)
//        {
//            if (UpdowmSide[1][i] != 1 && UpdowmSide[1][i + 1] != 1)
//            {
//                if (UpdowmSide[1][i] >= UpdowmSide[1][i + 1])
//                    inc++;
//                else
//                    dec++;
//                /* 有弧线 */
//                if (inc > num && dec > num)
//                {
//                    *index = i + num;
//                    return 1;
//                }
//            }
//            else
//            {
//                inc = 0;
//                dec = 0;
//            }
//        }
//        break;
//    }
switch(status)
{
    case 1:
   for (uint8 i =LCDW/2+10 ;i >20 - 1;  i--)
                           {
                                    if(
                                            SideInput[0][i] >SideInput[0][i+5]&&SideInput[0][i]>SideInput[0][i-5]&&
                                            SideInput[0][i]>SideInput[0][i+4]&&SideInput[0][i]>SideInput[0][i-4]&&
                                            SideInput[0][i]>=SideInput[0][i+3]&&SideInput[0][i]>=SideInput[0][i-3]&&
                                            SideInput[0][i]>=SideInput[0][i+2]&&SideInput[0][i]>=SideInput[0][i-2]&&
                                            SideInput[0][i]>=SideInput[0][i+1]&&SideInput[0][i]>=SideInput[0][i-1]&&
                                            SideInput[0][i]>30&&SideInput[0][i-1]>15&&SideInput[0][i+1]>15
                                       )

                               {
    //                                    ips200_draw_line(ImageSide[i][0], i, LCDW-1,i, RGB565_GREEN);
                                        //ips200_show_uint(60,180,i,3);
                                   *index = i;
                                   return 1;
                               }
                           }
   break;
    case 2:
        for (uint8 i =LCDW/2 ;i <LCDW-20;  i++)
                                {
                                         if(
                                                 SideInput[0][i] >SideInput[0][i+5]&&SideInput[0][i]>SideInput[0][i-5]&&
                                                 SideInput[0][i]>SideInput[0][i+4]&&SideInput[0][i]>SideInput[0][i-4]&&
                                                 SideInput[0][i]>=SideInput[0][i+3]&&SideInput[0][i]>=SideInput[0][i-3]&&
                                                 SideInput[0][i]>=SideInput[0][i+2]&&SideInput[0][i]>=SideInput[0][i-2]&&
                                                 SideInput[0][i]>=SideInput[0][i+1]&&SideInput[0][i]>=SideInput[0][i-1]&&
                                                 SideInput[0][i]>30&&SideInput[0][i-1]>15&&SideInput[0][i+1]>15
                                            )

                                    {
         //                                    ips200_draw_line(ImageSide[i][0], i, LCDW-1,i, RGB565_GREEN);
                                             //ips200_show_uint(60,180,i,3);
                                        *index = i;
                                        return 1;
                                    }
                                }
        break;


}


    return 0;
}
/*!
 * @brief    寻找断点
 *
 * @param    start_point ： 开始点
 * @param    end_point   ： 结束点
 * @param    UpdowmSide  ： 边线数组
 * @param    *pointup[2] ： 断点坐标数组
 *
 * @return   void
 *
 * @note
 *
 * @see         开始点小于结束点
 *
 * @date     2022/12/13
 */
//void X_Find_Point(uint8_t start_point, uint8_t end_point, uint8_t UpdowmSide[2][LCDW], uint8_t pointup[2])
//{
//    uint8_t i = 0;
//    for (i = start_point; i <= end_point; i++)
//    {
//        if ((abs(UpdowmSide[0][i - 1] - UpdowmSide[0][i - 2] < 3 && abs(UpdowmSide[0][i - 1] - UpdowmSide[0][i + 1]) > 3) && abs(UpdowmSide[0][i + 1] - UpdowmSide[0][i + 2]) < 3) || (abs(UpdowmSide[0][i - 1] - UpdowmSide[0][i - 3] < 5 && abs(UpdowmSide[0][i - 2] - UpdowmSide[0][i + 2]) > 5) && abs(UpdowmSide[0][i + 1] - UpdowmSide[0][i + 3]) < 5))
//        {
//            pointup[0] = i;
//            pointup[1] = UpdowmSide[0][i];
//            break;
//        }
//        else
//        {
//            pointup[0] = 0;
//            pointup[1] = 0;
//        }
//    }
//}

//Flag 0:左边线 1右边线
typedef struct
{
        uint8_t cross_left_up_X;
        uint8_t cross_left_up_Y;
        uint8_t cross_left_down_X;
        uint8_t cross_left_down_Y;
        uint8_t cross_right_down_X;
        uint8_t cross_right_down_Y;
        uint8_t cross_right_up_X;
        uint8_t cross_right_up_Y; // 修改这里，将第二次重复的 cross_right_up_X 改为 cross_right_up_Y
        uint8_t cross_flag;
} cross_struct;
cross_struct cross;
uint8_t line_miss(uint8_t imageIn[LCDH][2], uint8_t Flag,uint8 starty)
{
    uint8_t i = 0;
    uint8_t misi_number=0;

    if (Flag == 0)
    {
        for (i = starty; i < LCDH; i++)
        {
            if (imageIn[i][0] < 10)
                misi_number++;

        }
    }
    if (Flag == 1)
    {
        for (i = starty; i < LCDH; i++)
        {
            if (imageIn[i][1] > LCDW-5)//丢线严重，从-10变为-5
                misi_number++;
        }
    }
    return misi_number;


//if(misi_number>=5&&misi_number1>=5)cross.cross_flag=1;
//    return misi_number;
}
uint8_t cross_flag;
void cross_process(uint8_t imageIn[LCDH][2])
{




//    leftmiss=line_miss(imageIn,0);
//    rightmiss=line_miss(imageIn,1);
    for(uint8_t i=120-1;i>=20;i--)
    {
        if(abs(ImageSide[i][0]-ImageSide[i+1][0])<3&&abs(ImageSide[i][0]-ImageSide[i-1][0])>5&&ImageSide[i][0]<70&&i<50)
        {
            cross.cross_left_down_X=ImageSide[i][0];//x
            cross.cross_left_down_Y=i;//y
            ips200_show_char(0,150,'a');
            break;
        }
    }
    for(uint8_t i=120-1;i>=20;i--)
    {
        if(abs(ImageSide[i][1]-ImageSide[i+1][1])<3&&abs(ImageSide[i][1]-ImageSide[i-1][1])>5&&ImageSide[i][1]>90&&i<50)
                {
                    cross.cross_right_down_X=ImageSide[i][1];//x
                    cross.cross_right_down_Y=i;//y
                    ips200_show_char(0,160,'b');
                    break;
                }
    }

    for(uint8_t i=120-1;i>=20;i--)
    {
    if(abs(ImageSide[i][0]-ImageSide[i-1][0])>5&&abs(ImageSide[i-1][0]-ImageSide[i-2][0])<4&&ImageSide[i-1][0]<70&&i-1>50)
    {
        cross.cross_left_up_X=ImageSide[i-1][0];
        cross.cross_left_up_Y=i-1;
        ips200_show_char(0,190,'d');

        break;
    }

    }
    for(uint8_t i=120-1;i>=20;i--)
    {
    if(abs(ImageSide[i][1]-ImageSide[i-1][1])>5&&abs(ImageSide[i-1][1]-ImageSide[i-2][1])<4&&ImageSide[i-1][0]>90&&(i-1>50))
        {

            cross.cross_right_up_X=ImageSide[i-1][1];
            cross.cross_right_up_Y=i-1;
            cross.cross_flag=3;
            ips200_show_char(0,200,'e');
            break;

        }
    }

         ImageAddingLine(imageIn, 1, cross.cross_left_up_X, cross.cross_left_up_Y, cross.cross_left_down_X, cross.cross_left_down_Y);
         ImageAddingLine(imageIn, 2, cross.cross_right_up_X, cross.cross_right_up_Y, cross.cross_right_down_X, cross.cross_right_down_Y);
         cross.cross_flag=0;


}
uint8_t r_up_guaiflag;
uint8_t l_up_guaiflag;
uint8_t r_down_guaiflag;
uint8_t l_down_guaiflag;
uint8_t guai_Flag = 0;
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
int  left_down_guai(uint8_t imageSide[LCDH][2],int start,int end, uint8_t *flag)
{
    int i,t;
    int left_down_line=0;
    if(start<end)//从图像往上遍历
       {
           t=start;
           start=end;
           end=t;
       }
    if(start>=LCDH-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
            start=LCDH-1-5;
    if(end<=5)
            end=5;

    for(i=start;i>=end;i--)
    {
        if( left_down_line==0&&
                (imageSide[i][0] - imageSide[i + 1][0]) <= 5 &&
                (imageSide[i + 1][0] - imageSide[i + 2][0]) <= 5 &&
                (imageSide[i + 2][0] - imageSide[i + 3][0]) <= 5 &&
                (imageSide[i][0] - imageSide[i - 2][0]) >= 5 &&
                (imageSide[i][0] - imageSide[i - 3][0]) >= 10 &&
                (imageSide[i][0] - imageSide[i - 4][0]) >= 10  )
        {
            left_down_line = i;
            *flag = 1;
            break;

        }
        else
            *flag = 0;

    }
    return left_down_line;
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
int right_down_guai(uint8_t imageSide[LCDH][2],int start,int end, uint8_t *flag)
{
    int i,t;
    int right_down_line=0;
    if(start<end)//从图像往上遍历
       {
           t=start;
           start=end;
           end=t;
       }
    if(start>=LCDH-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
            start=LCDH-1-5;
    if(end<=5)
            end=5;

    for(i=start;i>=end;i--)               //便利每一行查看是否满足条件
    {
        if( right_down_line==0&&
                (imageSide[i][1] - imageSide[i + 1][1]) >= -5 &&
                (imageSide[i + 1][1] - imageSide[i + 2][1]) >= -5 &&
                (imageSide[i + 2][1] - imageSide[i + 3][1]) >= -5 &&
                (imageSide[i][1] - imageSide[i - 2][1]) <= -5 &&
                (imageSide[i][1] - imageSide[i - 3][1]) <= -10 &&
                (imageSide[i][1] - imageSide[i - 4][1]) <= 10  )
        {
            right_down_line = i;
            *flag = 1;
            break;

        }
        else  *flag = 0;


    }
    return right_down_line;
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
int left_up_guai(uint8_t imageSide[LCDH][2],int start,int end, uint8_t *flag)
{

    int i,t;
    int left_up_line=0;
    if(start > end)  //由上往下扫线
    {
        t=start;
        start=end;
        end=t;
    }
    if(start <= 5)
    {
        start = 5;
    }
    if(end >= LCDH - 5 )
    {
        end = LCDH - 5;
    }
    for(i=start;i<=end;i++)
    {
        if(left_up_line==0 &&
        (imageSide[i][0] - imageSide[i - 1][0]) >= -5 &&
        (imageSide[i - 1][0] - imageSide[i - 2][0]) >= -5 &&
        (imageSide[i - 2][0] - imageSide[i - 3][0]) >= -5 &&
        (imageSide[i][0] - imageSide[i + 2][0]) >= 5 &&
        (imageSide[i][0] - imageSide[i + 3][0]) >= 10 &&
        (imageSide[i][0] - imageSide[i + 4][0]) >= 10  )
        {
            left_up_line = i;
            *flag = 1;
            break;
        }
        else
            *flag = 0;
    }
    return left_up_line;
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
int right_up_guai(uint8_t imageSide[LCDH][2],int start,int end, uint8_t *flag)
{
    int i,t;
    int right_up_line=0;
    if(start > end)  //由上往下扫线
    {
        t=start;
        start=end;
        end=t;
    }
    if(start <= 5)
    {
        start = 5;
    }
    if(end >= LCDH - 5 )
    {
        end = LCDH - 5;
    }
    for(i=start;i<=end;i++)
       {
           if(right_up_line==0 &&
           (imageSide[i][1] - imageSide[i - 1][1]) <= 5 &&
           (imageSide[i - 1][1] - imageSide[i - 2][1]) <= 5 &&
           (imageSide[i - 2][1] - imageSide[i - 3][1]) <= 5 &&
           (imageSide[i][1] - imageSide[i + 2][1]) <= -5 &&
           (imageSide[i][1] - imageSide[i + 3][1]) <= -10 &&
           (imageSide[i][1] - imageSide[i + 4][1]) <= -10  )
       // if (1)
           {
//            printf("+++++ 1 = %d\n",(imageSide[i][1] - imageSide[i - 1][1]) );
//            printf("2 = %d\n",(imageSide[i - 1][1] - imageSide[i - 2][1]));
//            printf("3 = %d\n",(imageSide[i - 2][1] - imageSide[i - 3][1]));
//            printf("4 = %d\n", (imageSide[i][1] - imageSide[i + 2][1]));--
//            printf("5 = %d\n",(imageSide[i][1] - imageSide[i + 3][1]) );
//            printf("6 = %d\n",(imageSide[i][1] - imageSide[i + 4][1]));
////            printf("+++++ 1 = %d\n",(imageSide[i][1] ) );
////                  printf("2 = %d\n",(imageSide[i - 1][1] ));
////                  printf("3 = %d\n",(imageSide[i - 2][1] ));
////                  printf("4 = %d\n", (imageSide[i + 2][1]));
////                  printf("5 = %d\n",(imageSide[i + 3][1]) );
////                  printf("6 = %d\n",(imageSide[i + 4][1]));

               right_up_line = i;
               *flag = 1;
               break;

           }
           else
               *flag = 0;

       }
    return right_up_line;
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------

//void Crossroad_Find(uint8_t UpdowmSide[2][LCDW], uint8_t imageSide[LCDH][2], uint8_t Roadwide[LCDH], uint8_t *Flag)
//{
//    uint8_t i,  flag=0, flag_number;//j,
////    flag = 0;
////    int i = 15;
//    flag_number = 0;
//   // X_Find_Point(10, 70, UpdowmSide, leftup);
////    X_Find_Point(90, 158, UpdowmSide, rightup);
//    int r_up = right_up_guai(ImageSide,ROAD_END_ROW+15,110, &r_up_guaiflag);
//    int l_up = left_up_guai(ImageSide,ROAD_END_ROW+15,110, &l_up_guaiflag);
//    int r_d = right_down_guai(ImageSide,110,ROAD_END_ROW+5, &r_down_guaiflag);
//    int l_d = left_down_guai(ImageSide,110,ROAD_END_ROW+5, &l_down_guaiflag);
////    ImageAddingLine(imageSide, 1, leftup[0], leftup[1], 5, 110);
////    ImageAddingLine(imageSide, 2, rightup[0], rightup[1], 155, 110);
//        for (i = 100; i >= 40; i--)
//        {
//            if (imageSide[i][0] < 5 && imageSide[i][1] > 155)   //判断两端丢线 到达20则丢
//            {
//                flag_number++;
//            }
////
////                    if (flag_number > 20 && ((r_up_guaiflag == 1 && l_up_guaiflag == 0 && r_down_guaiflag == 0 && l_down_guaiflag == 1) ||
////                            (r_up_guaiflag == 0 && l_up_guaiflag == 1 && r_down_guaiflag == 1 && l_down_guaiflag == 0)))
////                    {
////                        flag = 2;             //只满足丢先
////                        printf("抽象情况且丢线\n");
////            //printf("只满足丢先\n");
////                        break;
////                    }
//                    if(flag_number > 20 && r_up_guaiflag == 1 && l_up_guaiflag == 1 && r_down_guaiflag == 1 && l_down_guaiflag == 1 && r_up < r_d && l_up < r_d)
//
//                    {
//                        flag = 3;
//                        printf("直入\n");
//                        ips200_draw_line((uint16)ImageSide[r_up][1],   (uint16)r_up,(uint16)ImageSide[r_d][1], (uint16)r_d, RGB565_BLACK);
//                        ips200_draw_line((uint16)ImageSide[l_up][0],   (uint16)l_up,(uint16)ImageSide[l_d][0], (uint16)l_d, RGB565_BLACK);
//                        ImageAddingLine(ImageSide,1,(uint16)ImageSide[l_up][0],   (uint16)l_up,(uint16)ImageSide[l_d][0], (uint16)l_d);
//                        ImageAddingLine(ImageSide,2,(uint16)ImageSide[r_up][1],   (uint16)r_up,(uint16)ImageSide[r_d][1], (uint16)r_d);
//
//                        break;
//
//                    }
//                    if(flag_number > 20 && r_up_guaiflag == 1 && l_up_guaiflag == 1 && r_down_guaiflag == 0 && l_down_guaiflag == 0 && flag !=3)
//                    {
//                        printf("只有上拐点\n");
//                        ips200_draw_line((uint16)ImageSide[r_up][1],   (uint16)r_up,155, 110, RGB565_BLACK);
//                        ips200_draw_line((uint16)ImageSide[l_up][0],   (uint16)l_up,5, 110, RGB565_BLACK);
//                        ImageAddingLine(ImageSide,1,(uint16)ImageSide[l_up][0],   (uint16)l_up,5, 110);
//                        ImageAddingLine(ImageSide,2,(uint16)ImageSide[r_up][1],   (uint16)r_up,155, 110);
//                        break;
//                    }
//
//        }
//}
uint8 round_found_flag=0;
uint8 Crossroad_Find(uint8_t UpdowmSide[2][LCDW], uint8_t imageSide[LCDH][2], uint8_t Roadwide[LCDH], uint8_t *Flag)
{
    if(round_found_flag)
        return 0;
    uint8_t i,flag_number;//j,flag,
    uint8_t zuodiuxian = 0;
    uint8_t youdiuxian = 0;

    flag_number = 0;

    int r_up = right_up_guai(ImageSide,ROAD_END_ROW+25,110, &r_up_guaiflag);
    int l_up = left_up_guai(ImageSide,ROAD_END_ROW+25,110, &l_up_guaiflag);
    int r_d = right_down_guai(ImageSide,110,ROAD_END_ROW+25, &r_down_guaiflag);
    int l_d = left_down_guai(ImageSide,110,ROAD_END_ROW+25, &l_down_guaiflag);
        for (i = 100; i >= 40; i--)
        {
            if (imageSide[i][0] < 5 && imageSide[i][1] > 155)   //判断两端丢线 到达20则丢
            {
                flag_number++;
            }
            if (imageSide[i][0] < 5)
            {
                zuodiuxian ++;
            }
            if (imageSide[i][1] > 155)
            {
                youdiuxian ++;
            }
                    if(flag_number > 20 && r_up_guaiflag == 1 && l_up_guaiflag == 1 && r_down_guaiflag == 1 && l_down_guaiflag == 1 && r_up < r_d && l_up < r_d)

                    {
                        *Flag = 3;

                        ImageAddingLine(ImageSide,1,(uint16)ImageSide[l_up][0]+9,   (uint16)l_up,(uint16)ImageSide[l_d][0]+9, (uint16)l_d);
                        ImageAddingLine(ImageSide,2,(uint16)ImageSide[r_up][1],   (uint16)r_up,(uint16)ImageSide[r_d][1], (uint16)r_d);

                        break;

                    }
                    if(flag_number > 20 && r_up_guaiflag == 1 && l_up_guaiflag == 1 && r_down_guaiflag == 0 && l_down_guaiflag == 0 )
                    {

                        ImageAddingLine(ImageSide,1,(uint16)ImageSide[l_up][0]+9,   (uint16)l_up,5+9, 110);
                        ImageAddingLine(ImageSide,2,(uint16)ImageSide[r_up][1],   (uint16)r_up,155, 110);
                        break;
                    }
                    if((flag_number > 20 || youdiuxian > 20) && l_up_guaiflag == 1&& r_up_guaiflag == 0 && l_down_guaiflag == 0 && r_down_guaiflag == 0 && l_up >10 )
                    {

                        ImageAddingLine(ImageSide,1,(uint16)ImageSide[l_up][0],   (uint16)l_up,1,115);

                        break;
                    }
                    if((flag_number > 20 || zuodiuxian > 20) && r_up_guaiflag == 1&& l_up_guaiflag == 0 && l_down_guaiflag == 0 && r_down_guaiflag == 0 && r_up > 10 )
                    {

                        ImageAddingLine(ImageSide,2,(uint16)ImageSide[r_up][1],   (uint16)r_up,159, 115);

                        break;
                    }
        }


return 1;
}
//void Crossroad_Find(uint8_t UpdowmSide[2][LCDW], uint8_t imageSide[LCDH][2], uint8_t Roadwide[LCDH], uint8_t *Flag)
//{
//    uint8_t i, j, flag, flag_number;
//    flag = 0;
//    flag_number = 0;
//    X_Find_Point(10, 70, UpdowmSide, leftup);
//    X_Find_Point(90, 158, UpdowmSide, rightup);
//
//    for (i = 5; i < 100; i++)
//    {
//        if ((UpdowmSide[0][i] - UpdowmSide[0][i + 4]) > 10 && UpdowmSide[0][i] < 80)
//        {
//            for (j = i; j < 155; j++)
//            {
//                if ((UpdowmSide[0][j] - UpdowmSide[0][j + 4]) < -10)
//                {
//                    flag = 1;
//
//                    break;
//                }
//            }
//        }
//        if (flag == 1)
//            break;
//    }
//    for (i = 100; i >= 40; i--)
//    {
//        if (imageSide[i][0] < 5 && imageSide[i][1] > 155)
//        {
//            flag_number++;
//        }
//        if (flag_number > 20 && flag == 0)
//        {
//            flag = 2;
//
//            break;
//        }
//        else if (flag_number > 20 && flag == 1)
//        {
//            flag = 3;
//
//            break;
//        }
//    }
//    if (flag == 1&& motor_flag == 1)//
//    {
//        *Flag = 1;
//    }
//    if (flag == 2&& motor_flag == 1 )//
//    {
//        *Flag = 2;
//        if (leftup[0] != 0 && rightup[0] != 0)
//        {
//            ImageAddingLine(imageSide, 1, leftup[0], leftup[1], 5, 110);
//            ImageAddingLine(imageSide, 2, rightup[0], rightup[1], 155, 110);
//        }
//        else
//        {
//            ImageAddingLine(imageSide, 1, 80, 10, 5, 110);
//            ImageAddingLine(imageSide, 2, 80, 10, 155, 110);
//        }
//    }
//    if (flag == 3&& motor_flag == 1 )//
//    {
//        *Flag = 3;
//        if (leftup[0] != 0 && rightup[0] != 0)
//        {
//            ImageAddingLine(imageSide, 1, leftup[0], leftup[1], 5, 110);
//            ImageAddingLine(imageSide, 2, rightup[0], rightup[1], 155, 110);
//        }
//        else
//        {
//            ImageAddingLine(imageSide, 1, 80, 10, 5, 110);
//            ImageAddingLine(imageSide, 2, 80, 10, 155, 110);
//        }
//    }
//    if (flag == 0)
//    {
//        *Flag = 0;
//    }
//    //ips200_show_int(0,250,Crossroad_Flag,3);
//}
/*!
 * @brief    补线处理
 *
 * @param    imageSide  : 边线
 * @param    status     : 1：左边线补线   2：右边线补线
 * @param    startX     : 起始点 列数
 * @param    startY     : 起始点 行数
 * @param    endX       : 结束点 列数
 * @param    endY       : 结束点 行数
 *
 * @return
 *
 * @note     endY 一定要大于 startY
 *
 * @see
 *
 * @date     2022/12/13
 */
void ImageAddingLine(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY)
{
    int i = 0;
    /* 直线 x = ky + b*/
    float k = 0.0f, b = 0.0f;
    switch (status)
    {
    case 1: // 左补线
    {
        k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
        b = (float)startX - (float)startY * k;

        for (i = startY; i < endY; i++)
        {
            imageSide[i][0] = (uint8_t)(k * i + b);
        }
        break;
    }
    case 2: // 右补线
    {
        k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
        b = (float)startX - (float)startY * k;

        for (i = startY; i < endY; i++)
        {
            imageSide[i][1] = (uint8_t)(k * i + b);
        }
        break;
    }
    }
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void ImageAddingray(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY)
{
    int i = 0;
    /* 直线 x = ky + b*/
    float k = 0.0f, b = 0.0f;
    switch (status)
    {
    case 1: // 左补线
    {
        k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
        b = (float)startX - (float)startY * k;

        for (i = 1; i < endY; i++)
        {
            imageSide[i][0] = (uint8_t)(k * i + b);
        }
        break;
    }
    case 2: // 右补线
    {
        k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
        b = (float)startX - (float)startY * k;

        for (i = 1; i < endY; i++)
        {
            imageSide[i][1] = (uint8_t)(k * i + b);
        }
        break;
    }
    }
}
/*!
 * @brief    判断是否是直道
 *
 * @param    image ： 左右边线
 *
 * @return   0：不是直道， 1：直道
 *
 * @note     思路：两边边线都单调
 *
 * @see
 *
 * @date     2022/11/29 星期二
 */
uint8_t leftState = 0, rightState = 0;
uint8_t RoadIsStraight(uint8_t imageSide[LCDH][2])
{
    uint8_t i = 0;
    leftState = 0;
    rightState = 0;

    /* 左边线是否单调 */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
    {
        if ((imageSide[i][0] >= imageSide[i + 1][0]) && (imageSide[i][0] > 10))
        {
            leftState++;
        }
    }
    /* 右边线是否单调 */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
    {
        if ((imageSide[i][1] <= imageSide[i + 1][1]) && (imageSide[i][1] < LCDW-5))
        {
            rightState++;
        }
    }
    if (leftState > 40 && rightState > 45)
    {
        return 1;
    }
    return 0;
}
/*!
 * @brief    判断左右边线是否单调
 * @param
 * @param
 * @param    imageIn ： 边线数组    Flag 0:左边线 1右边线
 *
 * @return   0：不单调 1：单调
 *
 * @note
 *
 * @see
 *
 * @date     2021/11/30 星期二
 */
#define straight 1
#define NOstraight 0
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
uint8_t RoadImageSide_Mono(uint8_t imageSide[LCDH][2], uint8_t Flag)
{
    uint8_t i = 0;
    uint8_t State;
    State = 0;
//    /* 左边线是否单调 */
//    if (Flag == 0)
//    {
//
//        /*从这里写直线判断*/
//        for (i = 110 - 1; i > 30; i--)
//                {
//                    if (abs(imageSide[i][0]-imageSide[i+1][0])<=2&&imageSide[i][0]>=imageSide[i+1][0]&&imageSide[i][0]>10)
//                    {
//                        State++;
//                    }
//                }
//    }
//    /* 右边线是否单调 */
//    if (Flag == 1)
//    {
//
//        for (i = 110 - 1; i > 30; i--)
//                        {
//                            if (abs(imageSide[i][1]-imageSide[i+1][1])<=2&&imageSide[i][1]<=imageSide[i+1][1])//&&(imageSide[i][1]<LCDW-10)
//                            {
//                                State++;
//                            }
//                        }
//    }
//    if (State > (110 - 30) * 0.7)
//        return 1;
//    else
//        return 0;
    if (Flag == 0)
    {
        for(i=110;i>=30;i--)
               {
                   if(abs(imageSide[i][0]-imageSide[i-1][0])>=5)//连续性阈值是5，可更改
                  {
                       State=i;
                       break;
                  }
               }

    }
    if (Flag == 1)
    {
         for(i=110;i>=30;i--)
        {
            if(abs(imageSide[i][1]-imageSide[i-1][1])>=5)//连续性阈值是5，可更改
           {
                State=i;
                break;
           }
        }

    }

    return State;
}




/*!
 * @brief    判断上边线是否单调
 * @param    X1 :起始X点
 * @param    X2 :终止X点              X1 < X2
 * @param    imageIn ： 边线数组
 *
 * @return   0：不单调or错误， 1：单调递增， 2：单调递减
 *
 * @note
 *
 * @see
 *
 * @date     2021/11/30 星期二
 */
uint8_t RoadUpSide_Mono(uint8_t X1, uint8_t X2, uint8_t imageIn[2][LCDW])
{
    uint8_t i = 0, num = 0;

    for (i = X1; i < X2 - 1; i++)
    {
        if (imageIn[0][i] >= imageIn[0][i + 1])
            num++;
        else
            num = 0;
        if (num >= (X2 - X1) * 4 / 5)
            return 1;
    }
    for (i = X1; i < X2 - 1; i++)
    {
        if (imageIn[0][i] <= imageIn[0][i + 1])
            num++;
        else
            num = 0;
        if (num >= (X2 - X1) * 4 / 5)
            return 2;
    }
    return 0;
}



//uint8_t RoadIsRoundabout(uint8_t Upimage[2][LCDW],  uint8_t image[LCDH][2], uint8_t *flag)
//{
//
//    uint8_t errL=0, errR=0;
//    uint8_t leftState = 0, rightState = 0;
//    uint8_t count = 0,num=0;
//    uint8_t  py=0;
//
//
//    for(uint8 i=80;i<100;i++)
//    {
//        if(Upimage[0][i]>10)//暂时修改为判断大于10超过十五个点大于十就是（弯道） 2
//        {
//            num++;
//        }
//    }
//    if(num>10)
//         {
//            num=0;
//            return 0;
//
//         }
//    if(RoadUpSide_Mono(20, LCDW-10, Upimage))//（弯道）是单调的 1
//        return 0;
//    if(Crossroad_Flag==1)
//        return 0;
//    leftState=RoadImageSide_Mono(image, 0);//左边界
//    rightState=RoadImageSide_Mono(image, 1);//右边界
//
//    errL = RoundaboutGetArc(image, 1, 5, &py);
//    errR = RoundaboutGetArc(image, 2,5, &py);
//
//    if(leftState == 1 && rightState == 0 && errR ==1)//&&l_state==1
//    {
//        count = 0;
//
//        if(RoundaboutGetArc(image, 2, 5, &count))//you检测 (5个连续增 且 5个连续减)
//        {
//
//            *flag = 1;
//            return 1;
//        }
//
//    }
//
//    // 右边单调， 检测左侧是否是环岛
//    if(rightState == 1 && leftState == 0&&errL==1)//
//    {
//        count = 0;
//        if(RoundaboutGetArc(image, 1, 5, &count))//左圆弧检测 (5个连续增 且 5个连续减)
//        {
//
//            *flag = 2;
//            return 2;
//        }
//    }
//    return 0;
//
//}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Down_Point(uint8 state)//找四个角点，返回值是角点所在的行数
{
    int i=0;
    int right_down_line=0;
    int left_down_line=0;
    switch(state)//1左2右
    {
        case 1:
                for(i=LCDH-5;i>=40;i--)
                {
                    if(left_down_line==0&&//只找第一个符合条件的点
                            abs(ImageSide[i][0]-ImageSide[i+1][0])<=5&&//角点的阈值可以更改,将角点的阈值从2到5
                            abs(ImageSide[i+1][0]-ImageSide[i+2][0])<=5&&
                            abs(ImageSide[i+2][0]-ImageSide[i+3][0])<=5&&
                            (ImageSide[i][0]-ImageSide[i-2][0])>5&&
                            (ImageSide[i][0]-ImageSide[i-3][0])>=10&&
                            (ImageSide[i][0]-ImageSide[i-4][0])>=10)
                    {
                        left_down_line=i;//获取行数即可
//                        ips200_draw_line(ImageSide[i][0], i, LCDW-1,i, RGB565_BLACK);

                        break;
                    }
                }
                return left_down_line;
            break;
        case 2:
            for(i=LCDH-5;i>=40;i--)
            {
                if(right_down_line==0&&//只找第一个符合条件的点
                        abs(ImageSide[i][1]-ImageSide[i+1][1])<=5&&//角点的阈值可以更改
                        abs(ImageSide[i+1][1]-ImageSide[i+2][1])<=5&&
                        abs(ImageSide[i+2][1]-ImageSide[i+3][1])<=5&&
                        (ImageSide[i][1]-ImageSide[i-2][1])<-5&&
                        (ImageSide[i][1]-ImageSide[i-3][1])<=-10&&
                        (ImageSide[i][1]-ImageSide[i-4][1])<=-10)
                {
                    right_down_line=i;//获取行数即可
                    //ips200_draw_line(ImageSide[right_down_line][1], right_down_line, LCDW-1,right_down_line, RGB565_BLACK);
                    //ips200_show_uint(50,240,ImageSide[right_down_line][1],3);
                    break;
                }
            }
            return right_down_line;
            break;
    }
return 0;
}
uint8 left_roadonetime=0;
uint8_t RoadIsRoundabout(uint8_t Upimage[2][LCDW],  uint8_t image[LCDH][2], uint8_t *flag)
{

    uint8_t errL=0, errR=0;
    uint8_t leftState = 0, rightState = 0;
    uint8_t  l_py=0,l_px=0,r_py=0,r_px=0;
    int left_down_x=0,left_down_y=0;
    int right_down_x=0,right_down_y=0;
    uint8 miss_number=0,miss_number1=0;
//    if(RoadUpSide_Mono(50, 130, Upimage))//（弯道）
//        return 0;
    //连续
    leftState=RoadImageSide_Mono(image, 0);//左边界 连续应该返回0
    rightState=RoadImageSide_Mono(image, 1);//右边界

    //丢线行
    miss_number=line_miss(image, 0,30);//由于上端丢线行较为严重，导致丢线数大于10，20变为30往下查丢线行
    miss_number1=line_miss(image, 1,30);

//右环岛
    if(leftState == 0  &&(miss_number<25))//15->20
    {
        right_down_y=Find_Down_Point(2);
        errR = RoundaboutGetArc(image, 2,5, &r_py,right_down_y-5);
        r_px=image[r_py][1];
        right_down_x=image[right_down_y][1];
        if( errR ==1&&right_down_y!=0&&((r_py-right_down_y)<-10))//&&r_px>85&&right_down_x>100暂时取消对点的范围限制
        {
            *flag = 1;
            seektype=left;
            return 1;
        }

    }
    // 左环岛
    if(rightState == 0 &&(miss_number1<30))//受小左环影响，丢线变为LCDW-3//丢线数变为<30
    {
        left_down_y=Find_Down_Point(1);
        errL = RoundaboutGetArc(image, 1, 5, &l_py,left_down_y-5);
        l_px=image[l_py][0];
        left_down_x=image[left_down_y][0];

        if(errL==1 &&left_down_y!=0&&((l_py-left_down_y)<-10))//原本只跑一次左环岛，现在取消只跑一次&&left_roadonetime==0//&&l_px<90&&left_down_x<70暂时取消对点的范围限制
        {
              *flag = 2;
              seektype=right;
              //left_roadonetime=1;  取消注释就只跑一次左环岛
            return 2;
        }
    }
    return 0;

}
#define sideoffset 0//为一时 使用边线偏移的办入环
#define try 0

uint8 offsetbasis=20;
//----------------------------------------------------------
//被用来抓圆环上拐点，for循环被改变
//
//
//----------------------------------------------------------
uint8_t ImageGetHop(uint8_t imageSide[LCDH][2], uint8_t state, uint8_t *x, uint8_t *y)
{
    int i = 0;
    uint8_t px = 0, py = 0;
    uint8_t count = 0;
    switch(state)
    {
      case 1:
        /* 寻找跳变点 */
        for(i = 70-1; i > ROAD_END_ROW; i--)
        {
            if(imageSide[i][0] <10 && i > (ROAD_END_ROW + 5))
            {
                count++;

                if(count > 10)
                {
                    if(imageSide[i-1][0] > (imageSide[i][0] + 20))
                    {
                        py = i - 1;
                        px = imageSide[py][0];
                        break;
                    }
                    if(imageSide[i-2][0] > (imageSide[i-1][0] + 15))
                    {
                        py = i - 2;
                        px = imageSide[py][0];
                        break;
                    }
                    if(imageSide[i-3][0] > (imageSide[i-2][0] + 15))
                    {
                        py = i - 3;
                        px = imageSide[py][0];
                        break;
                    }
                    if(imageSide[i-4][0] > (imageSide[i-3][0] + 15))
                    {
                        py = i - 4;
                        px = imageSide[py][0];
                        break;
                    }

                }

            }
            else
            {
                count = 0;
            }
        }

        if(py != 0)
        {
            *x = px;
            *y = py;
            return 1;
        }

        break;


      case 2:
        /* 寻找跳变点 */
        for(i = 70-1; i > ROAD_END_ROW; i--)
        {
            if(imageSide[i][1] >LCDW-10 && i > (ROAD_END_ROW + 5))
            {
                count++;

                if(count > 10)
                {
                    if(imageSide[i-1][1] < (imageSide[i][1] - 20))
                    {
                        py = i - 1;
                        px = imageSide[py][1];
                        break;
                    }
                    if(imageSide[i-2][1] < (imageSide[i-1][1] - 20))
                    {
                        py = i - 2;
                        px = imageSide[py][1];
                        break;
                    }
                    if(imageSide[i-3][1] < (imageSide[i-2][1] - 20))
                    {
                        py = i - 3;
                        px = imageSide[py][1];
                        break;
                    }
                    if(imageSide[i-4][1] < (imageSide[i-3][1] - 20))
                    {
                        py = i - 4;
                        px = imageSide[py][1];
                        break;
                    }

                }

            }
            else
            {
                count = 0;
            }
        }

        if(py != 0)
        {
            *x = px;
            *y = py;
            return 1;
        }

        break;
    }

    return 0;

}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void RoundaboutGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t status)
{
    uint8_t i = 0, j = 0;

    switch(status)
    {

        /* 左环岛 */
      case 1:
        {
            /* 重新确定左边界 */
            for(i = ROAD_START_ROW; i > ROAD_END_ROW; i--)
            {
                for(j = LCDW/2; j > 0; j--)
                {
                    if(!imageInput[i][j])
                    {
                        imageSide[i][0] = j;
                        break;
                    }
                }
            }
            break;
        }

      case 2:
        {
            /* 重新确定右边界 */
            for(i = ROAD_START_ROW; i > ROAD_END_ROW; i--)
            {
                for(j = LCDW/2; j < LCDW; j++)
                {
                    if(!imageInput[i][j])
                    {
                        imageSide[i][1] = j;
                        break;
                    }
                }
            }
            break;
        }
    }
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void Roundabout_Get_UpDowmSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[2][LCDW], uint8_t status)//重新获取上边线
{
    uint8_t i = 0, j = 0;

    switch(status)
    {
      case 1:
        {
            /* 重新确定上边界 */
            for(i = LCDW-1; i > 0; i--)
            {
                for(j = LCDH/2+10; j > 0; j--)
                {
                    if(!imageInput[j][i])
                    {
                        imageSide[0][i] = j;
                        break;
                    }
                }
            }
            break;
        }

      case 2:
        {
            /* 重新确定下边界 */
            for(i = LCDW-1; i > 0; i--)
            {
                for(j = LCDH/2; j < LCDH; j++)
                {
                    if(!imageInput[j][i])
                    {
                        imageSide[1][i] = j;
                        break;
                    }
                }
            }
            break;
        }
    }
}

float previous_yaw=0;
extern uint8_t gyro_flag;
float relative_difference=0;
uint8 const_y=0;
//宽度从30到110
uint8 roadwide[80]={0};
uint8 numadd=5;
//----------------------------------------------------------
//
uint8 nodownflag=0;
uint8 out_flag=0;
//
//
//----------------------------------------------------------
void roundabout(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t UpdowmSide[2][LCDW], uint8_t* state)

{
    uint8_t errL = 0,
            errR = 0,
//            i = 0,
//            j=0,
            num=0;
//            inc = 0,
//            dec = 0,
          //uint8 a=0,b=0;
    Lpointx = 0, Lpointy = 0;
   uint8 miss_number=0,miss_number1=0;

    //uint8 ldownpointx=0,ldownpointy=0;
    uint8 luppointx=0,luppointy=0,ruppointx=0,ruppointy=0;
    switch(* state)//偶数左环岛（2 4 6 8 10） \奇数右环岛（1 3 5 7 9）
    {
        case 1:     //当上拐点大于某个值就可以拐弯
            ruppointx=0,ruppointy=0;
           // ips200_show_char(20,200,'a');
                  UpSideErr(UpdowmSide,2,1,&ruppointx);
                  ruppointy=UpdowmSide[0][ruppointx];
                  if(ruppointy>45&&ruppointx<125&&ruppointy<70)//120->100
                  {
                      *state = 3;//准备进入环岛,暂时屏蔽观察数据
                      ImageAddingray(imageSide, 1, Rpointx+8, Rpointy+5, 45, 95);
                      gyro_flag=1;
                      seektype=normal;
                      previous_yaw=eulerAngle.yaw;
                  }

        break;
                ///偶数左环岛
        case 2 :
                //ips200_show_char(0,200,'A');//
                UpSideErr(UpdowmSide,1,1,&luppointx);
                luppointy=UpdowmSide[0][luppointx];
                if(luppointy>45&&luppointx>40)
                {
                   *state = 4;//准备进入环岛,暂时屏蔽观察数据
                  // ImageAddingray(imageSide, 2, luppointx, luppointy, LCDW-11, 100);//115->90
                   gyro_flag=1;
                  seektype=normal;
                   previous_yaw=eulerAngle.yaw;
                }
                break;
         case 3://进行拐弯如果陀螺仪偏了25就恢复正常巡线，不然就是一直拐弯
                            //ips200_show_char(20,220,'b');
                            Rpointx=0;
                            Rpointy=0;
                            UpSideErr(UpdowmSide,2,1,&Rpointx);
                            Rpointy=UpdowmSide[0][Rpointx];
            relative_difference = eulerAngle.yaw - previous_yaw;
               // 调整相对差值的周期性
               if (relative_difference < 0)
               {
                   relative_difference += 360; // 确保相对差值在0到360度之间
               }
                if(relative_difference <= 330&&relative_difference>250)//335->330
                  {
                    *state = 5;
                    seektype=normal;
                   // ips200_show_char(20,240,'c');
                  }
                 else
                 {
                  ImageAddingray(imageSide, 1, Rpointx+5, Rpointy, 40, 100);//补线入环

                 }
                 break;
        case 4:
                              //     ips200_show_char(0,220,'B');//补线入环，找上边界最低点补线进入
             UpSideErr(UpdowmSide,1,1,&Lpointx);
             Lpointy=UpdowmSide[0][Lpointx];

            relative_difference = eulerAngle.yaw - previous_yaw;
            // 调整相对差值的周期性
            if (relative_difference < 0)
            {
                relative_difference += 360; // 确保相对差值在0到360度之间
            }
             if(relative_difference >= 35&&relative_difference<100)//40->35
               {
                 *state = 6;
                 seektype=normal;
               //  ips200_show_char(0,240,'C');
               }
              else
              {
               ImageAddingray(imageSide, 2, Lpointx+5, Lpointy, LCDW-11, 100);//补线入环

              }
        break;
                case 5: //找点找得到就拉线，找不到就固定
                    Rpointy= 0;
                    Rpointx = 0;
                  relative_difference = eulerAngle.yaw - previous_yaw;
                                // 调整相对差值的周期性
                              if (relative_difference < 0)
                              {
                                relative_difference += 360; // 确保相对差值在0到360度之间
                               }
                              //寻找断点
                               if(relative_difference <= 150)
                                {

                                   Rpointy=(uint8)Find_Down_Point(1);
                                   Rpointx=imageSide[Rpointy][1];
                                   if(Rpointy!=0)
                                   {
                                      ImageAddingLine(imageSide, 1, 155, 5,Rpointx+25, Rpointy);
                                //      ips200_show_char(20,260,'d');//出环
                                   }
                                   else
                                   {
                                       if(relative_difference<140&&relative_difference>60)
                                                             {
                                                                 ImageAddingLine(imageSide, 1, 145, 5, 50, 90);
                                                             }
                                   }
                                }
                               if(relative_difference <= 50)
                               {
                                   *state = 7;
                               }
                             break;
        case 6:
            Lpointy=0;
            Lpointx=0;
            relative_difference = eulerAngle.yaw - previous_yaw;
               // 调整相对差值的周期性
             if (relative_difference < 0)
             {
               relative_difference += 360; // 确保相对差值在0到360度之间
              }
             //寻找断点，补射线，没有或者250强制出环
              if(relative_difference >= 238)
               {
                  Lpointy=(uint8)Find_Down_Point(2);
                  Lpointx=imageSide[Lpointy][1];
                  if(Lpointy!=0)
                  {
                      ImageAddingLine(imageSide, 2, 30, 5, Lpointx+5, Lpointy);//原本+10，+10
//                      ImageAddingLine(imageSide, 2, 10, 5, Lpointx-4, Lpointy+5);//原本+10，+10
                 //    ips200_show_char(0,260,'D');//出环
                  }
                  else
                  {
                      if(relative_difference >= 250)
                      {
                           ImageAddingLine(imageSide, 2, 30, 5, LCDW-25, 100);//LCDW-17

                      }
                  }

               }
              if(relative_difference >= 340)////320
              {
                  *state = 8;
                  seektype=right;
              }
            break;
                    case 7: //找上拐点拉线，当上拐点大于某个值或者陀螺仪小于某个值就结束

                        relative_difference = eulerAngle.yaw - previous_yaw;
                                      // 调整相对差值的周期性
                                    if (relative_difference < 0)
                                    {
                                      relative_difference += 360; // 确保相对差值在0到360度之间
                                    }
//                                    if(relative_difference<45)
//                                          {
//                                         //ImageAddingLine(imageSide, 2, 110, 15, LCDW-2, 119);
//                                      //   ips200_show_char(20,280,'e');
//
//                                          }
                                    errR = (uint8)right_up_guai(ImageSide,20,90, &num);
                                    if(errR)
                                    {
                                        ImageAddingray(imageSide, 2, imageSide[errR][0]-10, errL,imageSide[errR][0]+10, 119);

                                    }
                                    if(errR>70||relative_difference<15)
                                               {

                                                   *state=0;
                                                   gyro_flag=0;

                                               }

                    break;
        case 8:

//            relative_difference = eulerAngle.yaw - previous_yaw;
//              // 调整相对差值的周期性
//            if (relative_difference < 0)
//            {
//              relative_difference += 360; // 确保相对差值在0到360度之间
//            }
////            if(relative_difference >= 320||relative_difference <35)
////                  {
////
////                // ips200_show_char(0,280,'E');
////                 }
//
//            errL = (uint8)left_up_guai(ImageSide,20,90, &num);
//
//            if(errL)
//            {
//                ImageAddingLine(imageSide, 1, imageSide[errL][0]+17, errL,50, 119);
//
//
//            }
//            if(errL>90||(relative_difference>355||relative_difference<10))
//            {
//
//                *state=0;
//                seektype=normal;
//                gyro_flag=0;
//
//            }
            miss_number=line_miss(ImageSide, 0,30);
           // miss_number1=line_miss(ImageSide, 1,30);
            if(miss_number<35)
            {
                        *state=0;
                        seektype=normal;
                        gyro_flag=0;
            }
            break;

}
}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void zebra_panduan(unsigned char bin_image[LCDH][LCDW])
{
    //由上往下找线
    unsigned int count = 0;
    unsigned int count1 = 0;
    unsigned int count2 = 0;
    unsigned int count3 = 0;
    unsigned int count4 = 0;
    unsigned int count5 = 0;
    unsigned int count6 = 0;
    unsigned int tiaobiancishu =8;

static unsigned int stop = 0;
    for(int i = 0; i < LCDW - 6;i++)
    {

        if(bin_image[45][i - 1] == 0 && bin_image[45][i] == 1)
        {
             count++;
        }
        if(bin_image[40][i - 1] == 0 && bin_image[40][i] == 1)
               {
                    count1++;
               }
        if(bin_image[35][i - 1] == 0 && bin_image[35][i] == 1)
               {
                    count2++;
               }
        if(bin_image[50][i - 1] == 0 && bin_image[50][i] == 1)
               {
                    count3++;
               }
        if(bin_image[55][i - 1] == 0 && bin_image[55][i] == 1)
               {
                    count4++;
               }
        if(bin_image[100][i - 1] == 0 && bin_image[100][i] == 1)
               {
                    count5++;
               }
        if(bin_image[90][i - 1] == 0 && bin_image[90][i] == 1)
                      {
                           count6++;
                      }
        if(count == tiaobiancishu || count1 == tiaobiancishu || count2 == tiaobiancishu  || count3 == tiaobiancishu || count4 == tiaobiancishu
                || count5 == tiaobiancishu  || count6 == tiaobiancishu)
        {
            stop = 1;
            motor_flag = 2;
            break;
        }
    }
}



extern volatile sint16 Target_Speed1;
extern volatile sint16 Target_Speed2;
uint8 detect_round=1;
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void cameracar(void) // 在中断里处理
{

}
/*!
 * @brief    根据主跑行，求取舵机偏差
 *
 * @param
 *
 * @return
 *
 * @note
 *
 * @see    求几个偏差值，然后求平均值
 *
 * @date     2022/11/25
 */
//int16_t RoadGetSteeringError(uint8_t imageSide[LCDH][2], uint8_t lineIndex)
//{
//    int16_t sum = 0;
//    sum = (imageSide[lineIndex][0] + imageSide[lineIndex][1] - LCDW-1) +
//          (imageSide[lineIndex - 6][0] + imageSide[lineIndex - 6][1] - LCDW-1) +
//          (imageSide[lineIndex - 12][0] + imageSide[lineIndex - 12][1] - LCDW-1) +
//          (imageSide[lineIndex - 18][0] + imageSide[lineIndex - 18][1] - LCDW-1) +
//          (imageSide[lineIndex - 24][0] + imageSide[lineIndex - 24][1] - LCDW-1);
//    sum = sum / 5;
//    return sum;
//}

//int16_t midlineerror(uint8_t imageSide[LCDH][2], uint8_t lineIndex)
//{
//
//    int16_t sum = 0;
//       int e1,e2,e3,e4,e5=0;
//    e1=(imageSide[lineIndex][0]+ imageSide[lineIndex][1])/2;
//    e2=(imageSide[lineIndex - 6][0] + imageSide[lineIndex - 6][1])/2;
//    e3=(imageSide[lineIndex - 12][0] + imageSide[lineIndex - 12][1])/2;
//    e4=(imageSide[lineIndex - 18][0] + imageSide[lineIndex - 18][1])/2;
//    e5=(imageSide[lineIndex - 24][0] + imageSide[lineIndex - 24][1])/2;
//    sum=(80-e1)+(80-e2)+(80-e3)+(80-e4)+(80-e5);
//    sum=sum/5;
//    return sum;
//
//}
/*************************************************************************
 *  函数名称：void Get_Errand(void)
 *  功能说明：获得图像偏差
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2022年12月21日
 *  备   注：
 *************************************************************************/
//加权控制
const uint8 Weight_list[LCDH]=
{
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //图像最远端00 ——09 行权重
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               //图像最远端10 ——19 行权重
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //图像最远端20 ——29 行权重
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //图像最远端30 ——39 行权重
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //图像最远端40 ——49 行权重
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //图像最远端50 ——59 行权重
        3, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //图像最远端60 ——69 行权重
        4, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //70-79
        3, 0, 0, 0, 0, 0, 0, 0, 0, 0,               //80-89
        2, 0, 0, 0, 0, 0, 0, 0, 0, 0,               //90-99
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0,               //100-119
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
const uint8 Weight_list1[LCDH]=
{
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //图像最远端00 ——09 行权重
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               //图像最远端10 ——19 行权重
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //图像最远端20 ——29 行权重
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //图像最远端30 ——39 行权重
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //图像最远端40 ——49 行权重
        3, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //图像最远端50 ——59 行权重
        4, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //图像最远端60 ——69 行权重
        3, 0, 0, 0, 0, 0, 0, 0, 0, 0,              //70-79
        2, 0, 0, 0, 0, 0, 0, 0, 0, 0,               //80-89
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0,               //90-99
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               //100-119
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
//误差选取的范围距离车越近，车模反应越慢，从而越切外；
//选取的范围越远，反应越提前，车模越切内
//----------------------------------------------------------
const uint8 ROADWIDE[101]={20,
        22,
        23,
        24,
        26,
        27,
        29,
        29,
        31,
        33,
        33,
        35,
        39,
        41,
        41,
        40,
        41,
        42,
        43,
        45,
        46,
        49,
        49,
        51,
        53,
        53,
        55,
        57,
        57,
        59,
        61,
        61,
        63,
        64,
        65,
        67,
        68,
        69,
        70,
        71,
        73,
        73,
        75,
        77,
        77,
        79,
        81,
        81,
        83,
        84,
        85,
        87,
        88,
        89,
        91,
        92,
        93,
        95,
        96,
        97,
        99,
        100,
        101,
        103,
        104,
        105,
        106,
        108,
        109,
        110,
        112,
        112,
        114,
        115,
        116,
        118,
        119,
        120,
        122,
        122,
        124,
        125,
        126,
        128,
        128,
        130,
        132,
        132,
        134,
        135,
        137,
        138,
        138,
        140,
        140,
        141,
        142,
        143,
        144,
        145};
//获取中线
//变量
uint8 midline[LCDH]={0};

//
//----------------------------------------------------------
void get_midline(void)
{

    if(seektype==normal)
    {
            for(uint8 i=LCDH-1;i>=1;i--)//常规误差计算
        {
        midline[i]=(ImageSide[i][0]+ImageSide[i][1])>>1;//右移1位，等效除2

        }
    }
   if(seektype==right)
   {
           for(uint8 i=LCDH-20;i>=10;i--)
    {
        midline[i]=ImageSide[i][1]-(ROADWIDE[i]>>1)+10;
    }
   }
   if(seektype==left)
   {
       for(uint8 i=LCDH-20;i>=10;i--)
           {
               midline[i]=ImageSide[i][0]+(ROADWIDE[i]>>1)-5;
           }
   }

    //不补线寻单边

}
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
float Err_Sum(void)
{
    int i;
    float err=0;
    float weight_count=0;

         //常规误差
    for(i=LCDH-1;i>=1;i--)//常规误差计算
    {
        err+=(LCDW/2-(midline[i]))*Weight_list[i];//右移1位，等效除2
        weight_count+=Weight_list[i];
    }



    err=err/weight_count;
    return err;
}
extern volatile sint16 Target_Speed1;
extern volatile sint16 Target_Speed2;
//----------------------------------------------------------
//
//
//
//----------------------------------------------------------
void Get_Errand(void)
{


            g_sSteeringError = Err_Sum();//ImageSide, ROAD_MAIN_ROW
    if ((g_sSteeringError < 60) && (g_sSteeringError > -60))
    {
        if ((g_sSteeringError < 30) && (g_sSteeringError > -30))
        {
            Servo_P = 11;
//            Target_Speed1=130;
//            Target_Speed2=130;
        }
        else
        {
            Servo_P = 14;
//            Target_Speed1=100;
//            Target_Speed2=100;
        }
    }
    else
    {
        Servo_P = 11;
//        Target_Speed1=90;
//        Target_Speed2=90;
    }
    error_servo = g_sSteeringError * Servo_P / 10;
    // 偏差限幅
    if (error_servo > 100)
        error_servo = 100;
    if (error_servo < -100)
        error_servo = -100;

}
////----------------------------------------------------------
////限幅函数
////
////
////----------------------------------------------------------
//int clip(int x, int low, int up) {
//    return x > up ? up : x < low ? low : x;
//}

////----------------------------------------------------------
////点集三角滤波
////
////
////----------------------------------------------------------
//void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel)
//{
//
//    //左边线
//    int half = kernel / 2;
//    for (int i = 0; i < num; i++)
//    {
//        pts_out[i][0] = pts_out[i][1] = 0;
//        for (int j = -half; j <= half; j++)
//        {
//            pts_out[i][0] += pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
//            pts_out[i][1] += pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
//        }
//        pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
//        pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
//    }
//}

////----------------------------------------------------------
//
////----------------------------------------------------------

////----------------------------------------------------------
//
//
////----------------------------------------------------------
//
////----------------------------------------------------------
////搜索边线
////
////
////----------------------------------------------------------

////----------------------------------------------------------
//// 八邻域获取左右边线
////
////
////----------------------------------------------------------


////----------------------------------------------------------
//// 八邻域总的处理函数
////
////
////----------------------------------------------------------

////----------------------------------------------------------
////
////
////
////----------------------------------------------------------




