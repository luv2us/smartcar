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
#include "ENA.h"

uint8_t r_up_guaiflag;
uint8_t l_up_guaiflag;
uint8_t r_down_guaiflag;
uint8_t l_down_guaiflag;
uint8_t guai_Flag = 0;
fifo_obj_struct camera_receiver_fifo;                               // 定义摄像头接收数据fifo结构体
uint8 camera_receiver_buffer[CAMERA_RECEIVER_BUFFER_SIZE];          // 定义摄像头接收数据缓冲区
uint8 camera_send_image_frame_header[4] = {0x00, 0xFF, 0x01, 0x01}; // 定义摄像头数据发送到上位机的帧头
/** 压缩后之后用于存放屏幕显示数据  */
unsigned char Image_Use[LCDH][LCDW];
unsigned char Threshold = 0;
unsigned char Bin_Image[LCDH][LCDW];
/** 二值化后用于OLED显示的数据 */

uint8_t ImageSide[LCDH][2];      // 左右边线数组
uint8_t ImageSide_last[LCDH][2]; // 上张图片左右边线数组
uint8_t UpdowmSide[2][LCDW];     // 上下边线数组
uint8_t Roadwide[LCDH];          // 赛道宽度
extern uint8_t can ;
/**  @brief    主跑行  */
#define ROAD_MAIN_ROW 44

/**  @brief    使用起始行  */
#define ROAD_START_ROW 115

/**  @brief    使用结束行  */
#define ROAD_END_ROW 10
extern unsigned char motor_flag;
//
uint8_t Crossroad_Flag = 0;
//
uint8_t leftup[2];
uint8_t rightup[2];
uint8_t R_CircleFlag = 0; // 右环岛标志位
uint8_t L_CircleFlag = 0; // 左环岛标志位
uint8_t miss_Flag = 0;    // 丢线标志位 1左丢 2右丢
uint8_t turnonpoint[2][3];

uint8_t Lpointx = 0, Lpointy = 0, Rpointx = 0, Rpointy = 0;

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
float my_abs(float x)
{
    if (x >= 0)
        return x;
    else
        return -x;
}
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
void Get_Use_Image(void)
{
    short i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < LCDH; i++)
    {
        for (j = 0; j <= LCDW; j++)
        {
            Image_Use[row][line] = mt9v03x_image[i][j + 14];
            line++;
        }
        line = 0;
        row++;
    }
}
short GetOSTU(unsigned char tmImage[LCDH][LCDW])
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    unsigned char HistoGram[256]; //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; // 初始化灰度直方图

    for (j = 0; j < LCDH; j++)
    {
        for (i = 0; i < LCDW; i++)
        {
            HistoGram[tmImage[j][i]]++; // 统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++)
        ; // 获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--)
        ; // 获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue; // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue; // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j]; //  像素总数

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j; // 灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];                                              // 前景像素点数
        PixelFore = Amount - PixelBack;                                                    // 背景像素点数
        OmegaBack = (float)PixelBack / Amount;                                             // 前景像素百分比
        OmegaFore = (float)PixelFore / Amount;                                             // 背景像素百分比
        PixelshortegralBack += HistoGram[j] * j;                                           // 前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;                       // 背景灰度值
        MicroBack = (float)PixelshortegralBack / PixelBack;                                // 前景灰度百分比
        MicroFore = (float)PixelshortegralFore / PixelFore;                                // 背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore); // 计算类间方差
        if (Sigma > SigmaB)                                                                // 遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold; // 返回最佳阈值;
}
/*
double computeEntropy(int histogram[], int totalPixels)
{
    double entropy = 0.0;

    for (int i = 0; i < 256; i++)
    {
        if (histogram[i] > 0)
        {
            double probability = (double)histogram[i] / totalPixels;
            entropy -= probability * log2(probability);
        }
    }

    return entropy;
}

*/
/*
// Kapur算法选择阈值
int kapurThreshold(uint8 tmImage[LCDH][LCDW])
{
    int bestThreshold = 0;
    double maxEntropy = 0.0;
    int totalPixels = LCDW * LCDH;
    unsigned char HistoGram[256];


    //计算灰度直方图
    for (int j = 0; j < LCDH; j++)
    {
        for (int i = 0; i < LCDW; i++)
        {
            HistoGram[tmImage[j][i]]++; // 统计灰度级中每个像素在整幅图像中的个数
        }
    }
    //计算结束

    for (int t = 0; t < 256; t++)
    {
        int histogramLow[256] = {0};
        int histogramHigh[256] = {0};

        // 分别计算低于和高于阈值t的像素数目
        int totalLow = 0;
        int totalHigh = 0;
        for (int i = 0; i < 256; i++)
        {
            if (i <= t)
            {
                histogramLow[i] = HistoGram[i];
                totalLow += HistoGram[i];
            }
            else
            {
                histogramHigh[i] = HistoGram[i];
                totalHigh += HistoGram[i];
            }
        }

        // 计算低于和高于阈值t的信息熵
        double entropyLow = computeEntropy(histogramLow, totalLow);
        double entropyHigh = computeEntropy(histogramHigh, totalHigh);

        // 计算总的信息熵
        double totalEntropy = (totalLow * entropyLow + totalHigh * entropyHigh) / totalPixels;

        // 更新最大信息熵及对应的阈值
        if (totalEntropy > maxEntropy)
        {
            maxEntropy = totalEntropy;
            bestThreshold = t;
        }
    }

    return bestThreshold;
}
*/

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

unsigned char my_adapt_threshold(uint8 *image, uint16 col, uint16 row)
{

    uint16 width = col;
    uint16 height = row;

    int i, j, pixelSum = width * height / 4;

    uint8 *data = image; // 指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32 gray_sum = 0;
    // 统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i += 2)
    {
        for (j = 0; j < width; j += 2)
        {
            pixelCount[(int)data[i * width + j]]++; // 将当前的点的像素值作为计数数组的下标
            gray_sum += (int)data[i * width + j];   // 灰度值总和
        }
    }

    // 计算每个像素值的点在整幅图像中的比例

    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }

    // 遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;

    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)
    {

        w0 += pixelPro[j];        // 背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j]; // 背景部分       每个灰度值的点的比例 *灰度值

        w1 = 1 - w0;
        u1tmp = gray_sum / pixelSum - u0tmp;

        u0 = u0tmp / w0;   // 背景平均灰度
        u1 = u1tmp / w1;   // 前景平均灰度
        u = u0tmp + u1tmp; // 全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            Threshold = (unsigned char)j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    return Threshold;
}
#define HISTOGRAM_SIZE 256

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
int otsuThreshold(unsigned char image[LCDH][LCDW])
{
    int histogram[HISTOGRAM_SIZE] = {0};
    computeHistogram(image, histogram);

    int totalPixels = LCDH * LCDW;
    double sum = 0.0, sumB = 0.0;
    double varMax = 0.0;
    int threshold = 0;

    // 计算灰度平均值
    for (int t = 0; t < HISTOGRAM_SIZE; t++)
    {
        sum += t * histogram[t];
    }

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

        varBetween = (double)wB * (double)wF * (meanB - meanF) * (meanB - meanF);

        if (varBetween > varMax)
        {
            varMax = varBetween;
            threshold = t;
        }
    }

    return threshold;
}
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

#define white 1
#define black 0
void Get_Bin_Image(uint8 mode) // 对图像二值化
{

    // unsigned short i = 0, j = 0;
    switch (mode)
    {
    case 1: // 大津法阈值
            // Threshold = (unsigned char)GetOSTU(Image_Use); /* longqiu */
        // my_adapt_threshold(Image_Use[0], LCDW, LCDH);
        Threshold = (unsigned char)otsuThreshold1(Image_Use);
        for (int i = 1; i < LCDH - 1; i++)
        {
            for (int j = 1; j < LCDW - 1; j++)
            {
                if (Image_Use[i][j] > Threshold) // 数值越大，显示的内容越多，较浅的图像也能显示出来
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
    imageOut[lineIndex][1] = 159;
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

    for (i = last; i < 159; i++)
    {
        if (!imageInput[lineIndex][i])
        {
            imageOut[lineIndex][1] = i;
            break;
        }
    }
    if (i == 159)
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
            for (i = imageOut[lineIndex][0]; i < 159; i++)
            {
                count++;
                for (j = lineIndex; j > ROAD_END_ROW && lineIndex > count; j--)
                {
                    if (imageInput[j][i])
                    {
                        imageOut[lineIndex - count][0] = i;
                        imageOut[lineIndex - count][1] = 159;
                        break;
                    }
                }
            }
            break;
        }
    }
//int line=0;
//int list=0;
//void image_scan()
//{
// for(line=120;line>=40;line--)
// {
//   for(list=93;list<188;list++)
//   {
//    if((image_deal[line][list-2]==white)&&(image_deal[line][list-1]==black)
//       &&(image_deal[line][list]==black))
//     {
//       rightline[line]=list;
//        break;
//     }
//   }
//
//   for(list=93;list>1;list--)
//   {
//    if((image_deal[line][list]==black)&&(image_deal[line][list+1]==black)
//        &&(image_deal[line][list+2]==white))
//     {
//       leftline[line]=list;
//      break;
//     }
//   }
//    road_width[line]=my_abs(leftline[line],rightline[line]);
//    centerline[line]=(rightline[line]+leftline[line])/2;
//  }
//}
uint8_t right_line_lost,left_line_lost;
uint8_t Right_Lost_Flag[LCDH]={0};
uint8_t Left_Lost_Flag[LCDH]={0};
void Find_Boundry(uint8_t imageOut[LCDH][2])//从中间往两边搜索中线
{
    int i,j;
    static int left_border=0,right_border=0,mid=LCDW/2,last_mid=LCDW/2;
    right_line_lost=0;
    left_line_lost=0;
    //起始点合理性判断
    if(Bin_Image[MT9V03X_H-1][LCDW/2]==0x00)//屏幕中线是黑的话
    {
        if(Bin_Image[MT9V03X_H-1][LCDW/4]==0xff)//看看左1/4是不是白
        {
            last_mid=LCDW/4;//更改搜索起始点
        }
        else if(Bin_Image[MT9V03X_H-1][LCDW/4*3]==0xff)//看看右1/4是不是白
        {
            last_mid=LCDW/4*3;//更改搜索起始点
        }
    }
    //开始巡边
    for(i=MT9V03X_H-1;i>=0;i--)//从最底下往上扫描
    {
        for(j=last_mid;j<LCDW-3;j++)//往右扫描
        {
            if(Bin_Image[i][j]==0xff&&Bin_Image[i][j+1]==0x00&&Bin_Image[i][j+2]==0x00)//白黑黑，找到右边界
            {
                right_border=j;
                Right_Lost_Flag[i]=0; //右丢线数组，丢线置1，不丢线置0
                break;//跳出，找到本行边界就没必要循环下去了
            }
            else
            {
                right_border=j;//没找到右边界，把屏幕最右赋值给右边界
                Right_Lost_Flag[i]=1; //右丢线数组，丢线置1，不丢线置0
            }
        }
        right_line_lost+=Right_Lost_Flag[i];
        for(j=last_mid;j>1;j--)//往左边扫描
        {
            if(Bin_Image[i][j]==0xff&&Bin_Image[i][j-1]==0x00&&Bin_Image[i][j-2]==0x00)//黑黑白认为到达左边界
            {
                left_border=j;
                Left_Lost_Flag[i]=0; //左丢线数组，丢线置1，不丢线置0
                break;//跳出，找到本行边界就没必要循环下去了
            }
            else
            {
                left_border=j;//找到头都没找到边，就把屏幕最左右当做边界
                Left_Lost_Flag[i]=1; //左丢线数组，丢线置1，不丢线置0
            }
        }
        left_line_lost+=Left_Lost_Flag[i];
        mid=(left_border+right_border)/2;//中线坐标
        last_mid=mid;//中线查找开始点，方便下一次找中线
        imageOut[i][0]= (uint8_t)left_border ;      //左边线线数组
        imageOut[i][1]= (uint8_t)right_border;      //右边线线数组
        Roadwide[i]=imageOut[i][1]-imageOut[i][0];
    }
}

uint8_t ImageGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t imageOut_last[LCDH][2]) // 获取左右边线
{
    uint8_t i = 0, j = 0;

    RoadIsNoSide(imageInput, imageOut, ROAD_START_ROW); // ？存疑
    for (i = 0; i <= LCDH; i++)
    {
        imageOut_last[i][0] = imageOut[i][0];
        imageOut_last[i][1] = imageOut[i][1];
    }

    /* 离车头近的40行 寻找边线 */
    for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
    {
        imageOut[i][0] = 0;
        imageOut[i][1] = 159;

        /* 根据边界连续特性 寻找边界 */
        for (j = imageOut[i + 1][0] + 10; j > 0; j--)
        {
            if (!imageInput[i][j])
            {
                imageOut[i][0] = j;
                break;
            }
        }
        for (j = imageOut[i + 1][1] - 10; j < 160; j++)
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

        // 边线数组发生突变
        if ((imageOut_last[110][0] - imageOut[110][0]) > 20 && (imageOut_last[110][0] - imageOut[110][0]) < -20)
        {
            for (i = 0; i < LCDH; i++)
            {
                imageOut[i][0] = imageOut_last[i][0];
                imageOut[i][1] = 158;
            }
        }
        if (imageOut_last[110][1] - imageOut[110][1] > 20 && imageOut_last[110][1] - imageOut[110][1] < -20)
        {
            for (i = 0; i <= LCDH; i++)
            {
                imageOut[i][1] = imageOut_last[i][1];
                imageOut[i][0] = 1;
            }
        }
    }
    return 0;
}
void imageside(uint8 mode)
{
    switch (mode)
    {
    case 1:
        ImageGetSide(Bin_Image, ImageSide, ImageSide_last);
        break;
    case 2:
        enh_process();
        break;
    default:
        break;
    }

}

/*-----------------------------------------------------------------!
  * @brief    获取边线
  *
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  *
  * @return   是否丢线
  *
  * @note     思路：从距离车头较近的行开始从中间向两边搜线
 ---------------------------------------------------------------------- */
uint8_t ImageGetSide1(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2])
{
    uint8_t i = 0, j = 0, k = 0;

    for (i = LCDH - 1; i > 0; i--)
    {
        imageOut[i][0] = 1;
        imageOut[i][1] = LCDW - 1;
    }

    // 用距离小车比较近的行 判断是否丢线
    for (i = LCDW / 2; i > 1; i--)
    {
        if (!imageInput[LCDH - 1][i])
        {
            imageOut[LCDH - 1][0] = i;
            break;
        }
    }

    for (i = LCDW / 2 + 1; i < LCDW - 1; i++)
    {
        if (!imageInput[LCDH - 1][i])
        {
            imageOut[LCDH - 1][1] = i;
            break;
        }
    }
    //-------------------------------------------------------------

    // 遍历每行
    for (i = LCDH - 2; i > 0; i--)
    {
        // 向左搜线
        for (j = imageOut[i + 1][0] + 10; j > 0; j--)
        {
            if (!imageInput[i][j])
            {
                imageOut[i][0] = j;
                break;
            }
        }

        if (imageOut[i][0] > LCDW / 2)
        {
            imageOut[i][0] = LCDW / 2;

            for (k = i; k > 0; k--)
            {
                imageOut[k][0] = LCDW / 2;
            }

            break;
        }
    }

    // 遍历每一行
    for (i = LCDH - 2; i > 0; i--)
    {
        // 向右搜线
        for (j = imageOut[i + 1][1] - 10; j < LCDW - 1; j++)
        {
            if (!imageInput[i][j])
            {
                imageOut[i][1] = j;
                break;
            }
        }

        if (imageOut[i][1] < LCDW / 2)
        {
            imageOut[i][1] = LCDW / 2;

            for (k = i; k > 0; k--)
            {
                imageOut[k][1] = LCDW / 2;
            }

            break;
        }
    }
    return 0;
}
/*-----------------------------------------------------------------!
  * @brief    dfs获取边线
  *
  * @param
  * @param
  *
  * @return
  *
  * @note
 ---------------------------------------------------------------------- */
//bool visited[LCDH][LCDW]={False};
//void dfs(int i, int j, uint8 imageInput[LCDH][LCDW], int *num_points)
//{
//    if (i < 0 || i >= LCDH || j < 0 || j >= LCDW || imageInput[i][j] == 255 || visited[i][j])
//    {
//        return;
//    }
//
//    visited[i][j] = true;
//    ImageSide[*num_points][0] = i;
//    ImageSide[*num_points][1] = j;
//    (*num_points)++;
//
//    // 对当前像素的八个邻域进行搜索
//    dfs(i - 1, j - 1, ImageSide, num_points);
//    dfs(i - 1, j, ImageSide, num_points);
//    dfs(i - 1, j + 1, ImageSide, num_points);
//    dfs(i, j - 1, ImageSide, num_points);
//    dfs(i, j + 1, ImageSide, num_points);
//    dfs(i + 1, j - 1, ImageSide, num_points);
//    dfs(i + 1, j, ImageSide, num_points);
//    dfs(i + 1, j + 1, ImageSide, num_points);
//}
uint8_t UpdownSideGet(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[2][LCDW]) // 获取上下边线
{
    uint8_t i = 0, j = 0;
    uint8_t last = 60;

    imageOut[0][159] = 0;
    imageOut[1][159] = 119;
    /* 用中线比较近的行 判断是否丢线 */
    for (i = last; i >= 0; i--)
    {
        if (!imageInput[i][80])
        {
            imageOut[0][80] = i;
            break;
        }
    }

    for (i = last; i < 120; i++)
    {
        if (!imageInput[i][80])
        {
            imageOut[1][80] = i;
            break;
        }
    }

    /* 中线往左 寻找边线 */
    for (i = 80 - 1; i > 0; i--)
    {
        imageOut[0][i] = 0;
        imageOut[1][i] = 119;

        /* 根据边界连续特性 寻找边界 */
        for (j = imageOut[0][i + 1] + 10; j > 0; j--)
        {
            if (!imageInput[j][i])
            {
                imageOut[0][i] = j;
                break;
            }
        }
        for (j = imageOut[1][i + 1] - 10; j < 120; j++)
        {
            if (!imageInput[j][i])
            {
                imageOut[1][i] = j;
                break;
            }
        }
    }
    /*中线往右 寻找边线*/
    for (i = 80 + 1; i < 159; i++)
    {
        imageOut[0][i] = 0;
        imageOut[1][i] = 119;

        /* 根据边界连续特性 寻找边界 */
        for (j = imageOut[0][i - 1] + 10; j > 0; j--)
        {
            if (!imageInput[j][i])
            {
                imageOut[0][i] = j;
                break;
            }
        }
        for (j = imageOut[1][i - 1] - 10; j < 120; j++)
        {
            if (!imageInput[j][i])
            {
                imageOut[1][i] = j;
                break;
            }
        }
    }
    return 0;
}
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
    }
    return 0;
}
void ips200_bin_roadside(uint8_t imageOut[LCDH][2])
{
//    uint8_t image[LCDH][2] = {0};
//    memcpy(image,imageOut,sizeof(imageOut));
    uint8_t i = 0;
    for (i = 10; i < 110; i++)
    {
//        if(imageOut[i][0]>159)imageOut[i][0]=159;
//        if(imageOut[i][0]<=159)imageOut[i][0]=0;
        ips200_draw_point(imageOut[i][0], i, RGB565_GREEN);
        ips200_draw_point(imageOut[i][1], i, RGB565_GREEN);
    }
}
void ips200_show_side_image (uint16 x, uint16 y,   uint16 dis_width, uint16 dis_height)
{
    uint16 color_buffer[dis_width];
    uint16 i = 0, j = 0;

    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(0);
    }
    ips200_set_region(0, 0, dis_width - 1, dis_height - 1);
    for (j = 0; j < dis_height; j ++)
    {
    for(i = 0; i < dis_width; i ++)
    {
        if(i==ImageSide[j][0])
        {
            color_buffer[i]=ips200_pencolor;
        }
        else if((i==ImageSide[j][1]))
        {
            color_buffer[i]=ips200_pencolor;
        }
        else
        {
              color_buffer[i] = ips200_bgcolor;
        }

    }

        ips200_write_16bit_data_array(color_buffer, dis_width);
    }
    if(IPS200_TYPE_SPI == ips200_display_type)
    {
        IPS200_CS(1);
    }
}
void ips200_bin_updownside(uint8_t imageOut[2][LCDW])
{

    uint8_t i = 0;
    for (i = 0; i < LCDW; i++)
    {
        ips200_draw_point(i, imageOut[0][i], RGB565_BLUE);
        ips200_draw_point(i, imageOut[1][i],  RGB565_MAGENTA);
    }
}
void ips200_bin_middleline(uint8_t imageOut[LCDH][2]) //画中线
{
    uint8_t i = 0;
    for (i = 10; i < 110; i++){
        ips200_draw_point(imageOut[i][0]+ (uint8_t)((Roadwide[i])/2), i, RGB565_YELLOW);

    }
}
void Show_Camera_Info(void)
{

//    ips200_show_gray_image(0, 0, Image_Use[0], LCDW, LCDH, LCDW, LCDH, Threshold);
//     ips200_draw_line(80, 0, 80, 119, RGB565_PURPLE);
//    ips200_bin_middleline(ImageSide);
    ips200_bin_roadside(ImageSide);
//    ips200_bin_updownside(UpdowmSide);

}

void FindCarbarn(uint8_t imageInput[LCDH][LCDW], uint8_t *Flag) // 斑马线
{
    uint8_t i, Flag_number;
    Flag_number = 0;
    for (i = 5; i <= LCDW - 5; i++)
    {
        if (imageInput[50][i] != imageInput[50][i + 1])
        {
            Flag_number++;
        }
    }
    if (Flag_number > 10)
    {
        *Flag = 1;
    }
}
void ZebraProcess(uint8_t imageSide[LCDH][2], uint8_t state, uint8_t *motor)
{
    static uint16_t count = 0;
    count++;
    if (state == 1)
    {
        imageSide[ROAD_MAIN_ROW][0] = 0;
        imageSide[ROAD_MAIN_ROW][1] = LCDW / 2;
    }
    else
    {
        imageSide[ROAD_MAIN_ROW][0] = LCDW / 2;
        imageSide[ROAD_MAIN_ROW][1] = LCDW - 1;
    }
    if (count > 100)
    {
        *motor = 0;
        while (1)
            ;
    }
}
void Carbarn(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t *motor_flag, uint8_t *Flag)
{
    switch (*Flag)
    {
    case 0:
        FindCarbarn(imageInput, &*Flag); // 车库识别
        break;
    case 1:
        ZebraProcess(imageSide, 1, &*motor_flag); // 进入车库
        break;
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
uint8 Monotonicity_Change_left(uint8 start,uint8 end)//单调性改变，返回值是单调性改变点所在的行数
{
    uint8 i;
    uint8 monotonicity_change_line=0;

//    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
//        return monotonicity_change_line;
//    if(start>=MT9V03X_H-1-5)//数组越界保护
//        start=MT9V03X_H-1-5;
//     if(end<=5)
//        end=5;
//    if(start<=end)
//        return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
//        if(Right_Line[i]==Right_Line[i+5]&&Right_Line[i]==Right_Line[i-5]&&
//        Right_Line[i]==Right_Line[i+4]&&Right_Line[i]==Right_Line[i-4]&&
//        Right_Line[i]==Right_Line[i+3]&&Right_Line[i]==Right_Line[i-3]&&
//        Right_Line[i]==Right_Line[i+2]&&Right_Line[i]==Right_Line[i-2]&&
//        Right_Line[i]==Right_Line[i+1]&&Right_Line[i]==Right_Line[i-1])
//        {//一堆数据一样，显然不能作为单调转折点
//            continue;
//        }
        if(
                ImageSide[i][0] >ImageSide[i+4][0]&&ImageSide[i][0] >ImageSide[i-4][0]&&
                ImageSide[i][0]>=ImageSide[i+3][0]&&ImageSide[i][0]>=ImageSide[i-3][0]&&
                ImageSide[i][0]>=ImageSide[i+2][0]&&ImageSide[i][0]>=ImageSide[i-2][0]&&
                ImageSide[i][0]>=ImageSide[i+1][0]&&ImageSide[i][0]>=ImageSide[i-1][0])
        {
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}
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
uint8_t RoundaboutGetArc(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t num, uint8_t *index)
{
    int i = 0;
    uint8_t inc = 0, dec = 0, n = 0;
    switch (status)
    {
    case 1:
        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
        {
            if (imageSide[i][0] != 0 && imageSide[i + 1][0] != 0)
            {
                if (imageSide[i][0] == imageSide[i + 1][0])
                {
                    n++;
                    continue;
                }
                if (imageSide[i][0] > imageSide[i + 1][0])
                {
                    inc++;
                    inc += n;
                    n = 0;
                }
                else
                {
                    dec++;
                    dec += n;
                    n = 0;
                }
                /* 有弧线 */
                if (inc > num && dec > num)
                {
                    *index = i + num;
                    return 1;
                }
            }
            else
            {
                inc = 0;
                dec = 0;
                n = 0;
            }
        }
        break;

    case 2:
        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
        {
            if (imageSide[i][1] != 159 && imageSide[i + 1][1] != 159)
            {
                if (imageSide[i][1] == imageSide[i + 1][1])
                {
                    n++;
                    continue;
                }
                if (imageSide[i][1] > imageSide[i + 1][1])
                {
                    inc++;
                    inc += n;
                    n = 0;
                }
                else
                {
                    dec++;
                    dec += n;
                    n = 0;
                }
                /* 有弧线 */
                if (inc > num && dec > num)
                {
                    *index = i + num;
                    return 1;
                }
            }
            else
            {
                inc = 0;
                dec = 0;
                n = 0;
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
    uint8_t dec = 0, inc = 0, i;
    // 上线是否右突起
    switch (status)
    {
    case 1:
        for (i = 159 - 1; i > 0; i--)
        {
            if (UpdowmSide[0][i] > 1 && UpdowmSide[0][i + 1] > 1)
            {
                if (UpdowmSide[0][i] >= UpdowmSide[0][i + 1])
                    inc++;
                else
                    dec++;
                /* 有弧线 */
                if (inc > num && dec > num)
                {
                    *index = i + num;
                    return 1;
                }
            }
            else
            {
                inc = 0;
                dec = 0;
            }
        }
        break;
    // 下边线
    case 2:
        for (i = 159 - 1; i > 0; i--)
        {
            if (UpdowmSide[1][i] != 1 && UpdowmSide[1][i + 1] != 1)
            {
                if (UpdowmSide[1][i] >= UpdowmSide[1][i + 1])
                    inc++;
                else
                    dec++;
                /* 有弧线 */
                if (inc > num && dec > num)
                {
                    *index = i + num;
                    return 1;
                }
            }
            else
            {
                inc = 0;
                dec = 0;
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
void X_Find_Point(uint8_t start_point, uint8_t end_point, uint8_t UpdowmSide[2][LCDW], uint8_t pointup[2])
{
    uint8_t i = 0;
    for (i = start_point; i <= end_point; i++)
    {
        if ((abs(UpdowmSide[0][i - 1] - UpdowmSide[0][i - 2] < 3 && abs(UpdowmSide[0][i - 1] - UpdowmSide[0][i + 1]) > 3) && abs(UpdowmSide[0][i + 1] - UpdowmSide[0][i + 2]) < 3) || (abs(UpdowmSide[0][i - 1] - UpdowmSide[0][i - 3] < 5 && abs(UpdowmSide[0][i - 2] - UpdowmSide[0][i + 2]) > 5) && abs(UpdowmSide[0][i + 1] - UpdowmSide[0][i + 3]) < 5))
        {
            pointup[0] = i;
            pointup[1] = UpdowmSide[0][i];
            break;
        }
        else
        {
            pointup[0] = 0;
            pointup[1] = 0;
        }
    }
}

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
uint8_t line_miss(uint8_t imageIn[LCDH][2], uint8_t Flag)
{
    uint8_t i = 0;
    uint8_t misi_number=0;
    uint8_t misi_number1=0;
    if (Flag == 0)
    {
        for (i = 0; i < LCDH; i++)
        {
            if (imageIn[i][0] < 5)
                misi_number++;

        }
    }
    if (Flag == 1)
    {
        for (i = 0; i < LCDH; i++)
        {
            if (imageIn[i][1] > 155)
                misi_number1++;
        }
    }
if(misi_number>=5&&misi_number1>=5)cross.cross_flag=1;;
    return misi_number;
}
uint8_t cross_flag;
void cross_process(uint8_t imageIn[LCDH][2])
{


    uint8_t leftmiss=0;
    uint8_t rightmiss=0;
    leftmiss=line_miss(imageIn,0);
    rightmiss=line_miss(imageIn,1);
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

void Crossroad_Find(uint8_t UpdowmSide[2][LCDW], uint8_t imageSide[LCDH][2], uint8_t Roadwide[LCDH], uint8_t *Flag)
{
    uint8_t i=0,flag=0, flag_number=0;//j=0
//    flag = 0;
//    int i = 15;
    flag_number = 0;
   // X_Find_Point(10, 70, UpdowmSide, leftup);
//    X_Find_Point(90, 158, UpdowmSide, rightup);
    int r_up = right_up_guai(ImageSide,ROAD_END_ROW+15,110, &r_up_guaiflag);
    int l_up = left_up_guai(ImageSide,ROAD_END_ROW+15,110, &l_up_guaiflag);
    int r_d = right_down_guai(ImageSide,110,ROAD_END_ROW+5, &r_down_guaiflag);
    int l_d = left_down_guai(ImageSide,110,ROAD_END_ROW+5, &l_down_guaiflag);
//    ImageAddingLine(imageSide, 1, leftup[0], leftup[1], 5, 110);
//    ImageAddingLine(imageSide, 2, rightup[0], rightup[1], 155, 110);
        for (i = 100; i >= 40; i--)
        {
            if (imageSide[i][0] < 5 && imageSide[i][1] > 155)   //判断两端丢线 到达20则丢
            {
                flag_number++;
            }
//
//                    if (flag_number > 20 && ((r_up_guaiflag == 1 && l_up_guaiflag == 0 && r_down_guaiflag == 0 && l_down_guaiflag == 1) ||
//                            (r_up_guaiflag == 0 && l_up_guaiflag == 1 && r_down_guaiflag == 1 && l_down_guaiflag == 0)))
//                    {
//                        flag = 2;             //只满足丢先
//                        printf("抽象情况且丢线\n");
//            //printf("只满足丢先\n");
//                        break;
//                    }
                    if(flag_number > 20 && r_up_guaiflag == 1 && l_up_guaiflag == 1 && r_down_guaiflag == 1 && l_down_guaiflag == 1 && r_up < r_d && l_up < r_d)

                    {
                        flag = 3;
                        printf("直入\n");
                        ips200_draw_line((uint16)ImageSide[r_up][1],   (uint16)r_up,(uint16)ImageSide[r_d][1], (uint16)r_d, RGB565_BLACK);
                        ips200_draw_line((uint16)ImageSide[l_up][0],   (uint16)l_up,(uint16)ImageSide[l_d][0], (uint16)l_d, RGB565_BLACK);
                        ImageAddingLine(ImageSide,1,(uint16)ImageSide[l_up][0],   (uint16)l_up,(uint16)ImageSide[l_d][0], (uint16)l_d);
                        ImageAddingLine(ImageSide,2,(uint16)ImageSide[r_up][1],   (uint16)r_up,(uint16)ImageSide[r_d][1], (uint16)r_d);

                        break;

                    }
                    if(flag_number > 20 && r_up_guaiflag == 1 && l_up_guaiflag == 1 && r_down_guaiflag == 0 && l_down_guaiflag == 0 && flag !=3)
                    {
                        printf("只有上拐点\n");
                        ips200_draw_line((uint16)ImageSide[r_up][1],   (uint16)r_up,155, 110, RGB565_BLACK);
                        ips200_draw_line((uint16)ImageSide[l_up][0],   (uint16)l_up,5, 110, RGB565_BLACK);
                        ImageAddingLine(ImageSide,1,(uint16)ImageSide[l_up][0],   (uint16)l_up,5, 110);
                        ImageAddingLine(ImageSide,2,(uint16)ImageSide[r_up][1],   (uint16)r_up,155, 110);
                        break;
                    }

        }


////    ips200_draw_line((uint16)ImageSide[left_up_guai(ImageSide,5,114, guai_Flag)][0],  (uint16)left_up_guai(ImageSide,5,114, guai_Flag), 5, 110, RGB565_BLACK);
//   printf("y_right_up_guai = %d\n",(uint16)ImageSide[r][1]);
////   printf("x_right_up_guai = %d\n",r);
//    printf("left_up_guai = %d\n",left_up_guai(ImageSide,5,114, guai_Flag));
//    ips200_draw_line((uint16)ImageSide[i][1],  i, 155, 110, RGB565_BLACK);
//    printf("+++++ 1 = %d\n",(imageSide[i][1] - imageSide[i - 1][1]) );
//            printf("2 = %d\n",(imageSide[i - 1][1] - imageSide[i - 2][1]));
//            printf("3 = %d\n",(imageSide[i - 2][1] - imageSide[i - 3][1]));
//            printf("4 = %d\n", (imageSide[i][1] - imageSide[i + 2][1]));
//            printf("5 = %d\n",(imageSide[i][1] - imageSide[i + 3][1]) );
//            printf("6 = %d\n",(imageSide[i][1] - imageSide[i + 4][1]));
//            printf("r = %d\n",r);
//    for (i = 5; i < 100; i++)
//    {
////        printf("(UpdowmSide[0][%d] - UpdowmSide[0][%d + 4]) = %d \n",i,i,(UpdowmSide[0][i] - UpdowmSide[0][i + 4]));
//        if ((UpdowmSide[0][i] - UpdowmSide[0][i + 4]) > 10 && UpdowmSide[0][i] < 80)
//        {
//            for (j = i; j < 155; j++)
//            {
////                printf("flag = %d\n",flag);
//                if ((UpdowmSide[0][j] - UpdowmSide[0][j + 4]) < -10)
//                {
//                    flag = 1;
////                    printf("flag = %d\n",flag);
//                    break;
//                }
//            }
//        }
//        if (flag == 1){
//            printf("flag = %d\n",flag);
//            break;
//        }
//    }
//    for (i = 100; i >= 40; i--)
//    {
//        if (imageSide[i][0] < 5 && imageSide[i][1] > 155)   //判断两端丢线 到达20则丢
//        {
//            flag_number++;
//        }
//        if (flag_number > 20 && flag == 0)
//        {
//            flag = 2;             //只满足丢先
////printf("只满足丢先\n");
//            break;
//        }
//        else if (flag_number > 20 && flag == 1)
//        {
//            flag = 3;                 //满足丢线并且上边线符合十字
//printf("满足丢线并且上边线符合十字\n");
//            break;
//        }
//    }
//    if (flag == 1&& motor_flag == 1)//  没有丢线但是满足上边线十字
//    {
//        *Flag = 1;
//        printf("没有丢线但是满足上边线十字\n");
//    }
//    if (flag == 2&& motor_flag == 1 )//只满足丢线
//    {
//        *Flag = 2;
//        if (leftup[0] != 0 && rightup[0] != 0)
//        {
//            ImageAddingLine(imageSide, 1, leftup[0], leftup[1], 5, 110);
//            ImageAddingLine(imageSide, 2, rightup[0], rightup[1], 155, 110);
//            ips200_draw_line(leftup[0], leftup[1], 5, 110, RGB565_BLACK);
//            ips200_draw_line(rightup[0], rightup[1], 155, 110, RGB565_BLACK);
//        }
//        else
//        {
//            ImageAddingLine(imageSide, 1, 80, 10, 5, 110);
//            ImageAddingLine(imageSide, 2, 80, 10, 155, 110);
//            ips200_draw_line(80, 10, 5, 110, RGB565_BLACK);
//            ips200_draw_line(80, 10, 155, 110, RGB565_BLACK);
//        }
//    }
//    if (flag == 3&& motor_flag == 1 )//
//    {
//        *Flag = 3;
//        if (leftup[0] != 0 && rightup[0] != 0)
//        {
//            ImageAddingLine(imageSide, 1, leftup[0], leftup[1], 5, 110);
//            ImageAddingLine(imageSide, 2, rightup[0], rightup[1], 155, 110);
//            ips200_draw_line( leftup[0], leftup[1], 5, 110, RGB565_BLACK);
//            ips200_draw_line(rightup[0], rightup[1], 155, 110, RGB565_BLACK);
//        }
//        else
//        {
//            ImageAddingLine(imageSide, 1, 80, 10, 5, 110);
//            ImageAddingLine(imageSide, 2, 80, 10, 155, 110);
//            ips200_draw_line(  80, 10, 5, 110, RGB565_BLACK);
//            ips200_draw_line( 80, 10, 155, 110, RGB565_BLACK);
//        }
//    }
//    if (flag == 0)
//    {
//        *Flag = 0;
//    }
//    //ips200_show_int(0,250,Crossroad_Flag,3);
}
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
        if ((imageSide[i][1] <= imageSide[i + 1][1]) && (imageSide[i][1] < 155))
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
uint8_t RoadImageSide_Mono(uint8_t imageSide[LCDH][2], uint8_t Flag)
{
    uint8_t i = 0;
    uint8_t State;
    State = 0;
    /* 左边线是否单调 */
    if (Flag == 0)
    {
        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
        {
            if ((my_abs(imageSide[i][0] - imageSide[i + 1][0]) <= 1) && (imageSide[i][0] > 10))
            {
                State++;
            }
        }
    }
    /* 右边线是否单调 */
    if (Flag == 1)
    {
        for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
        {
            if ((my_abs(imageSide[i][1] - imageSide[i + 1][1]) <= 1) && (imageSide[i][1] < 155))
            {
                State++;
            }
        }
    }
    if (State > (ROAD_START_ROW - ROAD_END_ROW) * 0.7)
        return 1;
    else
        return 0;
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

/*!
 * @brief    圆形处理
 *
 */

//void round_detect()
//{
//    uint8 lose_line_l=0;
//    uint8 lose_line_r=0;
//    uint8 lose_left=0;
//    uint8 lose_right=0;
//    for (uint8 i = 100; i >= 40; i--)
//        {
//            if (ImageSide[i][0] < 5 )
//            {
//                lose_line_l++;
//                if(lose_line_l>=30)
//                {
//                    lose_line_l=0;
//                    lose_left=1;
//                    break;
//                }
//
//            }
//
//        }
//
//    for (uint8 i = 100; i >= 40; i--)
//            {
//                if (ImageSide[i][1] >155 )
//                {
//                    lose_line_r++;
//                    if(lose_line_r>=30)
//                    {
//                        lose_line_r=0;
//                        lose_right=1;
//                        break;
//                    }
//
//                }
//
//            }
//    if(lose_left==1&&lose_right==0)L_CircleFlag=1;
//    if(lose_left==0&&lose_right==1)R_CircleFlag=1;
//
//}

void r_Round_process(uint8_t imageSide[LCDH][2], uint8_t UpImageSide[2][LCDW], uint8_t *R_Flag)
{
    uint8_t errL = 0,
            errR = 0,
            i = 0,
            inc = 0,
            dec = 0;
    Lpointx = 0, Lpointy = 0;
    Rpointx = 0, Rpointy = 0;
    static uint8_t l_finderr = 0;
    uint8_t flag = 0, Down_flag = 0;
    /////
    leftState = RoadImageSide_Mono(imageSide, 0);
    errR = RoundaboutGetArc(imageSide, 2, 5, &Rpointy);

    if(leftState==1&&errR==1)R_CircleFlag=1;

    /////
    switch (*R_Flag)
    { // 奇数右圆环
    case 1:
        //leftState = RoadImageSide_Mono(imageSide, 0);

        errR = RoundaboutGetArc(imageSide, 2, 5, &Rpointy);
        if (errR)
        {

            Rpointx = imageSide[Rpointy][1];
            ImageAddingLine(imageSide, 2, Rpointx, Rpointy, 159, 119);//可能可以补更靠近车体
            l_finderr = 1;
        }
        else
        {
            if (l_finderr)
                *R_Flag = 3; // 准备进入环岛
        }
        break;

    case 3:

        for (i = 1; i < 119; i++)
        {
            ImageSide[i][0] = ImageSide[i][0] + 50;
        }
        if (RoadUpSide_Mono(30, 159, UpImageSide) == 1) // 上线单调增进入下一步
            *R_Flag = 5;
        break;

    case 5:
        errL = RoundaboutGetArc(imageSide, 1, 10, &Rpointy); // 右圆环再要出环岛处，是左侧存在弧线，将第二个形参改为1

        // 检查下线
        for (i = 159; i > 0; i--)
        {
            if (UpdowmSide[1][i] == 119)
                inc++;
            else
                dec++;
            if (dec <= 15)
            {
                Down_flag = 1;
                break;
            }
        }
        flag = RoadUpSide_Mono(20, 120, UpImageSide);

        if (flag && errL && Down_flag)
        {
            *R_Flag = 7;
        }
        break;

    case 7:

        ImageAddingLine(imageSide, 1, 80, 10, 0, 119);
        flag = RoadUpSide_Mono(50, 120, UpImageSide);

        if (flag == 0)
        {

            *R_Flag = 0;
            l_finderr = 0;
            errR = 0;
        }
        break;
    }

}
#define sideoffset 0//为一时使用边线偏移的办入环
#define try 0

uint8 offsetbasis=20;
uint8_t ImageGetHop(uint8_t imageSide[LCDH][2], uint8_t state, uint8_t *x, uint8_t *y)
{
    int i = 0;
    uint8_t px = 0, py = 0;
    uint8_t count = 0;
    switch(state)
    {
      case 1:
        /* 寻找跳变点 */
        for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
        {
            if(imageSide[i][0] == 0 && i > (ROAD_END_ROW + 5))
            {
                count++;

                if(count > 5)
                {
                    if(imageSide[i-1][0] > (imageSide[i][0] + 20))
                    {
                        py = i - 1;
                        px = imageSide[py][0];
                        break;
                    }
                    if(imageSide[i-2][0] > (imageSide[i-1][0] + 20))
                    {
                        py = i - 2;
                        px = imageSide[py][0];
                        break;
                    }
                    if(imageSide[i-3][0] > (imageSide[i-2][0] + 20))
                    {
                        py = i - 3;
                        px = imageSide[py][0];
                        break;
                    }
                    if(imageSide[i-4][0] > (imageSide[i-3][0] + 20))
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
        for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
        {
            if(imageSide[i][1] == 159 && i > (ROAD_END_ROW + 5))
            {
                count++;

                if(count > 5)
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

void l_Round_process(uint8_t imageSide[LCDH][2], uint8_t UpImageSide[2][LCDW], uint8_t *L_Flag)
{
    uint8_t errL = 0,
            errR = 0,
            i = 0,
            inc = 0,
            dec = 0;
   // uint8 a=0,b=0;
    Lpointx = 0, Lpointy = 0;
    static uint8_t r_finderr = 0;
    uint8_t flag = 0, Down_flag = 0;
if(L_CircleFlag==0 )
{
     rightState = RoadImageSide_Mono(imageSide, 1);
    errL = RoundaboutGetArc(imageSide, 1, 3, &Lpointy);
     if(rightState==1&&errL==1)L_CircleFlag=2;
}



    switch (*L_Flag)
    {

    case 2:
        //rightState = RoadImageSide_Mono(imageSide, 1);
        errL = RoundaboutGetArc(imageSide, 1, 4, &Lpointy);
        if (errL)
        {
            Lpointx = imageSide[Lpointy][0];
            //found_round_flag=1;
            ImageAddingLine(imageSide, 1, Lpointx, Lpointy, 1, 119);
            r_finderr = 1;
        }
        else
        {
            if (r_finderr)
                *L_Flag = 4; // 准备进入环岛
        }
        break;
    case 4:
#if sideoffset

        for (uint8 i = 1; i < 119; i++)
        {

             a++;
            if(a>=20)a=20;
            ImageSide[i][1] = ImageSide[i][1] - 60-offsetbasis+a;
        }
         // 上线单调减进入下一步
#endif
//        X_Find_Point(40,150 , UpdowmSide, leftup);
//
//        if(leftup[0]!=0)
//        {
//            ImageAddingLine(imageSide, 2, leftup[0], leftup[1], 155, 90);
//
//        }
#if 0
            for (uint8 i= 10; i <= 130; i++)
               {

                if(UpdowmSide[0][i]-UpdowmSide[0][i-1]>=0
                        &&UpdowmSide[0][i]-UpdowmSide[0][i-2]>=0
                        &&UpdowmSide[0][i]-UpdowmSide[0][i-3]>0
                        &&UpdowmSide[0][i]-UpdowmSide[0][i+1]>=0
                        &&UpdowmSide[0][i]-UpdowmSide[0][i+2]>=0
                        &&UpdowmSide[0][i]-UpdowmSide[0][i+3]>0)
                   {
                       leftup[0] = i;
                       leftup[1] = UpdowmSide[0][i];
                       printf("%d %d\n",leftup[0],leftup[1]);
                       ImageAddingLine(imageSide, 2, leftup[0], leftup[1], 155, 110);
                   }

               }
#endif
#if 1
//        for(uint8 j=120;j>=30;j--)
//                    {
//                        if(
//                                (UpdowmSide[0][j+3]<UpdowmSide[0][j])
//                                &&(UpdowmSide[0][j+5]<UpdowmSide[0][j])
//                                &&(UpdowmSide[0][j+7]<UpdowmSide[0][j])
//                                &&(UpdowmSide[0][j]>UpdowmSide[0][j-3])
//                                &&(UpdowmSide[0][j]>UpdowmSide[0][j-5])
//                                &&(UpdowmSide[0][j]>UpdowmSide[0][j-7])
//
//                                )
//                    {
//                    leftup[0]=imageSide[j][0];
//                    leftup[1]=j;
//                    printf("%d %d\n",leftup[0],leftup[1]);
//                      break;
//                    }
//                    }
//            errL = RoundaboutGetArc(imageSide, 1, 4, &Lpointy);
//            if(errL)
//            {
//
//                for(uint8 j=Lpointy;j>=2;j--)
//            {
//                    if((Bin_Image[j][Lpointx]!=0)
//                    {
//                        leftup[0]=Bin_Image[j][Lpointx];
//                        leftup[1]=j;
//                        break;
//                    }
//            }
//            }
//            else
            ImageGetHop(imageSide,1,&leftup[0],&leftup[1]);

        ImageAddingLine(imageSide, 2, leftup[0], leftup[1], 155, 100);
#endif
        if (RoadUpSide_Mono(30, 159, UpImageSide) == 2)
        {
             *L_Flag = 6;

        }

        break;

    case 6:
        errR = RoundaboutGetArc(imageSide, 2, 10, &Lpointy);
        // 检查下线
        for (i = 159; i > 0; i--)
        {
            if (UpdowmSide[1][i] == 119)
                inc++;
            else
                dec++;
            if (dec <= 15)
            {
                Down_flag = 1;
                break;
            }
        }
        flag = RoadUpSide_Mono(20, 120, UpImageSide);
        if (flag && errR && Down_flag)
        {
            *L_Flag = 8;
        }
        break;
    case 8:
        ImageAddingLine(imageSide, 2, 80, 10, 0, 119);
        flag = RoadUpSide_Mono(50, 120, UpImageSide);
        if (flag == 0)
        {
            *L_Flag = 0;
            r_finderr = 0;
            errR = 0;
        }
        break;
    }
}

void cameracar(void) // 在中断里处理
{
    /*十字处理函数*/

//    Crossroad_Find(UpdowmSide, ImageSide, Roadwide, &Crossroad_Flag);







//   if( RoadIsCross(ImageSide, can))
//   {
//
//       ips200_show_char(0, 200, 'x');
//   }

    /*环岛处理函数*/
   // r_Round_process(ImageSide, UpdowmSide, &R_CircleFlag);
//    l_Round_process(ImageSide, UpdowmSide, &L_CircleFlag);
   // printf("round_flag%d\n",L_CircleFlag);
    Get_Errand();
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
int16_t RoadGetSteeringError(uint8_t imageSide[LCDH][2], uint8_t lineIndex)
{
    int16_t sum = 0;
    sum = (imageSide[lineIndex][0] + imageSide[lineIndex][1] - 159) +
          (imageSide[lineIndex - 6][0] + imageSide[lineIndex - 6][1] - 159) +
          (imageSide[lineIndex - 12][0] + imageSide[lineIndex - 12][1] - 159) +
          (imageSide[lineIndex - 18][0] + imageSide[lineIndex - 18][1] - 159) +
          (imageSide[lineIndex - 24][0] + imageSide[lineIndex - 24][1] - 159);
    sum = sum / 5;
    return sum;
}
int16_t midlineerror(uint8_t imageSide[LCDH][2], uint8_t lineIndex)
{
    int16_t sum = 0;
       int e1,e2,e3,e4,e5=0;
    if(found_round_flag)
    {
            e1=(imageSide[lineIndex][1]-35);
            e2=(imageSide[lineIndex - 6][1] -35);
            e3=(imageSide[lineIndex - 12][1] -35);
            e4=(imageSide[lineIndex - 18][1] -35);
            e5=(imageSide[lineIndex - 24][1] -35);
            sum=(80-e1)+(80-e2)+(80-e3)+(80-e4)+(80-e5);
            sum=sum/5;
    }
    else
    {
    e1=(imageSide[lineIndex][0]+ imageSide[lineIndex][1])/2;
    e2=(imageSide[lineIndex - 6][0] + imageSide[lineIndex - 6][1])/2;
    e3=(imageSide[lineIndex - 12][0] + imageSide[lineIndex - 12][1])/2;
    e4=(imageSide[lineIndex - 18][0] + imageSide[lineIndex - 18][1])/2;
    e5=(imageSide[lineIndex - 24][0] + imageSide[lineIndex - 24][1])/2;
    sum=(80-e1)+(80-e2)+(80-e3)+(80-e4)+(80-e5);
    sum=sum/5;
    }

    return sum;

}
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
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端00 ——09 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端10 ——19 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,           //图像最远端20 ——29 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端30 ——39 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,     //图像最远端40 ——49 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端50 ——59 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端60 ——69 行权重
};
//误差选取的范围距离车越近，车模反应越慢，从而越切外；
//选取的范围越远，反应越提前，车模越切内
float Err_Sum(void)
{
    int i;
    float err=0;
    float weight_count=0;
    //常规误差
    for(i=LCDH-1;i>=LCDH-120-1;i--)//常规误差计算
    {
        err+=(MT9V03X_W/2-((ImageSide[i][0]+ImageSide[i][1])>>1))*Weight_list[i];//右移1位，等效除2
        weight_count+=Weight_list[i];
    }
    err=err/weight_count;
    return err;
}
void Get_Errand(void)
{

    g_sSteeringError =  Err_Sum();//midlineerror(ImageSide, ROAD_MAIN_ROW);
    //g_sSteeringError= Err_Sum();//加权偏差
//    int a=0;
//    a=(int)g_sSteeringError;
//    ips200_show_int(0, 150, a, 3);
    // 偏差放大
    if ((g_sSteeringError < 60) && (g_sSteeringError > -60))
    {
        if ((g_sSteeringError < 20) && (g_sSteeringError > -20))
        {
            Servo_P = 11;
        }
        else
        {
            Servo_P = 14;//
        }
    }
    else
    {
        Servo_P = 11;
    }
    error_servo = g_sSteeringError * Servo_P / 10;
    // 偏差限幅
    if (error_servo > 100)
        error_servo = 100;
    if (error_servo < -100)
        error_servo = -100;
}
/*!
  * @brief    判断是否是斑马线
  *
  * @param    image ： 二值图像信息
  *
  * @return   0：不是， 1：是
  *
  * @note     思路：
  *
  * @see
  *
  * @date     2020/6/23 星期二
  */
char RoadIsZebra(uint8_t image[LCDH][LCDW], uint8_t *flag)
{
    int i = 0, j = 0;
    int count = 0;
    for(i = ROAD_MAIN_ROW - 30; i < ROAD_MAIN_ROW + 30; i++)
    {
        for(j = 1; j < LCDW; j++)
        {
            if(image[i][j] == 1 && image[i][j-1] == 0)
            {
                count ++;
            }
        }
        if(count > 5)
        {
            *flag = 1;
            return 1;
        }
    }
    return 0;
}




uint8_t RoadIsCross(uint8_t imageSide[LCDH][2], uint8_t *flag) //是否是十字
{
    int i = 0;
    uint8_t errR = 0, errF = 0;
    uint8_t  rightState = 0, leftState = 0;
    int start[5] = {0, 0, 0, 0, 0}, end[5] = {0, 0, 0, 0, 0};
    uint8_t count = 0;
    uint8_t index = 0;

    /* 检测右侧边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        if(imageSide[i][1] == 159)
            count++;
        else
        {
            if(count > 10 && index < 5)
            {
                start[index] = i + count;
                end[index]   = i;
                index++;
            }
            count = 0;
        }
    }
    if(index > 1)
    {
        if(end[0] - start[1] > 10)
            rightState = 1;
    }
    index = 0;

    /* 检测左侧边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 */
    for(i = ROAD_START_ROW-1; i > ROAD_END_ROW; i--)
    {
        if(imageSide[i][0] == 0)
            count++;
        else
        {
            if(count > 10 && index < 5)
            {
                start[index] = i + count;
                end[index]   = i;
                index++;
            }
            count = 0;
        }
    }
    if(index > 1)
    {
        if(end[0] - start[1] > 10)
            leftState = 1;
    }

    if(errR && errF)
    {
        count = 0;
        index = 0;
        //上线是否右突起
        for(i = 159-1; i > 0; i--)
        {
          if(UpdowmSide[0][i] != 1 && UpdowmSide[0][i+1] != 1)
          {
              if(UpdowmSide[0][i] >= UpdowmSide[0][i+1])
                  index++;
              else
                  count++;
              /* 有弧线 */
              if(index > 20 && count > 20)
              {
                  *flag = 1;
                  return 1;
              }
          }
          else
          {
              index = 0;
              count = 0;
          }
        }
    }

    return 0;

}

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

void edge_black(void) //将bin_image的边框初始化为黑色,此前需要将获取二值化图像中的循环进行更改，使之不将边框进行刷新
{
    for (int i = 0; i < LCDH ; i++)
    {

        Bin_Image[i][0] = 0;

        Bin_Image[i][LCDW - 1] = 0;

    }
    for (int j = 0; j < LCDW ; j++)
    {
        Bin_Image[0][j] = 0;

        Bin_Image[LCDH - 1][j] = 1;
    }
}
