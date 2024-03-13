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
 * 2022-09-15       pudding            first version
 * 2023-04-25       pudding            增加中文注释说明
 ********************************************************************************************************************/

#ifndef _zf_device_camera_h_
#define _zf_device_camera_h_

#include "zf_common_fifo.h"
#include "zf_common_typedef.h"
#include "zf_driver_uart.h"
#include "zf_device_type.h"

#include "zf_device_mt9v03x.h"

//=================================================摄像头公共库 基本配置================================================
#define CAMERA_RECEIVER_BUFFER_SIZE (8)         // 定义摄像头接收数据缓冲区大小
extern fifo_obj_struct camera_receiver_fifo;    // 声明摄像头接收数据fifo结构体
extern uint8 camera_send_image_frame_header[4]; // 声明摄像头数据发送到上位机的帧头
#define LCDH 120
#define LCDW 160
extern uint8_t UpdowmSide[2][LCDW];
extern uint8_t ImageSide[LCDH][2];
extern uint8_t ImageSide_last[LCDH][2];
extern uint8_t Roadwide[LCDH];
//=================================================摄像头公共库 基本配置================================================

//=================================================摄像头公共库 基础函数================================================
void camera_binary_image_decompression(const uint8 *data1, uint8 *data2, uint32 image_size); // 摄像头二进制图像数据解压为十六进制八位数据 小钻风用
void camera_send_image(uart_index_enum uartn, const uint8 *image_addr, uint32 image_size);   // 摄像头图像发送至上位机查看图像
void camera_fifo_init(void);                                                                 // 摄像头串口 FIFO 初始化

uint8 camera_init(uint8 *source_addr, uint8 *destination_addr, uint16 image_size); // 摄像头初始化
float my_abs(float x);                                                             // 求绝对值函数

unsigned char my_adapt_threshold(uint8 *image, uint16 col, uint16 row);
short GetOSTU(unsigned char tmImage[LCDH][LCDW]); // 大津法
void lq_sobelAutoThreshold(unsigned char imageIn[LCDH][LCDW], unsigned char imageOut[LCDH][LCDW]);
void computeHistogram(unsigned char image[LCDH][LCDW], int histogram[]);
int otsuThreshold(unsigned char image[LCDH][LCDW]);

/*
double computeEntropy(int histogram[], int totalPixels);
int kapurThreshold(uint8 tmImage[LCDH][LCDW]);
*/

void Get_Use_Image(void);       // 压缩图像
void Get_Bin_Image(uint8 mode); // 二值化
void Bin_Image_Filter(void);    // 图像滤波

uint8_t ImageGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t imageOut_last[LCDH][2]); // 获取左右边线
uint8_t ImageGetSide1(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2]);
void Find_Boundry(uint8_t imageOut[LCDH][2]);
void RoadNoSideProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t mode, uint8_t lineIndex); // 丢线处理
uint8_t RoadIsNoSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t lineIndex);                 // 判断左右边线是否丢线
uint8_t UpdownSideGet(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[2][LCDW]);                                   // 获取上下边线
uint8_t GetRoadWide(uint8_t imageInput[LCDH][2], uint8_t imageOut[LCDH]);                                           // 获取道路宽度

void ips200_bin_roadside(uint8_t imageOut[LCDH][2]);   // 显示左右边线
void ips200_bin_updownside(uint8_t imageOut[2][LCDW]); // 显示左右边线
void Show_Camera_Info(void);

void FindCarbarn(uint8_t imageInput[LCDH][LCDW], uint8_t *Flag);                                              //                                                  //寻找斑马线
void Carbarn(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t *motor_flag, uint8_t *Flag); // 调用上下两条函数
void ZebraProcess(uint8_t imageSide[LCDH][2], uint8_t state, uint8_t *motor);                                 // 斑马线处理函数

uint8_t RoundaboutGetArc(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t num, uint8_t *index); // 寻找左右圆弧，并返回圆弧的点
uint8_t UpSideErr(uint8_t SideInput[2][LCDW], uint8_t status, uint8_t num, uint8_t *index);        // 寻找上下圆弧，并返回圆弧的点

void X_Find_Point(uint8_t start_point, uint8_t end_point, uint8_t UpdowmSide[2][LCDW], uint8_t pointup[2]);                   // 用于寻找断点
void Crossroad_Find(uint8_t UpdowmSide[2][LCDW], uint8_t imageSide[LCDH][2], uint8_t Roadwide[LCDH], uint8_t *Flag);          // 十字处理函数
void ImageAddingLine(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY); // 补线

uint8_t RoadIsStraight(uint8_t imageSide[LCDH][2]); // 判断是否处于直道
// void Find_m_Point(uint8_t start_row, uint8_t end_row, uint8_t imageSide[LCDH][2], uint8_t pointup[2]);
uint8_t RoadImageSide_Mono(uint8_t imageSide[LCDH][2], uint8_t Flag);                                           // 判断一边边线是否单调
uint8_t RoadUpSide_Mono(uint8_t X1, uint8_t X2, uint8_t imageIn[2][LCDW]);                                      // 判断上下边线是否单调
void Round_process(uint8_t imageSide[LCDH][2], uint8_t UpImageSide[2][LCDW], uint8_t *L_Flag, uint8_t *R_Flag); // 圆环处理函数

int16_t RoadGetSteeringError(uint8_t imageSide[LCDH][2], uint8_t lineIndex); // 获取转向误差
int16_t midlineerror(uint8_t imageSide[LCDH][2], uint8_t lineIndex);
void Get_Errand(void);                                                       // 获取图像偏差
void cameracar(void);
//=================================================摄像头公共库 基础函数================================================

#endif
