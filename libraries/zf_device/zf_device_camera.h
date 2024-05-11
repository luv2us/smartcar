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
 * �ļ�����          zf_device_camera
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          ADS v1.9.4
 * ����ƽ̨          TC264D
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2022-09-15       pudding            first version
 * 2023-04-25       pudding            ��������ע��˵��
 ********************************************************************************************************************/

#ifndef _zf_device_camera_h_
#define _zf_device_camera_h_

#include "zf_common_fifo.h"
#include "zf_common_typedef.h"
#include "zf_driver_uart.h"
#include "zf_device_type.h"

#include "zf_device_mt9v03x.h"


//=================================================����ͷ������ ��������================================================
#define CAMERA_RECEIVER_BUFFER_SIZE (8)         // ��������ͷ�������ݻ�������С
extern fifo_obj_struct camera_receiver_fifo;    // ��������ͷ��������fifo�ṹ��
extern uint8 camera_send_image_frame_header[4]; // ��������ͷ���ݷ��͵���λ����֡ͷ
#define LCDH 120
#define LCDW 160
extern uint8_t UpdowmSide[2][LCDW];
extern uint8_t ImageSide[LCDH][2];
extern uint8_t ImageSide_last[LCDH][2];
extern uint8_t Roadwide[LCDH];
extern uint8 l_border[LCDH];//��������
extern uint8 r_border[LCDH];//��������
extern uint8 center_line[LCDH];
extern uint8 hightest;
extern uint8_t neighbor8_completed;
typedef struct
{
        uint8 x;
        uint8 y;
}Pointtypedef;

typedef enum
{
        normal=0,
        right,
        left
}seektypedef;
extern seektypedef seektype;
//#define use_num LCDH*3 //�����Ѱ����
//extern Pointtypedef neighbor8_start_l;
//extern Pointtypedef neighbor8_start_r;
//extern Pointtypedef points_l[(uint16)use_num];
//extern uint16 data_stastics_l;
//=================================================����ͷ������ ��������================================================

//=================================================����ͷ������ ��������================================================
void camera_binary_image_decompression(const uint8 *data1, uint8 *data2, uint32 image_size); // ����ͷ������ͼ�����ݽ�ѹΪʮ�����ư�λ���� С�����
void camera_send_image(uart_index_enum uartn, const uint8 *image_addr, uint32 image_size);   // ����ͷͼ��������λ���鿴ͼ��
void camera_fifo_init(void);                                                                 // ����ͷ���� FIFO ��ʼ��
uint8 camera_init(uint8 *source_addr, uint8 *destination_addr, uint16 image_size); // ����ͷ��ʼ��
void lq_sobelAutoThreshold(unsigned char imageIn[LCDH][LCDW], unsigned char imageOut[LCDH][LCDW]);

float my_abs(float x);                                                             // �����ֵ����
void computeHistogram(unsigned char image[LCDH][LCDW], int histogram[]);
//int otsuThreshold(unsigned char image[LCDH][LCDW]);
int otsuThreshold1(unsigned char image[LCDH][LCDW]);

void Get_Use_Image(void);       // ѹ��ͼ��
void Get_Bin_Image(uint8 mode); // ��ֵ��
void Bin_Image_Filter(void);    // ͼ���˲�

uint8_t ImageGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t imageOut_last[LCDH][2],uint8 mode); // ��ȡ���ұ���
//void Find_Boundry(uint8_t imageOut[LCDH][2]);
void RoadNoSideProcess(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t mode, uint8_t lineIndex); // ���ߴ���
uint8_t RoadIsNoSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[LCDH][2], uint8_t lineIndex);                 // �ж����ұ����Ƿ���
uint8_t UpdownSideGet(uint8_t imageInput[LCDH][LCDW], uint8_t imageOut[2][LCDW]);                                   // ��ȡ���±���
uint8_t GetRoadWide(uint8_t imageInput[LCDH][2], uint8_t imageOut[LCDH]);                                           // ��ȡ��·���
//��ʾ����
void ips200_bin_roadside(uint8_t imageOut[LCDH][2]);
void ips200_bin_updownside(uint8_t imageOut[2][LCDW]);
void Show_Camera_Info(void);
//
//void FindCarbarn(uint8_t imageInput[LCDH][LCDW], uint8_t *Flag);                                              //                                                  //Ѱ�Ұ�����
//void Carbarn(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t *motor_flag, uint8_t *Flag); // ����������������
//void ZebraProcess(uint8_t imageSide[LCDH][2], uint8_t state, uint8_t *motor);                                 // �����ߴ�����
//void X_Find_Point(uint8_t start_point, uint8_t end_point, uint8_t UpdowmSide[2][LCDW], uint8_t pointup[2]);                   // ����Ѱ�Ҷϵ�

void zebra_panduan(unsigned char bin_image[LCDH][LCDW]);
///////////////////////////////////////////////////
//ʮ����غ���
uint8 Crossroad_Find(uint8_t UpdowmSide[2][LCDW], uint8_t imageSide[LCDH][2], uint8_t Roadwide[LCDH], uint8_t *Flag);          // ʮ�ִ�����
int  left_down_guai(uint8_t imageSide[LCDH][2],int start,int end, uint8_t *flag);
int right_down_guai(uint8_t imageSide[LCDH][2],int start,int end, uint8_t *flag);
int left_up_guai(uint8_t imageSide[LCDH][2],int start,int end, uint8_t *flag);
int right_up_guai(uint8_t imageSide[LCDH][2],int start,int end, uint8_t *flag);
////////////////////////////////////////////////////


void ImageAddingLine(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY); // ����
void ImageAddingray(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY);//��һ������
uint8_t line_miss(uint8_t imageIn[LCDH][2], uint8_t Flag,uint8 starty);//�ж϶�����
uint8_t RoadIsStraight(uint8_t imageSide[LCDH][2]); // �ж��Ƿ���ֱ��
uint8_t RoadImageSide_Mono(uint8_t imageSide[LCDH][2], uint8_t Flag);                                           // �ж�һ�߱����Ƿ񵥵�
uint8_t RoadUpSide_Mono(uint8_t X1, uint8_t X2, uint8_t imageIn[2][LCDW]);                                      // �ж����±����Ƿ񵥵�
void cross_process(uint8_t imageIn[LCDH][2]);
//uint8 Monotonicity_Change_left(uint8 start,uint8 end);//�����Ըı䣬����ֵ�ǵ����Ըı�����ڵ�����
uint8_t ImageGetHop(uint8_t imageSide[LCDH][2], uint8_t state, uint8_t *x, uint8_t *y);


/////////////////////////////////////////////////
//������غ���
uint8_t RoadIsRoundabout(uint8_t Upimage[2][LCDW],  uint8_t image[LCDH][2], uint8_t *flag);
void roundabout(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t UpdowmSide[2][LCDW], uint8_t* state);
int Find_Down_Point(uint8 state);//����Բ�������¹յ�ͳ����Ĺյ�
void Roundabout_Get_UpDowmSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[2][LCDW], uint8_t status);
void RoundaboutGetSide(uint8_t imageInput[LCDH][LCDW], uint8_t imageSide[LCDH][2], uint8_t status);
uint8_t RoundaboutGetArc(uint8_t imageSide[LCDH][2], uint8_t status, uint8_t num, uint8_t *index,uint8 start); // Ѱ������Բ����������Բ���ĵ�
uint8_t UpSideErr(uint8_t SideInput[2][LCDW], uint8_t status, uint8_t num, uint8_t *index);        // Ѱ������Բ����������Բ���ĵ�
/////////////////////////////////////////////////

//int16_t RoadGetSteeringError(uint8_t imageSide[LCDH][2], uint8_t lineIndex); // ��ȡת�����
//int16_t midlineerror(uint8_t imageSide[LCDH][2], uint8_t lineIndex);

//////////////////////////////////////////////////
//ƫ��ѭ����غ���
void Get_Errand(void);// ��ȡͼ��ƫ��
void get_midline(void);
float Err_Sum(void);
//////////////////////////////////////////////////
void cameracar(void);
//////////////////////////////////////////////////
//��������غ���
void image_filter(void);
void medianFilterEdge(uint8_t imagein[LCDH][2], uint8_t imageOut[LCDH][2]);//�Ա����˲�
//void bilateralFilterEdge(uint8_t edge[LCDH][2], uint8_t filteredEdge[LCDH][2]);
//double gaussian(double x, double sigma) ;
void imageside(uint8 mode);
void triangularFilter(uint8_t edge[LCDH][2], uint8_t filteredEdge[LCDH][2], int windowSize) ;
//////////////////////////////////////////////////


//=================================================����ͷ������ ��������================================================

#endif
