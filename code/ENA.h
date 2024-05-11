/*
 * ENA.h
 *
 *  Created on: 2024��5��2��
 *      Author: �����
 */

#ifndef APP_ENA_H_
#define APP_ENA_H_
#include "zf_common_headfile.h"
#define USE_num LCDH*3
extern uint8 start_point_l[2];//�������x��yֵ
extern uint8 start_point_r[2];//�ұ�����x��yֵ
extern uint16 points_l[(uint16)USE_num][2];//����
extern uint16 points_r[(uint16)USE_num][2];//����
extern uint16 dir_r[(uint16)USE_num];//�����洢�ұ���������
extern uint16 dir_l[(uint16)USE_num];//�����洢�����������
extern uint16 data_stastics_l;//ͳ������ҵ���ĸ���
extern uint16 data_stastics_r;//ͳ���ұ��ҵ���ĸ���
extern uint8 hightest;//��ߵ�
uint8 get_start_point(uint8 start_row);
void search_l_r(uint16 break_flag, uint8(*image)[LCDW], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest);
void enh_process(void);
void image_draw_rectan(uint8(*image)[LCDW]);

#endif /* APP_ENA_H_ */
