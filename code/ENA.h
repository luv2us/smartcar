/*
 * ENA.h
 *
 *  Created on: 2024年5月2日
 *      Author: 凌纪哲
 */

#ifndef APP_ENA_H_
#define APP_ENA_H_
#include "zf_common_headfile.h"
#define USE_num LCDH*3
extern uint8 start_point_l[2];//左边起点的x，y值
extern uint8 start_point_r[2];//右边起点的x，y值
extern uint16 points_l[(uint16)USE_num][2];//左线
extern uint16 points_r[(uint16)USE_num][2];//右线
extern uint16 dir_r[(uint16)USE_num];//用来存储右边生长方向
extern uint16 dir_l[(uint16)USE_num];//用来存储左边生长方向
extern uint16 data_stastics_l;//统计左边找到点的个数
extern uint16 data_stastics_r;//统计右边找到点的个数
extern uint8 hightest;//最高点
uint8 get_start_point(uint8 start_row);
void search_l_r(uint16 break_flag, uint8(*image)[LCDW], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest);
void enh_process(void);
void image_draw_rectan(uint8(*image)[LCDW]);

#endif /* APP_ENA_H_ */
