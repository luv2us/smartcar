/*
 * ENA.c
 *
 *  Created on: 2024��5��9��
 *      Author: ASUS
 */


#include "ENA.h"
#define border_max  LCDW-2
#define border_min  1
extern unsigned char Bin_Image[LCDH][LCDW];
uint8 start_point_l[2] = { 0 };//�������x��yֵ
uint8 start_point_r[2] = { 0 };//�ұ�����x��yֵ
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0,l_found = 0,r_found = 0;
    //����
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

        //���м�����ߣ��������
    for (i = LCDW / 2; i > border_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if ( Bin_Image[start_row][i] == 255 && Bin_Image[start_row][i - 1] == 0)
        {
//            printf("�ҵ�������image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }

    for (i = LCDW / 2; i < border_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if ( Bin_Image[start_row][i] == 255 && Bin_Image[start_row][i + 1] == 0)
        {
//            printf("�ҵ��ұ����image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if(l_found&&r_found)return 1;
    else {
//        printf("δ�ҵ����\n");
        return 0;
    }
}

  //�����ҵ�������Ա��������˵300�����ܷ��£�������Щ�������ȷʵ�Ѷ����ඨ����һ��
 //��ŵ��x��y����
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//����
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//����
uint16 dir_r[(uint16)USE_num] = { 0 };//�����洢�ұ���������
uint16 dir_l[(uint16)USE_num] = { 0 };//�����洢�����������
uint16 data_stastics_l = 0;//ͳ������ҵ���ĸ���
uint16 data_stastics_r = 0;//ͳ���ұ��ҵ���ĸ���
uint8 hightest = 0;//��ߵ�
void search_l_r(uint16 break_flag, uint8(*image)[LCDW], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{
   //printf("���");
    uint8 i = 0, j = 0;

    //��߱���
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics ;//ͳ�����

    //����˸�����
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //�����˳ʱ��

    //�ұ߱���
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//���������
    uint8 index_r = 0;//�����±�
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics ;//ͳ���ұ�
    //����˸�����
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //�������ʱ��

    l_data_statics = *l_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������
    r_data_statics = *r_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������
   // printf("l_data_statics = %d \n", l_data_statics);
    //��һ�θ��������  ���ҵ������ֵ������
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y
//printf("δ��while");
        //��������ѭ��
    while (break_flag--)
    {
//printf("break_flag = %d \n",break_flag);
        //���
        for (i = 0; i < 8; i++)//����8F����
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //�����������䵽�Ѿ��ҵ��ĵ���
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//������һ

        //�ұ�
        for (i = 0; i < 8; i++)//����8F����
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //�����������䵽�Ѿ��ҵ��ĵ���
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        index_l = 0;//�����㣬��ʹ��
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//�����㣬��ʹ��
            temp_l[i][1] = 0;//�����㣬��ʹ��
        }

        //����ж�
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i);//��¼��������
            }

            if (index_l)
            {
                //���������
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
//                ips200_draw_point(  center_point_l[0],  center_point_l[1], RGB565_GREEN);
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
    && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
    ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
        && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
{
//            printf("���ν���ͬһ���㣬�˳�\n");
    break;
}
if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
    && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
    )
{
//            printf("\n���������˳�\n");
    *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//ȡ����ߵ�
//            printf("\n��y=%d���˳�\n",*hightest);
    break;
}
if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
{
//            printf("\n�����߱��ұ߸��ˣ���ߵȴ��ұ�\n");
    continue;//�����߱��ұ߸��ˣ���ߵȴ��ұ�
}
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//��߱��ұ߸����Ѿ�����������
        {
//            printf("\n��߿�ʼ�����ˣ��ȴ��ұߣ��ȴ���... \n");
            center_point_l[0] =(uint8) points_l[l_data_statics - 1][0];//x
            center_point_l[1] =(uint8) points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//������һ

        index_r = 0;//�����㣬��ʹ��
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//�����㣬��ʹ��
            temp_r[i][1] = 0;//�����㣬��ʹ��
        }

        //�ұ��ж�
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//������һ
                dir_r[r_data_statics - 1] = (i);//��¼��������
//                printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                //���������
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
//                ips200_draw_point( center_point_r[0],  center_point_r[1], RGB565_GREEN);
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }

            }
        }


    }


    //ȡ��ѭ������
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;

}
uint8 l_border[LCDH];//��������
uint8 r_border[LCDH];//��������
uint8 center_line[LCDH];//��������
void get_left(uint16 total_L, uint8_t imageOut[LCDH][2])
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    //��ʼ��
    for (i = 0;i<LCDH;i++)
    {
        imageOut[i][0] = border_min;
    }
    h = LCDH - 2;
    //���
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            imageOut[h][0] = points_l[j][0]+1;
        }
        else continue; //ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0)
        {
            break;//�����һ���˳�
        }
    }
}
void get_right(uint16 total_R, uint8_t imageOut[LCDH][2])
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < LCDH; i++)
    {
        imageOut[i][1] = border_max;//�ұ��߳�ʼ���ŵ����ұߣ�����߷ŵ�����ߣ�����������պ�����������߾ͻ����м䣬������ŵõ�������
    }
    h = LCDH - 2;
    //�ұ�
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            imageOut[h][1] = points_r[j][0] - 1;
        }
        else continue;//ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0)break;//�����һ���˳�
    }
}
void image_draw_rectan(uint8(*image)[LCDW])
{

    uint8 i = 0;
    for (i = 0; i < LCDH; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][LCDW - 1] = 0;
        image[i][LCDW - 2] = 0;

    }
    for (i = 0; i < LCDW; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
        //image[image_h-1][i] = 0;

    }
}


void enh_process(void)
{
    image_draw_rectan(Bin_Image);
    data_stastics_l = 0;
    data_stastics_r = 0;
    if ( get_start_point(LCDH-2))//�ҵ�����ˣ���ִ�а�����û�ҵ���һֱ��
        {

       // printf("���ڿ�ʼ������\n");
        search_l_r((uint16)USE_num, Bin_Image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
        //printf("�������ѽ���\n");
        // ����ȡ�ı߽�������ȡ���� �� ��������������õı���
        get_left(data_stastics_l,ImageSide);
        get_right(data_stastics_r, ImageSide);
        //�������������Ҫ�ŵ�if����ȥ�ˣ���Ҫ�ŵ�if����ȥ�ˣ���Ҫ�ŵ�if����ȥ�ˣ���Ҫ����˵����

    }
}



