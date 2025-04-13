/*
 * image.c
 *
 *  Created on: 2024��8��27��
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

uint8 *change_image_ip [CHANGE_ROW][CHANGE_COL];                                        // ��͸�Ӻ�ͼ���ַ
uint8 change_image_show[CHANGE_ROW][CHANGE_COL];                                        // ������ʾ��͸�Ӻ��ͼ��
uint8 threshold;                                                                        // ��̬��ֵ
uint16 change_x, change_y;                                                              // ԭͼ��������Ӧ��͸�Ӻ�������
uint16 left_count, right_count, middle_count;                                           // ���ұ��ߵ���
uint16 change_left_count, change_right_count,change_middle_count;                       // ��͸�Ӻ����ұ��ߵ���
uint16 left_line[300][2], right_line[300][2], middle_line[300][2];                      // ���ұ�������
uint16 change_left_line[300][2], change_right_line[300][2], change_middle_line[300][2]; // ��͸�Ӻ����ұ�������
line   left_flag, right_flag;                                                           // �����жϱ�־λ
uint16 middle_line_flag;                                                                //
uint16 left_lose_count, right_lose_count;
uint16 left_jump_count, right_jump_count;                                               // ���ұ�����������
uint16 left_jump_point[30][2], right_jump_point[30][2];                                 // ���ұ��������
uint16 track_width[CHANGE_ROW];                                                         // �������
float turn_error;                                                                       // ͼ��ƫ��

//-------------------------------------------------------------------------------------------------------------------
//  �������      ȫͼ��͸��
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      image_change_init();
//  ��ע��Ϣ      ��ʼ��һ�μ���
//-------------------------------------------------------------------------------------------------------------------
void image_change_init(void)
{
    static uint8 blackcolor = 255;                                                 // ͼ�������ݲ��ֵĻҶ�ֵ
    double change_un_mat[3][3] ={{0.695602,-0.712314,26.234660},
                                 {0.000000,0.186862,6.033990  },
                                 {-0.000000,-0.007773,0.749002}};

    for (int x = 0; x < CHANGE_COL; x++)                                           // ͨ����͸�Ӻ��ͼ��ĵ��������о������������Ӧ��ԭͼ�ĵ������
    {
        for (int y = 0; y < CHANGE_ROW; y++)
        {
            int local_x = (int) ((change_un_mat[0][0] * x
                                + change_un_mat[0][1] * y + change_un_mat[0][2])
                                / (change_un_mat[2][0] * x + change_un_mat[2][1] * y
                                + change_un_mat[2][2]));

            int local_y = (int) ((change_un_mat[1][0] * x
                                + change_un_mat[1][1] * y + change_un_mat[1][2])
                                / (change_un_mat[2][0] * x + change_un_mat[2][1] * y
                                + change_un_mat[2][2]));

            if (local_x >= 0 && local_y >= 0 && local_x < COL && local_y < ROW)
            {
                change_image_ip[y][x] = &mt9v03x_image[local_y][local_x];          // ��ԭͼ��Ϣ����͸�Ӻ��ͼ��������
            }
            else
            {
                change_image_ip[y][x] = &blackcolor;                               // �����͸�Ӻ��ͼ��������ݲ���
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  �������      ����任����
//  ����˵��      *mat
//  ����˵��      *x
//  ����˵��      *y
//  ���ز���      void
//  ʹ��ʾ��      solve_point(mat, &solve_x, &solve_y);
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void solve_point(float *mat, float *x, float *y)
{
    float a1 = mat[0];
    float b1 = mat[1];
    float c1 = -mat[2];
    float a2 = mat[3];
    float b2 = mat[4];
    float c2 = -mat[5];

    *x = (b1 * c2 - b2 * c1) / (b1 * a2 - b2 * a1);
    *y = (c1 * a2 - c2 * a1) / (b1 * a2 - b2 * a1);
}

//-------------------------------------------------------------------------------------------------------------------
//  �������      ͨ��ԭͼ������������͸�Ӻ�������
//  ����˵��      y
//  ����˵��      x
//  ���ز���      void
//  ʹ��ʾ��      change_point_coord(y, x);
//  ��ע��Ϣ      ���ڱ�����͸��
//-------------------------------------------------------------------------------------------------------------------
void change_point_coord(uint16 y, uint16 x)
{
    float mat[6], solve_x, solve_y;
    double change_un_mat[3][3] ={{0.695602,-0.712314,26.234660},
                                 {0.000000,0.186862,6.033990  },
                                 {-0.000000,-0.007773,0.749002}};

    mat[0] = change_un_mat[2][0] * y - change_un_mat[1][0];
    mat[1] = change_un_mat[2][1] * y - change_un_mat[1][1];
    mat[2] = change_un_mat[2][2] * y - change_un_mat[1][2];
    mat[3] = change_un_mat[2][0] * x - change_un_mat[0][0];
    mat[4] = change_un_mat[2][1] * x - change_un_mat[0][1];
    mat[5] = change_un_mat[2][2] * x - change_un_mat[0][2];

    solve_point(mat, &solve_x, &solve_y);

    change_x = (int)solve_x;
    change_y = (int)solve_y;
}

//-------------------------------------------------------------------------------------------------------------------
// �������
// ����˵��
// ���ز���
// ʹ��ʾ��
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
//void change_image_init(void)
//{
//    for (int y = 0; y < ROW; y++)
//    {
//        for (int x = 0; x < COL; x++)
//        {
//            change_point_coord(y, x);
//
//            if (change_x >= 0 && change_y >= 0 && change_x < CHANGE_COL && change_y < CHANGE_ROW)
//            {
//
//            }
//        }
//    }
//}

//-------------------------------------------------------------------------------------------------------------------
// �������     �����̬��ֵ
// ����˵��     *image
// ����˵��     row
// ����˵��     col
// ���ز���     threshold
// ʹ��ʾ��     otsu_threshold(image, row, col);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 otsu_threshold(uint8 *image, uint16 row, uint16 col)
{
    #define GrayScale 256
    uint16  height= row;
    uint16  width = col;
    int     pixelCount[GrayScale];
    float   pixelPro  [GrayScale];
    int     i, j, pixelSum = width * height / 4;
    uint8   threshold = 0;
    uint8   *data = image;

    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro  [i] = 0;
    }

    uint32 gray_sum = 0;

    for (i = 0; i < height; i += 2)
    {
        for (j = 0 ; j < width; j += 2)
        {
            pixelCount [(int)data[i * width + j]]++;                    // ����ǰ�ĵ������ֵ��Ϊ����������±�
            gray_sum += (int)data[i * width + j];                       // �Ҷ�ֵ�ܺ�
        }
    }

   for (i = 0; i < GrayScale; i++)
   {
       pixelPro[i] = (float)pixelCount[i] / pixelSum;
   }

   float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;

   w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;

   for (j = 0; j < GrayScale; j++)
   {
       w0 += pixelPro[j];                                               // ��������ÿ���Ҷ�ֵ�����ص���ռ����֮��   ���������ֵı���
       u0tmp += j * pixelPro[j];                                        // �������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ

       w1 = 1 - w0;
       u1tmp=gray_sum / pixelSum-u0tmp;

       u0 = u0tmp / w0;                                                 // ����ƽ���Ҷ�
       u1 = u1tmp / w1;                                                 // ǰ��ƽ���Ҷ�
       u  = u0tmp + u1tmp;                                              // ȫ��ƽ���Ҷ�

       deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);

       if (deltaTmp > deltaMax)
       {
           deltaMax  = deltaTmp;
           threshold = (uint8)j;
       }
       if (deltaTmp < deltaMax)
       {
           break;
       }
   }

   return threshold;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��ֵ��ͼ��
// ����˵��     threshold
// ����˵��     *image
// ����˵��     row
// ����˵��     col
// ���ز���     void
// ʹ��ʾ��     get_bin_image(threshold, image, row, col);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void get_bin_image(uint8 threshold, uint8 *image, uint16 row, uint16 col)
{
   for (uint16 y = 0; y < row; y++)
   {
       for (uint16 x = 0; x < col; x++)
       {
           if (image[y * col + x] < threshold)
           {
               image[y * col + x] = 0;
           }
           else
           {
               image[y * col + x] = 255;
           }
       }
   }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ͼ��һ���ڿ�
// ����˵��     *image
// ����˵��     row
// ����˵��     col
// ���ز���     void
// ʹ��ʾ��     draw_black_frame(image, row, col);
// ��ע��Ϣ     ��ͼ��һ���ڿ��ֹ��ȡ��������ʱ����
//-------------------------------------------------------------------------------------------------------------------
void draw_black_frame(uint8 *image, uint16 row, uint16 col)
{
    for (uint16 y = 0; y < row; y++)
    {
        image[y * col + 0] = 0;
        image[y * col + (col - 1)] = 0;
    }
    for (uint16 x = 0; x < col; x++)
    {
        image[0 * col + x] = 0;
        image[(row - 1) * col + x] = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��������
// ����˵��     *image
// ����˵��     row
// ����˵��     col
// ���ز���     void
// ʹ��ʾ��     climb_line(image, row, col);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void climb_line(uint8 *image, uint16 row, uint16 col)
{
    uint16 all_count = 300;
    uint16 centerpoint_l[2] = {0};
    uint16 centerpoint_r[2] = {0};
    uint16 search_line_l[8][2] = {{0}};
    uint16 search_line_r[8][2] = {{0}};
    uint16 left_dir [8][2] = {{1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1,  1}, {0,  1}, {1,  1}};
    uint16 right_dir[8][2] = {{1, 0}, {1,  1}, {0,  1}, {-1,  1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
    uint16 left_line_h  = 0;
    uint16 right_line_h = 0;

    for (uint16 i = 0; i < left_count; i++)
    {
        left_line[i][0] = row - 2;
        left_line[i][1] = 1;
    }
    left_count = 0;

    for (uint16 i = 0; i < change_left_count; i++)
    {
        change_left_line[i][0] = CHANGE_ROW - 1 - i;
        change_left_line[i][1] = 0;
    }
    change_left_count = 0;

    for (uint16 i = 0; i < right_count; i++)
    {
        right_line[i][0] = row - 2;
        right_line[i][1] = col - 2;
    }
    right_count = 0;

    for (uint16 i = 0; i < change_right_count; i++)
    {
        change_right_line[i][0] = CHANGE_ROW - 1 - i;
        change_right_line[i][1] = CHANGE_COL - 1;
    }
    change_right_count = 0;

    for (uint16 i = 0; i < change_middle_count; i++)
    {
        change_middle_line[i][0] = CHANGE_ROW - 1 - i;
        change_middle_line[i][1] = CHANGE_COL / 2;
    }
    change_middle_count = 0;


    for (uint16 i = col / 2; i > 0; i--)
    {
        if (image[(row - 2) * col + i] == 255 && image[(row - 2) * col + (i - 1)] == 0)
        {
            centerpoint_l[0] = row - 2;
            centerpoint_l[1] = i;
            left_count++;

            change_point_coord(centerpoint_l[0], centerpoint_l[1]);
            if (change_x >= 0 && change_y >= 0 && change_x < CHANGE_COL && change_y < CHANGE_ROW)
            {
                change_left_line[change_left_count][0] = change_y;
                change_left_line[change_left_count][1] = change_x;
                change_left_count++;
            }
            else
            {
                change_left_count++;
            }

            break;
        }
    }

    for (uint16 i = col / 2; i < col - 1; i++)
    {
        if (image[(row - 2) * col + i] == 255 && image[(row - 2) * col + (i + 1)] == 0)
        {
            centerpoint_r[0] = row - 2;
            centerpoint_r[1] = i;
            right_count++;

            change_point_coord(centerpoint_r[0], centerpoint_r[1]);
            if (change_x >= 0 && change_y >= 0 && change_x < CHANGE_COL && change_y < CHANGE_ROW)
            {
                change_right_line[change_right_count][0] = change_y;
                change_right_line[change_right_count][1] = change_x;
                change_right_count++;
            }
            else
            {
                change_right_count++;
            }

            break;
        }
    }

    if (left_count)
    {
        left_line[0][0] = centerpoint_l[0];
        left_line[0][1] = centerpoint_l[1];
    }

    if (right_count)
    {
        right_line[0][0] = centerpoint_r[0];
        right_line[0][1] = centerpoint_r[1];
    }

    left_line_h  = centerpoint_l[0];
    right_line_h = centerpoint_r[0];

    while ((left_count && right_count) && --all_count)
    {
        for (uint16 i = 0; i < 8; i++)
        {
            search_line_l[i][0] = centerpoint_l[0] + left_dir[i][0];
            search_line_l[i][1] = centerpoint_l[1] + left_dir[i][1];
        }

        if (left_line_h >= right_line_h)
        {
            for (uint16 i = 0; i < 8; i++)
            {
                if (image[search_line_l[i][0] * col + search_line_l[i][1]] == 0 &&
                    image[search_line_l[(i + 1) & 7][0] * col + search_line_l[(i + 1) & 7][1]] == 255)
                {
                    left_line[left_count][0] = search_line_l[(i + 1) & 7][0];
                    left_line[left_count][1] = search_line_l[(i + 1) & 7][1];

                    change_point_coord(left_line[left_count][0], left_line[left_count][1]);
                    if (change_x >= 0 && change_y >= 0 && change_x < CHANGE_COL && change_y < CHANGE_ROW)
                    {
                        change_left_line[change_left_count][0] = change_y;
                        change_left_line[change_left_count][1] = change_x;
                        change_left_count++;
                    }
                    else
                    {
                        change_left_line[change_left_count][0] = change_left_line[change_left_count - 1][0] - 1;
                        change_left_line[change_left_count][1] = 0;
                        change_left_count++;
                    }

                    left_count++;

                    centerpoint_l[0] = search_line_l[(i + 1) & 7][0];
                    centerpoint_l[1] = search_line_l[(i + 1) & 7][1];

                    left_line_h = centerpoint_l[0];

                    break;
                }
            }
        }

        for (uint16 i = 0; i < 8; i++)
        {
            search_line_r[i][0] = centerpoint_r[0] + right_dir[i][0];
            search_line_r[i][1] = centerpoint_r[1] + right_dir[i][1];
        }

        if (right_line_h >= left_line_h)
        {
            for (uint16 i = 0; i < 8; i++)
            {
                if (image[search_line_r[i][0] * col + search_line_r[i][1]] == 0 &&
                    image[search_line_r[(i + 1) & 7][0] * col + search_line_r[(i + 1) & 7][1]] == 255)
                {
                    right_line[right_count][0] = search_line_r[(i + 1) & 7][0];
                    right_line[right_count][1] = search_line_r[(i + 1) & 7][1];

                    change_point_coord(right_line[right_count][0], right_line[right_count][1]);
                    if (change_x >= 0 && change_y >= 0 && change_x < CHANGE_COL && change_y < CHANGE_ROW)
                    {
                        change_right_line[change_right_count][0] = change_y;
                        change_right_line[change_right_count][1] = change_x;
                        change_right_count++;
                    }
                    else
                    {
                        change_right_line[change_right_count][0] = change_right_line[change_right_count - 1][0] - 1;
                        change_right_line[change_right_count][1] = CHANGE_COL - 1;
                        change_right_count++;
                    }

                    right_count++;

                    centerpoint_r[0] = search_line_r[(i + 1) & 7][0];
                    centerpoint_r[1] = search_line_r[(i + 1) & 7][1];

                    right_line_h = centerpoint_r[0];

                    break;
                }
            }
        }

        change_middle_count++;

        if ((left_line[left_count - 1][0] == right_line[right_count - 1][0] && left_line[left_count - 1][1] == right_line[right_count - 1][1]) ||
            (left_line[left_count - 1][0] == right_line[right_count - 1][0] && left_line[left_count - 1][1] - 1 == right_line[right_count - 1][1]))
        {
            break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������
// ����˵��
// ���ز���
// ʹ��ʾ��
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void get_track_information(void)
{
    uint16 left_value [30] = {0};                       // ��¼����ߵ������
    uint16 right_value[30] = {0};                       // ��¼�ұ��ߵ������

    for (uint16 i = 0; i < CHANGE_ROW; i++)
    {
        track_width[i] = 0;
    }

    for (uint16 i = 0; i < left_jump_count; i++)
    {
        left_jump_point[i][0] = CHANGE_ROW - 1;
        left_jump_point[i][1] = 0;
    }
    left_jump_count = 0;

    for (uint16 i = 0; i < right_jump_count; i++)
    {
        right_jump_point[i][0] = CHANGE_ROW - 1;
        right_jump_point[i][1] = CHANGE_COL - 1;
    }
    right_jump_count = 0;

    for (uint16 i = 0; i < CHANGE_ROW; i++)
    {
        track_width[i] = change_right_line[i][1] - change_left_line[i][1];
    }

    for (uint16 i = 10, j = 0; i < change_left_count; i += 10, j++)
    {
        left_value[j]  = (uint16)my_abs(change_left_line[i][1] - change_left_line[i - 10][1]);

        if (left_value[j] >= 7)
        {
            left_jump_point[left_jump_count][0] = change_left_line[i - 10][0];
            left_jump_point[left_jump_count][1] = change_left_line[i - 10][1];
            left_jump_count++;
        }
    }

    for (uint16 i = 10, j = 0; i < change_right_count; i += 10, j++)
    {
        right_value[j] = (uint16)my_abs(change_right_line[i][1] - change_right_line[i - 10][1]);

        if (right_value[j] >= 7)
        {
            right_jump_point[right_jump_count][0] = change_right_line[i - 10][0];
            right_jump_point[right_jump_count][1] = change_right_line[i - 10][1];
            right_jump_count++;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������
// ����˵��
// ���ز���
// ʹ��ʾ��
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void track_identify(void)
{

}

//-------------------------------------------------------------------------------------------------------------------
// �������
// ����˵��
// ���ز���
// ʹ��ʾ��
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void get_middle_line(void)
{
    for (uint16 i = 0; i < ROW; i++)
    {
        middle_line[i][0] = left_line[i][0];
        middle_line[i][1] = (uint16)((left_line[i][1] + right_line[i][1]) / 2);
    }

    for (uint16 i = 0; i < change_middle_count; i++)
    {
        change_middle_line[i][0] = change_left_line[i][0];
        change_middle_line[i][1] = (uint16)((change_left_line[i][1] + change_right_line[i][1]) / 2);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʾ����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     line_show();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void line_show(void)
{
    for (uint16 i = 0; i < change_left_count; i++)
    {
        change_left_line[i][1] = (uint16)my_limit(change_left_line[i][1], 0, CHANGE_COL - 2);
        change_left_line[i][0] = (uint16)my_limit(change_left_line[i][0], 0, CHANGE_ROW - 1);

        ips200_draw_point(change_left_line[i][1], change_left_line[i][0], RGB565_RED);
        ips200_draw_point(change_left_line[i][1] + 1, change_left_line[i][0], RGB565_RED);
    }

    for (uint16 i = 0; i < change_right_count; i++)
    {
        change_right_line[i][1] = (uint16)my_limit(change_right_line[i][1], 1, CHANGE_COL - 1);
        change_right_line[i][0] = (uint16)my_limit(change_right_line[i][0], 0, CHANGE_ROW - 1);

        ips200_draw_point(change_right_line[i][1], change_right_line[i][0], RGB565_BLUE);
        ips200_draw_point(change_right_line[i][1] - 1, change_right_line[i][0], RGB565_BLUE);
    }

    for (uint16 i = 0; i < (uint16)((change_left_count + change_right_count) / 2); i++)
    {
        change_middle_line[i][1] = (uint16)my_limit(change_middle_line[i][1], change_left_line[i][1], change_right_line[i][1]);
        change_middle_line[i][0] = (uint16)my_limit(change_middle_line[i][0], 0, CHANGE_ROW - 1);

        ips200_draw_point(change_middle_line[i][1], change_middle_line[i][0], RGB565_YELLOW);
    }

    for (uint16 i = 0; i < left_count; i++)
    {
        ips200_draw_point(left_line[i][1], left_line[i][0] + 180, RGB565_RED);
    }

    for (uint16 i = 0; i < right_count; i++)
    {
        ips200_draw_point(right_line[i][1], right_line[i][0] + 180, RGB565_BLUE);
    }

    for (uint16 i = 0; i < ROW; i++)
    {
        ips200_draw_point(middle_line[i][1], middle_line[i][0] + 180, RGB565_YELLOW);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ͼ����
// ����˵��     *image
// ����˵��     row
// ����˵��     col
// ���ز���     void
// ʹ��ʾ��     image_process(mt9v03x_image[0], ROW, COL);
// ��ע��Ϣ     ͼ����ֻ����øú���
//-------------------------------------------------------------------------------------------------------------------
void image_process(uint8 *image, uint16 row, uint16 col)
{
    threshold = otsu_threshold(image, row, col);

    get_bin_image(threshold, image, row, col);

    draw_black_frame(image, row, col);

    climb_line(image, row, col);

    get_track_information();

    get_middle_line();
}

//-------------------------------------------------------------------------------------------------------------------
// �������
// ����˵��
// ���ز���
// ʹ��ʾ��
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
float get_turn_error(void)
{
    int16 sum_weight = 0;
    int16 sum_middle_line = 0;
    int16 avg_middle;
    static float last_error, before_last_error;
    const uint8 weight[CHANGE_ROW] =
    {
        0,0,0,0,0,0,0,0,0,1,
        1,1,1,1,1,1,1,1,1,1,
        2,2,2,2,2,2,2,2,2,2,
        3,3,3,6,6,6,6,5,5,5,
        3,3,3,3,3,3,2,2,2,2,
        1,1,1,1,1,1,1,1,1,1,
        1,1,0,0,0,0,0,0,0,0,
    };

    for (uint16 i = 0; i < CHANGE_ROW; i++)
    {
        sum_weight += weight[i];
        sum_middle_line += weight[i] * change_middle_line[i][1];
    }

    avg_middle = (float)sum_middle_line / sum_weight;
    turn_error = (float)(CHANGE_COL / 2 - avg_middle);

    turn_error = 0.2 * turn_error + last_error * 0.3 + before_last_error * 0.5;
    before_last_error = last_error;
    last_error = turn_error;

    return turn_error;
}
