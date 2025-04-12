/*
 * image.h
 *
 *  Created on: 2024��8��27��
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#define ROW MT9V03X_H               // ԭʼͼ����(��)90
#define COL MT9V03X_W               // ԭʼͼ����(��)180
#define CHANGE_ROW 70               // ��͸�Ӻ��ͼ����(��)
#define CHANGE_COL 120              // ��͸�Ӻ��ͼ����(��)

typedef enum
{
    straight,                       // ֱ��
    arc,                            // Բ��
    broken,                         // ����
    lose,                           // ����
}line;

extern uint8 *change_image_ip [CHANGE_ROW][CHANGE_COL];
extern uint8 change_image_show[CHANGE_ROW][CHANGE_COL];
extern uint8 threshold;
extern uint16 left_count;
extern uint16 right_count;
extern uint16 left_jump_count;
extern uint16 right_jump_count;
extern line left_flag, right_flag;
extern uint16 middle_line_flag;
extern float turn_error;

void  image_change_init(void);
void  line_show        (void);
float get_turn_error   (void);
void  image_process    (uint8 *image, uint16 row, uint16 col);

#endif /* CODE_IMAGE_H_ */
