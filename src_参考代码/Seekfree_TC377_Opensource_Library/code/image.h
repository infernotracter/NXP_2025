/*
 * image.h
 *
 *  Created on: 2024年8月27日
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#define ROW MT9V03X_H               // 原始图像行(高)90
#define COL MT9V03X_W               // 原始图像列(宽)180
#define CHANGE_ROW 70               // 逆透视后的图像行(高)
#define CHANGE_COL 120              // 逆透视后的图像列(宽)

typedef enum
{
    straight,                       // 直线
    arc,                            // 圆弧
    broken,                         // 断线
    lose,                           // 丢线
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
