/*
 * menu.h
 *
 *  Created on: 2024年8月18日
 *      Author: shuyu
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_
#include "zf_common_headfile.h"
extern int place_index;
extern int value_index;
extern int last_place_index;
extern int allow_value_show ;
extern int allow_image_show ;
extern int key_pit_flag;
#define Font_size_H 16
#define Font_size_W 8
void adjust_menu(void);
void control_menu(void);
void image_menu(void);
void function_menu(void);
#endif /* CODE_MENU_H_ */
