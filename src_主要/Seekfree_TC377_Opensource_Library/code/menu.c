/*
 * menu.c
 *
 *  Created on: 2024年8月31日
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

int32 menu_flag = 1;        //菜单标志位
float value = 0.01;

//-------------------------------------------------------------------------------------------------------------------
//  函数简介
//  参数说明
//  返回参数
//  使用示例
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void menu1_show(void)
{
    static uint16 cursor_x = 0;
    static uint16 cursor_y = 0;

    ips200_show_char(cursor_x, cursor_y, '>');

    ips200_show_string(10, 0,   "HUAT_FZY");
    ips200_show_string(10, 20,  "CAMERA"  );
    ips200_show_string(10, 40,  "ENCODER" );
    ips200_show_string(10, 60,  "IMU660RA");
    ips200_show_string(10, 80,  "DEBUG"   );
    ips200_show_string(10, 100, "CONTROL" );

    while (1)
    {
        if (key_number == 1)
        {
            cursor_y += 20;
            cursor_y = (uint16)my_limit((uint16)cursor_y, 0, 100);
            ips200_clear();

            break;
        }
        if (key_number == 2)
        {
            cursor_y -= 20;
            cursor_y = (uint16)my_limit((uint16)cursor_y, 0, 100);
            ips200_clear();

            break;
        }

        if (key_number == 3)
        {
            switch (cursor_y)
            {
                case 20  : menu_flag = 2;  break;
                case 40  : menu_flag = 3;  break;
                case 60  : menu_flag = 4;  break;
                case 80  : menu_flag = 5;  break;
                case 100 : menu_flag = 6;  break;
            }
            ips200_clear();

            break;
        }
        cursor_y = (uint16)my_limit((uint16)cursor_y, 0, 100);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介
//  参数说明
//  返回参数
//  使用示例
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void menu2_show(void)
{
    ips200_show_gray_image(0, 0, change_image_show[0], CHANGE_COL, CHANGE_ROW, CHANGE_COL, CHANGE_ROW, 0);
    ips200_show_gray_image(0, 180, mt9v03x_image[0], COL, ROW, COL, ROW, threshold);
    line_show();
    ips200_show_string(0, 80, "left_flag :");             ips200_show_int  (100, 80, left_flag, 3);
    ips200_show_string(0, 100,"right_flag:");             ips200_show_int  (100, 100,right_flag, 3);
//    ips200_show_string(0, 100, "element    :");         ips200_show_int  (100, 100, element_flag, 3);
    ips200_show_string(0, 120, "turn_error :");         ips200_show_float(100, 120, turn_error, 3, 2);
    ips200_show_string(0, 140, "pitch_angle:");         ips200_show_float(100, 140, eulerAngle.pitch, 3, 2);
    ips200_show_string(0, 160, "yaw_angle  :");         ips200_show_float(100, 160, imu.gyro.angle[Z], 3, 2);

    if (key_number == 5)
    {
        menu_flag = 1;
        ips200_clear();
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介
//  参数说明
//  返回参数
//  使用示例
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void menu3_show(void)
{
    ips200_show_string(0, 0,   "encoder_data_1   : ");    ips200_show_int(150, 0, encoder_data_1, 3);
    ips200_show_string(0, 20,  "encoder_data_2   : ");    ips200_show_int(150, 20, encoder_data_2, 3);
    ips200_show_string(0, 40,  "encoder_data_temp: ");    ips200_show_int(150, 40, encoder_data_temp, 3);

    if (key_number == 4)
    {
        menu_flag = 1;
        ips200_clear();
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介
//  参数说明
//  返回参数
//  使用示例
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void menu4_show(void)
{
    ips200_show_string(0, 0,   "gyro_x     :");         ips200_show_int(150, 0,     imu.gyro.gyro[X] * 100, 4);
    ips200_show_string(0, 20,  "gyro_y     :");         ips200_show_int(150, 20,    imu.gyro.gyro[Y] * 100, 4);
    ips200_show_string(0, 40,  "gyro_z     :");         ips200_show_int(150, 40,    imu.gyro.gyro[Z] * 100, 4);
    ips200_show_string(0, 60,  "acc_x      :");         ips200_show_int(150, 60,    imu660ra_acc_x, 3);
    ips200_show_string(0, 80,  "acc_y      :");         ips200_show_int(150, 80,    imu660ra_acc_y, 3);
    ips200_show_string(0, 100, "acc_z      :");         ips200_show_int(150, 100,   imu660ra_acc_z, 3);
    ips200_show_string(0, 120, "pitch_angle:");         ips200_show_float(150, 120, eulerAngle.pitch, 3, 2);
    ips200_show_string(0, 140, "yaw_angle  :");         ips200_show_float(150, 140, imu.gyro.angle[Z], 3, 2);
    ips200_show_string(0, 160, "roll_angle :");         ips200_show_float(150, 160, eulerAngle.roll, 3, 2);

    if (key_number == 4)
    {
        menu_flag = 1;
        ips200_clear();
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介
//  参数说明
//  返回参数
//  使用示例
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void menu5_show(void)
{
    ips200_show_string(0, 0,  "duty_l    :");                 ips200_show_int  (120, 0,  duty_l, 5);
    ips200_show_string(0, 20, "duty_r    :");                 ips200_show_int  (120, 20, duty_r, 5);
    ips200_show_string(0, 40, "angle.out :");                 ips200_show_int  (120, 40, angle.out, 5);
    ips200_show_string(0, 60, "gyro.out  :");                 ips200_show_int  (120, 60, gyro.out, 5);
    ips200_show_string(0, 80, "speed.out :");                 ips200_show_int  (120, 80, speed.out, 5);
    ips200_show_string(0, 100,"dir_camera.out:");             ips200_show_int  (120, 100, dir_camera.out, 5);
    ips200_show_string(0, 120,"dir_gyro.out  :");             ips200_show_int  (120, 120, dir_gyro.out, 5);

    if (key_number == 4)
    {
        menu_flag = 1;
        ips200_clear();
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介
//  参数说明
//  返回参数
//  使用示例
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void menu6_show(void)
{
    static uint16 cursor_x = 0;
    static uint16 cursor_y = 0;

    ips200_show_char(cursor_x, cursor_y, '>');

    ips200_show_string(10, 0,   "speed.kp     :");         ips200_show_float(120, 0, speed.kp, 2, 2);
    ips200_show_string(10, 20,  "speed.ki     :");         ips200_show_float(120, 20, speed.ki, 2, 2);
    ips200_show_string(10, 40,  "angle.kp     :");         ips200_show_float(120, 40, angle.kp, 2, 2);
    ips200_show_string(10, 60,  "angle.kd     :");         ips200_show_float(120, 60, angle.kd, 2, 2);
    ips200_show_string(10, 80,  "gyro.kp      :");         ips200_show_float(120, 80, gyro.kp, 2, 2);
    ips200_show_string(10, 100, "gyro.ki      :");         ips200_show_float(120, 100, gyro.ki, 2, 2);
    ips200_show_string(10, 120, "dir_camera.kp:");         ips200_show_float(120, 120, dir_camera.kp, 2, 2);
    ips200_show_string(10, 140, "dir_camera.kd:");         ips200_show_float(120, 140, dir_camera.kd, 2, 2);
    ips200_show_string(10, 160, "dir_gyro.kp  :");         ips200_show_float(120, 160, dir_gyro.kp, 2, 2);
    ips200_show_string(10, 180, "dir_gyro.ki  :");         ips200_show_float(120, 180, dir_gyro.ki, 2, 2);
    ips200_show_string(10, 200, "division     :");         ips200_show_float(120, 200, value, 2, 2);
    ips200_show_string(10, 220, "expect_speed :");         ips200_show_int  (120, 220, expect_speed, 5);

    while (1)
    {
        if (key_number == 1)
        {
            cursor_y += 20;
            cursor_y = (uint16)my_limit((uint16)cursor_y, 0, 220);
            ips200_clear();

            break;
        }
        if (key_number == 2)
        {
            cursor_y -= 20;
            cursor_y = (uint16)my_limit((uint16)cursor_y, 0, 220);
            ips200_clear();

            break;
        }
        if (key_number == 3)
        {
            switch (cursor_y)
            {
                case 0   : speed.kp += value;      break;
                case 20  : speed.ki += value;      break;
                case 40  : angle.kp += value;      break;
                case 60  : angle.kd += value;      break;
                case 80  : gyro.kp  += value;      break;
                case 100 : gyro.ki  += value;      break;
                case 120 : dir_camera.kp  += value;break;
                case 140 : dir_camera.kd  += value;break;
                case 160 : dir_gyro.kp  += value;  break;
                case 180 : dir_gyro.ki  += value;  break;
                case 200 : value *= 10;            break;
                case 220 : expect_speed += 5;      break;
            }
            eeprom_writ();
            ips200_clear();

            break;
        }
        if (key_number == 4)
        {
            switch (cursor_y)
            {
                case 0   : speed.kp -= value;      break;
                case 20  : speed.ki -= value;      break;
                case 40  : angle.kp -= value;      break;
                case 60  : angle.kd -= value;      break;
                case 80  : gyro.kp  -= value;      break;
                case 100 : gyro.ki  -= value;      break;
                case 120 : dir_camera.kp  -= value;break;
                case 140 : dir_camera.kd  -= value;break;
                case 160 : dir_gyro.kp  -= value;  break;
                case 180 : dir_gyro.ki  -= value;  break;
                case 200 : value /= 10;            break;
                case 220 : expect_speed -= 5;      break;
            }
            eeprom_writ();
            ips200_clear();

            break;
        }
        if (key_number == 5)
        {
            menu_flag = 1;
            ips200_clear();

            break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介
//  参数说明
//  返回参数
//  使用示例
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void menu_show(void)
{
    switch (menu_flag)
    {
        case 1 : menu1_show();      break;
        case 2 : menu2_show();      break;
        case 3 : menu3_show();      break;
        case 4 : menu4_show();      break;
        case 5 : menu5_show();      break;
        case 6 : menu6_show();      break;
    }
}
