/*
 * menu.c
 *
 *  Created on: 2024年8月18日
 *      Author: shuyu
 */
#include "menu.h"
int place_index = 0 ;
int value_index = 0;
int last_place_index = 0;
int allow_value_show = 0;
int allow_image_show = 0;
int key_pit_flag = 1;

/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////

void cursor_selected(int max_index) {
    if (place_index >= max_index) {
        place_index = 0;
    } else if (place_index <= -1) {
        place_index = max_index - 1;
    }

    for (int index = 0; index < 20; index++) {
        if (index != place_index + 1) {
            // 假设"  "表示两个空格
            ips200_show_string(0, index * Font_size_H, "  ");
        }
    }
    // 显示箭头"->"，这里假设箭头字符在字体中已定义，或需要替换为实际字符代码
    ips200_show_string(0, (place_index + 1) * Font_size_H, "->");
    last_place_index = place_index;
}



void show_string_value(uint16 base_y, uint32 value, uint8 num_digits, const char* str) {
    ips200_show_string(20, base_y*16, str);
    ips200_show_uint(180,base_y*16, value, num_digits); // 假设x坐标为0，你可以根据需要调整
}

void adjust_menu(void)
{
    Read_FLASH();
    pit_ms_init(CCU61_CH0, 115);
    int value_number = 4;
    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 3){
        value_index = 0;
        control_menu();
        }
    }
    else if (place_index == 1 ) {
        if(value_index >= 3){
            value_index = 0;
            image_menu();
        }
    }
    else if (place_index == 2 ) {
        if(value_index >= 3){
            value_index = 0;
            function_menu();

        }
    }
    else if (place_index == 3 ) {
        if(value_index >= 3){
        key_pit_flag = 0;
        ips200_clear();
        value_index = 0;
        place_index = 0;
        allow_value_show = 1;
        allow_image_show = 1;
        Write_FLASH();
        break;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3){
        key_pit_flag = 0;
        ips200_full(RGB565_BLACK);
        value_index = 0;
        place_index = 0;
        allow_value_show = 0;
        allow_image_show = 0;
        Write_FLASH();
        break;
        }
    }

    cursor_selected(value_number+1);
    ips200_show_string(0, 0*Font_size_H, " -=  Menu  =- ");
    ips200_show_string(20, 1*Font_size_H, "Control");
    ips200_show_string(20, 2*Font_size_H, "image");
    ips200_show_string(20, 3*Font_size_H, "Function");
    ips200_show_string(20, 4*Font_size_H, "observer");
    ips200_show_string(20, (value_number+1)*Font_size_H, "go");
    ips200_show_string(20,121+16*9,"IP:");
    ips200_show_string(40,121+16*9,wifi_spi_ip_addr_port);
    }
}


void control_menu(void)
{
    int value_number = 8;
    ips200_clear();

    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        SpeedLoop.TempKP+=1;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        SpeedLoop.TempKP-=1;
        }
    }
    else if (place_index == 1 ) {
        if(value_index >= 1){
            value_index = 0;
            SpeedLoop.TempKI+=1;
        }
        if(value_index <= -1){
            value_index = 0;
            SpeedLoop.TempKI-=1;
        }
    }
    else if (place_index == 2 ) {
        if(value_index >= 1){
            value_index = 0;
            DirOutter.KP+=1;
        }
        if(value_index <= -1){
            DirOutter.KP-=1;
            value_index = 0;
        }
    }
    else if (place_index == 3 ) {
        if(value_index >= 1){
        DirOutter.KP_EX+=1;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        DirOutter.KP_EX-=1;
        value_index = 0;
        }
    }
    else if (place_index == 4 ) {
        if(value_index >= 1){
        DirOutter.KD+=1;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        DirOutter.KD-=1;
        }
    }
    else if (place_index == 5 ) {
        if(value_index >= 1){
        normal_speed+=10;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        normal_speed-=10;
        }
    }
    else if (place_index == 6 ) {
        if(value_index >= 1){
            DirInner.KP+=1;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        DirInner.KP-=1;
        }
    }
    else if (place_index == 7 ) {
        if(value_index >= 1){
        DirInner.KD+=1;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        DirInner.KD-=1;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3)
        {
            ips200_clear();
            value_index = 0;
            place_index = 0;
            break;
        }
    }
    cursor_selected(value_number+1);
    ips200_show_string(0, 0*Font_size_H, " -=  Control  =- ");
    show_string_value(1,SpeedLoop.TempKP,3,"SpeedLoop.KP");
    show_string_value(2,SpeedLoop.TempKI,3,"SpeedLoop.KI");
    show_string_value(3,DirOutter.KP,3,"DirOutter.KP");
    show_string_value(4,DirOutter.KP_EX,3,"DirOutter.KP_EX");
    show_string_value(5,DirOutter.KD,3,"DirOutter.KD");
    show_string_value(6,normal_speed,3,"normal_speed");
    show_string_value(7,DirInner.KP,3,"DirInner.KP");
    show_string_value(8,DirInner.KD,3,"DirInner.KD");
    ips200_show_string(20,(value_number+1)*Font_size_H, "EXIT");
    }


}
void image_menu(void)
{
    int value_number = 2;
    ips200_clear();
    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        }
    }
    else if (place_index == 1 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3)
        {
            ips200_clear();
            value_index = 0;
            place_index = 0;
            break;
        }
    }

    cursor_selected(value_number+1);
    ips200_show_string(0, 0*Font_size_H, " -=  Image  =- ");
    show_string_value(1,0,3,"XXXXXXXX");
    show_string_value(2,0,3,"XXXXXXXX");
    ips200_show_string(20,(value_number+1)*Font_size_H, "EXIT");

    }


}



void function_menu(void)
{
    int value_number = 2;
    ips200_clear();
    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        delay_time+=1;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        delay_time-=1;
        }
    }
    else if (place_index == 1 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        run_time+=1;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        run_time-=1;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3)
        {
            ips200_clear();
            value_index = 0;
            place_index = 0;
            break;
        }
    }

    cursor_selected(value_number+1);
    ips200_show_string(0, 0*Font_size_H, " -=  Function  =- ");
    show_string_value(1,delay_time,3,"delay_time");
    show_string_value(2,run_time,3,"run_time");
    ips200_show_string(20,(value_number+1)*Font_size_H, "EXIT");

    }


}

