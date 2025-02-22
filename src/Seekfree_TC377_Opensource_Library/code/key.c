/*
 * key.c
 *
 *  Created on: 2024��8��31��
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

int32 key_number;

//-------------------------------------------------------------------------------------------------------------------
//  �������      ��ȡ��������
//  ����˵��
//  ���ز���
//  ʹ��ʾ��
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void get_key_number(void)
{
    key_scanner();

    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        key_number = 1;
    }
    else if (key_get_state(KEY_1) == KEY_LONG_PRESS)
    {
        key_number = 5;
    }

    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        key_number = 2;
    }
    else if (key_get_state(KEY_2) == KEY_LONG_PRESS)
    {
        key_number = 6;
    }


    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        key_number = 3;
    }
    else if (key_get_state(KEY_3) == KEY_LONG_PRESS)
    {
        key_number = 7;
    }


    if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        key_number = 4;
    }
    else if (key_get_state(KEY_4) == KEY_LONG_PRESS)
    {
        key_number = 8;
    }

    if (key_get_state(KEY_1) == KEY_RELEASE && key_get_state(KEY_2) == KEY_RELEASE &&
        key_get_state(KEY_3) == KEY_RELEASE && key_get_state(KEY_4) == KEY_RELEASE)
    {
        key_number = 0;
    }

    key_clear_all_state();
}

