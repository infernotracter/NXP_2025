/*
 * flash.c
 *
 *  Created on: 2024��9��1��
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
//  �������
//  ����˵��
//  ���ز���
//  ʹ��ʾ��
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void eeprom_writ(void)
{
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))
            flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);

    flash_union_buffer[0].float_type = speed.kp;
    flash_union_buffer[1].float_type = speed.ki;
    flash_union_buffer[2].float_type = angle.kp;
    flash_union_buffer[3].float_type = angle.kd;
    flash_union_buffer[4].float_type = gyro.kp ;
    flash_union_buffer[5].float_type = gyro.ki ;
    flash_union_buffer[6].float_type = dir_camera.kp;
    flash_union_buffer[7].float_type = dir_camera.kd;
    flash_union_buffer[8].float_type = dir_gyro.kp ;
    flash_union_buffer[9].float_type = dir_gyro.ki ;
    flash_union_buffer[10].float_type = value ;
    flash_union_buffer[11].int32_type = expect_speed;

    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
}

//-------------------------------------------------------------------------------------------------------------------
//  �������
//  ����˵��
//  ���ز���
//  ʹ��ʾ��
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void eeprom_read(void)
{
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);

    speed.kp = flash_union_buffer[0].float_type;
    speed.ki = flash_union_buffer[1].float_type;
    angle.kp = flash_union_buffer[2].float_type;
    angle.kd = flash_union_buffer[3].float_type;
    gyro.kp  = flash_union_buffer[4].float_type;
    gyro.ki  = flash_union_buffer[5].float_type;
    dir_camera.kp = flash_union_buffer[6].float_type;
    dir_camera.kd = flash_union_buffer[7].float_type;
    dir_gyro.kp  = flash_union_buffer[8].float_type;
    dir_gyro.ki  = flash_union_buffer[9].float_type;
    value    = flash_union_buffer[10].float_type;
    expect_speed = flash_union_buffer[11].int32_type;
}
