/*
 * encoder.c
 *
 *  Created on: 2024年8月29日
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

int16 encoder_data_1;
int16 encoder_data_2;
int16 encoder_data_temp;

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      编码器采集初始化
//  参数说明      void
//  返回参数      void
//  使用示例      encoder_init();
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void encoder_init(void)
{
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                      // 初始化编码器模块与引脚 方向解码编码器模式
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                      // 初始化编码器模块与引脚 方向解码编码器模式
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      编码器数据采集
//  参数说明      void
//  返回参数      void
//  使用示例      encoder_data_get();
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void encoder_data_get(void)
{
    encoder_data_1 = -encoder_get_count(ENCODER_1);                             // 获取编码器计数
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数

    encoder_data_2 = encoder_get_count(ENCODER_2);                              // 获取编码器计数
    encoder_clear_count(ENCODER_2);                                             // 清空编码器计数

    encoder_data_temp = (encoder_data_1 + encoder_data_2) / 2 * 1.5;
}
