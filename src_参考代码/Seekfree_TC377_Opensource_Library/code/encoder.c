/*
 * encoder.c
 *
 *  Created on: 2024��8��29��
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

int16 encoder_data_1;
int16 encoder_data_2;
int16 encoder_data_temp;

//-------------------------------------------------------------------------------------------------------------------
//  �������      �������ɼ���ʼ��
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      encoder_init();
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void encoder_init(void)
{
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                      // ��ʼ��������ģ�������� ������������ģʽ
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                      // ��ʼ��������ģ�������� ������������ģʽ
}

//-------------------------------------------------------------------------------------------------------------------
//  �������      ���������ݲɼ�
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      encoder_data_get();
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void encoder_data_get(void)
{
    encoder_data_1 = -encoder_get_count(ENCODER_1);                             // ��ȡ����������
    encoder_clear_count(ENCODER_1);                                             // ��ձ���������

    encoder_data_2 = encoder_get_count(ENCODER_2);                              // ��ȡ����������
    encoder_clear_count(ENCODER_2);                                             // ��ձ���������

    encoder_data_temp = (encoder_data_1 + encoder_data_2) / 2 * 1.5;
}
