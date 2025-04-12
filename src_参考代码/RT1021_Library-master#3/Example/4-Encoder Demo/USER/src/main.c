/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		IAR 8.3 or MDK 5.26
 * @Target core		NXP RT1021DAG5A
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-02-18
 ********************************************************************************************************************/


//�����Ƽ�IO�鿴Projecct�ļ����µ�TXT�ı�



//���µĹ��̻��߹����ƶ���λ�����ִ�����²���
//��һ�� �ر��������д򿪵��ļ�
//�ڶ��� project  clean  �ȴ��·�����������



#include "headfile.h"


int16 encoder1;
int16 encoder2;
int16 encoder3;
int16 encoder4;

int main(void)
{
    DisableGlobalIRQ();
    board_init();   //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���
    
    
    pwm_init(PWM2_MODULE0_CHB_D7, 50, 5000);
    pwm_init(PWM2_MODULE0_CHA_D6, 50, 5000);
    pwm_init(PWM2_MODULE1_CHB_D5, 50, 5000);
    pwm_init(PWM2_MODULE1_CHA_D4, 50, 5000);
    
    
    
    
    //��ʼ�� QTIMER_1 A��ʹ��QTIMER1_TIMER0_D0 B��ʹ��QTIMER1_TIMER1_D1
    qtimer_quad_init(QTIMER_1,QTIMER1_TIMER0_D0,QTIMER1_TIMER1_D1);
    
    //��ʼ�� QTIMER_1 A��ʹ��QTIMER1_TIMER2_D2 B��ʹ��QTIMER1_TIMER3_D3
    qtimer_quad_init(QTIMER_1,QTIMER1_TIMER2_D2,QTIMER1_TIMER3_D3);
    
    //һ��QTIMER���� ����������������
    
    
    qtimer_quad_init(QTIMER_2,QTIMER2_TIMER0_C0,QTIMER2_TIMER1_C1);
    qtimer_quad_init(QTIMER_2,QTIMER2_TIMER2_C2,QTIMER2_TIMER3_C3);
    
    
    //��B22��D0ʹ�öŰ�����������
    //��B23��D2ʹ�öŰ�����������
    //��B24��B4ʹ�öŰ�����������
    //��B25��B6ʹ�öŰ�����������
    
    
    //��D1 D3 B5 B7�ӵأ����Կ����ɼ���������Ϊ5.
    
    //���ֱ�����ӱ�������A B�࣬��ô�����ֱ�Ӳɼ�����������
    
    EnableGlobalIRQ(0);
    
    
    
    while(1)
    {
        //��ȡ����������ֵ
        encoder1 = qtimer_quad_get(QTIMER_1,QTIMER1_TIMER0_D0);
        encoder2 = qtimer_quad_get(QTIMER_1,QTIMER1_TIMER2_D2);
        encoder3 = qtimer_quad_get(QTIMER_2,QTIMER2_TIMER0_C0);
        encoder4 = qtimer_quad_get(QTIMER_2,QTIMER2_TIMER2_C2);
        
        qtimer_quad_clear(QTIMER_1,QTIMER1_TIMER0_D0);
        qtimer_quad_clear(QTIMER_1,QTIMER1_TIMER2_D2);
        qtimer_quad_clear(QTIMER_2,QTIMER2_TIMER0_C0);
        qtimer_quad_clear(QTIMER_2,QTIMER2_TIMER2_C2);
        
        systick_delay_ms(100);
        
    }

    
}
