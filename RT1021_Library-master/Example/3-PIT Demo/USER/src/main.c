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


//ʵ������˵����
//����LED����˸���� ��ÿһ����˸��Ƶ�ʲ�һ��
int main(void)
{
    DisableGlobalIRQ();
    board_init();   //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���
    
    
    
    gpio_init(C14,GPO,0,GPIO_PIN_CONFIG);
    gpio_init(C15,GPO,0,GPIO_PIN_CONFIG);
    gpio_init(D5 ,GPO,0,GPIO_PIN_CONFIG);
    gpio_init(D7 ,GPO,0,GPIO_PIN_CONFIG);
    
    pit_init();                     //��ʼ��pit����
    pit_interrupt_ms(PIT_CH0,100);  //��ʼ��pitͨ��0 ����
    pit_interrupt_ms(PIT_CH1,300);  //��ʼ��pitͨ��1 ����
    pit_interrupt_ms(PIT_CH2,600);  //��ʼ��pitͨ��2 ����
    pit_interrupt_ms(PIT_CH3,900);  //��ʼ��pitͨ��3 ����
    

    EnableGlobalIRQ(0);
    
    //pit���жϺ�����isr.c�ļ�
    
    while(1)
    {
        
    }

    
}
