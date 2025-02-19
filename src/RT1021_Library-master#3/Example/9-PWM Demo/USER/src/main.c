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


int main(void)
{
    DisableGlobalIRQ();
    board_init();   //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���
    
    //��ʼ��PWM1 MODULE0 ��ͨ��B ����ΪB23 Ƶ��50hz ռ�ձ�Ϊ �ٷ�֮100*5000/PWM_DUTY_MAX   PWM_DUTY_MAX��fsl_pwm.h�ļ��� Ĭ��Ϊ50000
    //ÿһ��ͨ��ֻ����һ�����ų���PWM
    pwm_init(PWM1_MODULE0_CHB_B23, 50, 5000);

    pwm_init(PWM1_MODULE0_CHA_B22, 50, 5000);
    pwm_init(PWM1_MODULE1_CHB_B25, 50, 5000);
    pwm_init(PWM1_MODULE1_CHA_B24, 50, 5000);
    

    EnableGlobalIRQ(0);
    
    
    
    while(1)
    {
        //����ռ�ձ�Ϊ  �ٷ�֮100*2000/PWM_DUTY_MAX  PWM_DUTY_MAX��fsl_pwm.h�ļ��� Ĭ��Ϊ50000
        pwm_duty(PWM1_MODULE0_CHA_B22,2000);
        
        systick_delay_ms(100);
        
    }

    
}
