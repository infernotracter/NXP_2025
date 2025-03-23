/********************************************************************************
 * ��л:��ϸ�л�����,��ϣ��,������,�߼���,���,�׳���ѧ��ѧ��������ṩ��֧�������.
 * 		��лһ��������Уͬѧ,�������й���ҵ��ѧ�캣ѧԺ��ͬѧ,�ڽ�����ѧϰ���˺ܶ�.
 * 		��л����ĳ���ǰ��,���ʱ���ǿ��洫�������ܿ��ִ�Ϻ��.
 * 
 * ˵��:����ˮƽ����,�������Ϲ�ʱ�ڶ��PID���ֱ����ӵķ���.
 * 		����ı��������Ƚϳ�,��Ҳ�������,ע�ͽ�ȫ,ϣ���ܰﵽѧ����.
 * 		���޳�ѧ�߰�ֱ����������������,Ԫ�ش���Ҳ��ͼһ��,�������ܸ��´���PID����.
 * 		��ӭ������������.
 * 
 * ��Ȩ��Ϣ:
 *	�й���ҵ��ѧ ���繤��ѧԺ ����ɽ�����
 *  ������,������,���鳿,����Ң.
 *  (C)Copyright CUTM, 2021. 
 *  ALL RIGHTS RESERVED.
********************************************************************************/

#include "headfile.h"

int main(void)
{
	//��ʼ����ʼ
	board_init(true);												// ��ʼ�� debug �������
	gpio_init(KEY, GPI, GPIO_HIGH, GPI_PULL_UP);					// ��ʼ������Ϊ�������� Ĭ�ϸߵ�ƽ
	gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);					// ��ʼ������Ϊ������� Ĭ�ϸߵ�ƽ
	gpio_init(LED2, GPO, GPIO_HIGH, GPO_PUSH_PULL);					// ��ʼ������Ϊ������� Ĭ�ϸߵ�ƽ
    gpio_init(BUZZER_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	exti_interrupt_init(A15, EXTI_Trigger_Falling, 0x01);			//�ɻɹ��½��ش���
	oled_init();
    oled_p6x8str(0, 0, "CUMT");
    icm20602_init_spi();                                            //�����ǳ�ʼ��
    tim_encoder_init(TIM_3, TIM_3_ENC1_B04, TIM_3_ENC2_B05);		//��������ʼ��
    tim_encoder_init(TIM_4, TIM_4_ENC1_B06, TIM_4_ENC2_B07);
	adc_init(ADC_1, ADC1_CH10_C00, ADC_12BIT);						//ADC��ʼ��
	adc_init(ADC_1, ADC1_CH11_C01, ADC_12BIT);
	adc_init(ADC_1, ADC1_CH12_C02, ADC_12BIT);
	adc_init(ADC_1, ADC1_CH13_C03, ADC_12BIT);
	adc_init(ADC_1, ADC1_CH04_A04, ADC_12BIT);
    gpio_init(RTS_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);						// ��ʼ����������
	uart_init (WIRELESS_UART, 9600, WIRELESS_UART_TX, WIRELESS_UART_RX);	// ��ʼ������
	uart_rx_irq(WIRELESS_UART, ENABLE);										//���ڽ����ж�
	pwm_init(PWM_TIM, PWM_L_CH1, 16000, 0);		//PWM ͨ��1 ��ʼ��Ƶ�� ռ�ձȳ�ʼΪ0
	pwm_init(PWM_TIM, PWM_L_CH2, 16000, 0);									
	pwm_init(PWM_TIM, PWM_R_CH1, 16000, 0);										
	pwm_init(PWM_TIM, PWM_R_CH2, 16000, 0);							
	tim_interrupt_init_ms(TIM_2, 1000, 2);		//��ʱ��		
	tim_interrupt_init_ms(TIM_8, 1, 0);			//ֱ�����ж�	
	tim_interrupt_init_ms(TIM_6, 10, 1);		//�ٶȻ��ж�	
	tim_interrupt_init_ms(TIM_7, 5, 1);			//�����ж�
    _BEE_OFF		
	//��ʼ������
	systick_delay_ms(1000);
    rushB();									//Time bomb has planted in B side
    first2M();									//ǰ����������,������·������
	while(1)
	{
//        oled_printf_int32(0,2,Vol_left2_actual ,5);
//        oled_printf_int32(20,3,Vol_left1_actual ,5); 
//        oled_printf_int32(50,4,Vol_mid_actual ,5);
//        oled_printf_int32(70,3,Vol_right1_actual ,5);  
//        oled_printf_int32(90,2,Vol_right2_actual ,5); 
//        oled_printf_float(0,7,angle_Filtering,2,3);
//        oled_printf_int32(6,2,speedControlOut, 5); 
//        oled_printf_int32(6,5,dirControlOut, 5);  
//        oled_printf_int32(0,6,actualSpeed, 5);
//        oled_printf_int32(6,7,Encoder_total/93000, 3);
        run();
        Branch();
		fixedCircle();
	}
}
//TIM1 ����ͷ
//TIM3 ������
//TIM4 ������
//TIM5 ���
	
//oled_printf_float(0,0,x,2,3);			��ʾ����������
//oled_printf_int32(0,0,x,5);			��ʾ��������
//uart_putstr(UART_4,"i lvoe you"); 	�����ַ���
//uart_putbuff(UART_4,&a[0],5);			��������
//seekfree_sendimg_03x(WIRELESS_UART, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);

