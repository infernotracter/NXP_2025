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

uint8 example_rx_buffer;
lpuart_transfer_t   example_receivexfer;
lpuart_handle_t     example_g_lpuartHandle;


uint8 uart_data_out;

void example_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    if(kStatus_LPUART_RxIdle == status)
    {
        //�����Ѿ���д�뵽��֮ǰ���õ�BUFF��
        //������ʹ�õ�BUFFΪ example_rx_buffer
        uart_data_out = example_rx_buffer;//������ȡ��
    }
    
    handle->rxDataSize = example_receivexfer.dataSize;  //��ԭ����������
    handle->rxData = example_receivexfer.data;          //��ԭ��������ַ
}


//ʵ������˵������RX TX�̽�
//ͨ�����ߵ��Կ��Կ��� uart_data�����ڳ�����һ����

uint8 uart_send;
int main(void)
{
    DisableGlobalIRQ();
    board_init();   //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���
    
    
    //��ʼ������   ������Ϊ115200 TXΪB24 RXΪB25
    uart_init (USART_2, 115200,UART2_TX_B24,UART2_RX_B25);	
    uart_rx_irq(USART_2,1);
    
    //���ô��ڽ��յĻ�����������������
    example_receivexfer.dataSize = 1;
    example_receivexfer.data = &example_rx_buffer;
    
    //�����жϺ����������
    uart_set_handle(USART_2, &example_g_lpuartHandle, example_uart_callback, NULL, 0, example_receivexfer.data, 1);
    
    EnableGlobalIRQ(0);
    
    
    
    while(1)
    {
        uart_send++;
        
        //�����ֽڷ���
        uart_putchar(USART_2,uart_send);
        //���๦�ܺ��� ���в���zf_uart�ļ�
        systick_delay_ms(100);
        
    }

    
}
