/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		��ɿƼ�����ת����ģ��
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		IAR 8.3 or MDK 5.26
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-03-27
 * @note		    
 ********************************************************************************************************************/

#ifndef _SEEKFREE_WIRELESS
#define _SEEKFREE_WIRELESS


#include "common.h"


#define WIRELESS_UART        USART_7         //����ת����ģ�� ��ʹ�õ��Ĵ���     
#define WIRELESS_UART_TX     UART7_TX_D17
#define WIRELESS_UART_RX     UART7_RX_D18
#define WIRELESS_UART_BAUD   115200
// ------------------------------------ �Զ������� ------------------------------------
// ע������1������ת����ģ��汾��V2.0���µ����޷������Զ������ʵġ�
// ע������2�������Զ��������������RTS���ţ�����Ὺ��ʧ�ܡ�
// ע������3��ģ���Զ�������ʧ�ܵĻ������Գ��Զϵ�����

// �����Զ�����������Ķ��������� ע������
// �����Զ�����������Ķ��������� ע������
// �����Զ�����������Ķ��������� ע������

// 0���ر��Զ�������  
// 1�������Զ������� �Զ������ʵ��������޸�WIRELESS_UART_BAUD֮����Ҫ��ģ��������ã�ģ����Զ�����Ϊ��Ӧ�Ĳ�����

#define WIRELESS_AUTO_UART_BAUD	1
// ------------------------------------ �Զ������� ------------------------------------

#define RTS_PIN D16 //��������λ����  ָʾ��ǰģ���Ƿ���Խ�������  0���Լ�������  1�����Լ�������
#define WIRELESS_BUFFER_SIZE 16
extern uint8 wireless_send_buffer[WIRELESS_BUFFER_SIZE];
extern uint32 wireless_rx_index;

void    seekfree_wireless_init(void);
uint32  seekfree_wireless_send_buff(uint8 *buff, uint32 len);

#endif 
