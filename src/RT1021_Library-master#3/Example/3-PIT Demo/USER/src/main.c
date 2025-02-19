/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		IAR 8.3 or MDK 5.26
 * @Target core		NXP RT1021DAG5A
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-02-18
 ********************************************************************************************************************/


//整套推荐IO查看Projecct文件夹下的TXT文本



//打开新的工程或者工程移动了位置务必执行以下操作
//第一步 关闭上面所有打开的文件
//第二步 project  clean  等待下方进度条走完



#include "headfile.h"


//实验现象说明：
//板载LED会闪烁起来 ，每一个闪烁的频率不一样
int main(void)
{
    DisableGlobalIRQ();
    board_init();   //务必保留，本函数用于初始化MPU 时钟 调试串口
    
    
    
    gpio_init(C14,GPO,0,GPIO_PIN_CONFIG);
    gpio_init(C15,GPO,0,GPIO_PIN_CONFIG);
    gpio_init(D5 ,GPO,0,GPIO_PIN_CONFIG);
    gpio_init(D7 ,GPO,0,GPIO_PIN_CONFIG);
    
    pit_init();                     //初始化pit外设
    pit_interrupt_ms(PIT_CH0,100);  //初始化pit通道0 周期
    pit_interrupt_ms(PIT_CH1,300);  //初始化pit通道1 周期
    pit_interrupt_ms(PIT_CH2,600);  //初始化pit通道2 周期
    pit_interrupt_ms(PIT_CH3,900);  //初始化pit通道3 周期
    

    EnableGlobalIRQ(0);
    
    //pit的中断函数在isr.c文件
    
    while(1)
    {
        
    }

    
}
