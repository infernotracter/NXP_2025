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


uint8 gpio_status;
int main(void)
{
    DisableGlobalIRQ();
    board_init();   //务必保留，本函数用于初始化MPU 时钟 调试串口
    
    
    //初始化GPIO C14 为输出 默认输出低电平 使用默认引脚配置GPIO_PIN_CONFIG
    gpio_init(C14,GPO,0,GPIO_PIN_CONFIG);
    
    //初始化GPIO C15 为输出 默认输出高电平 使用默认引脚配置GPIO_PIN_CONFIG
    gpio_init(C15,GPO,1,GPIO_PIN_CONFIG);
    
    //初始化GPIO D5 D7 为输入引脚
    gpio_init(D5 ,GPI,0,GPIO_PIN_CONFIG);
    gpio_init(D7 ,GPI,0,GPIO_PIN_CONFIG);
    
    EnableGlobalIRQ(0);
    
    
    
    while(1)
    {
        gpio_set(C14,1);//设置引脚电平为高电平
        systick_delay_ms(100);
        gpio_set(C14,0);//设置引脚电平为低电平
        systick_delay_ms(100);
        gpio_toggle(C14);//翻转引脚电平
        systick_delay_ms(100);
        
        
        
        gpio_status = gpio_get(D5);//获取引脚电平
        systick_delay_ms(100);
        
    }

    
}
