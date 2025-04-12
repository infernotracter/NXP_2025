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


int16 encoder1;
int16 encoder2;
int16 encoder3;
int16 encoder4;

int main(void)
{
    DisableGlobalIRQ();
    board_init();   //务必保留，本函数用于初始化MPU 时钟 调试串口
    
    
    pwm_init(PWM2_MODULE0_CHB_D7, 50, 5000);
    pwm_init(PWM2_MODULE0_CHA_D6, 50, 5000);
    pwm_init(PWM2_MODULE1_CHB_D5, 50, 5000);
    pwm_init(PWM2_MODULE1_CHA_D4, 50, 5000);
    
    
    
    
    //初始化 QTIMER_1 A相使用QTIMER1_TIMER0_D0 B相使用QTIMER1_TIMER1_D1
    qtimer_quad_init(QTIMER_1,QTIMER1_TIMER0_D0,QTIMER1_TIMER1_D1);
    
    //初始化 QTIMER_1 A相使用QTIMER1_TIMER2_D2 B相使用QTIMER1_TIMER3_D3
    qtimer_quad_init(QTIMER_1,QTIMER1_TIMER2_D2,QTIMER1_TIMER3_D3);
    
    //一个QTIMER可以 创建两个正交解码
    
    
    qtimer_quad_init(QTIMER_2,QTIMER2_TIMER0_C0,QTIMER2_TIMER1_C1);
    qtimer_quad_init(QTIMER_2,QTIMER2_TIMER2_C2,QTIMER2_TIMER3_C3);
    
    
    //将B22与D0使用杜邦线链接起来
    //将B23与D2使用杜邦线链接起来
    //将B24与B4使用杜邦线链接起来
    //将B25与B6使用杜邦线链接起来
    
    
    //将D1 D3 B5 B7接地，可以看到采集到的数据为5.
    
    //如果直接连接编码器的A B相，那么则可以直接采集编码器数据
    
    EnableGlobalIRQ(0);
    
    
    
    while(1)
    {
        //读取编码器计数值
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
