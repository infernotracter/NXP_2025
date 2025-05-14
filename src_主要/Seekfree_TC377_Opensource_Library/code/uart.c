/*
 * uart.c
 *
 *  Created on: 2024年8月28日
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
//  函数简介
//  参数说明
//  返回参数
//  使用示例
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void send_wave(uart_index_enum uartn)
{
  uint8 frameNameHead[] = "AABBCC";     // 定义通道名帧头帧尾
  uint8 frameNameEnd[]  = "CCBBAA";

  uint8 frameDataHead[] = "DDEEFF";     // 定义数据帧头帧尾
  uint8 frameDataEnd [] = "FFEEDD";

  uint8 name[] = {"angle.out, gyro.out, speed.out, encoder_data_temp, expect_speed"};

  float channels[5];                    // 赋值数据

  channels[0] = (float)angle.out;
  channels[1] = (float)-gyro.out ;
  channels[2] = (float)speed.out;
  channels[3] = (float)encoder_data_temp;
  channels[4] = (float)expect_speed;

  uart_write_buffer(uartn,frameNameHead,sizeof(frameNameHead)-1);
  uart_write_buffer(uartn,name,sizeof(name)-1);
  uart_write_buffer(uartn,frameNameEnd,sizeof(frameNameEnd)-1);

  uart_write_buffer(uartn,frameDataHead,sizeof(frameDataHead)-1);
  uart_write_buffer(uartn,(uint8*)channels,sizeof(channels));
  uart_write_buffer(uartn,frameDataEnd,sizeof(frameDataEnd)-1);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介
//  参数说明
//  返回参数
//  使用示例
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
float parse_float(uint8* str)
{
    float result = 0;
    int sign = 1;
    int i = 0;
    int power = 1;

    // 无关字符
    while ((str[i] < '0' || str[i] > '9') && (str[i] != ' ') && str[i] != '-' && str[i] != '+') {
        i++;
    }

    // 处理空格或符号
    while (str[i] == ' ' || str[i] == '-' || str[i] == '+') {
        if (str[i] == '-')
            sign = -1;
        i++;
    }

    // 提取整数部分
    while (str[i] >= '0' && str[i] <= '9') {
        result = result * 10 + (str[i] - '0');
        i++;
    }

    // 提取小数部分
    if (str[i] == '.') {
        i++;
        while (str[i] >= '0' && str[i] <= '9') {
            result = result * 10 + (str[i] - '0');
            power *= 10;
            i++;
        }
    }

    return result * sign / power;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介
//  参数说明
//  返回参数
//  使用示例
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void uart_adjust_parameter(void)
{
    uint8 data_buffer[32];
    uint8 len = (uint8)wireless_uart_read_buffer(data_buffer, 32); // 查看是否有消息 默认缓冲区是 WIRELESS_UART_BUFFER_SIZE 总共 64 字节
    data_buffer[len] = '\0'; // 添加字符串结束符
    wireless_uart_send_buffer(data_buffer, len);

    if(data_buffer[0] == 'k')
    {
        //调节X轴
        if(data_buffer[1] == 'p')  speed.kp = parse_float(data_buffer);

    }
    if(data_buffer[0] == 'k')
    {
            //调节X轴
        if(data_buffer[1] == 'i')  speed.ki = parse_float(data_buffer);

    }
//    if(data_buffer[0] == 'S')
//    {
//        //调节X轴
//        if(data_buffer[1] == 'p')  setspeed = parse_float(data_buffer);
//
//    }
//    if(data_buffer[0] == 'L')
//    {
//        //调节X轴
//        if(data_buffer[1] == 'd')  pid_motor_l.Kd = parse_float(data_buffer);
//
//    }
//    if(data_buffer[0] == 'R')
//    {
//        //调节X轴
//        if(data_buffer[1] == 'p')  pid_motor_r.Kp = parse_float(data_buffer);
//
//    }
//    if(data_buffer[0] == 'R')
//    {
//        //调节X轴
//        if(data_buffer[1] == 'i')  pid_motor_r.Ki = parse_float(data_buffer);
//
//    }
//
//    if(data_buffer[0] == 'R')
//    {
//        //调节X轴
//        if(data_buffer[1] == 'd')  pid_motor_r.Kd = parse_float(data_buffer);
//
//    }
//
//    if(data_buffer[0] == 'G')
//   {
//       //调节X轴
//       if(data_buffer[2] == 'P')  ZPID.inner.kp = parse_float(data_buffer);
//
//   }
//
//    if(data_buffer[0] == 'A')
//      {
//          //调节X轴
//          if(data_buffer[2] == 'P')  ZPID.outer.kp = parse_float(data_buffer);
//
//      }
}
