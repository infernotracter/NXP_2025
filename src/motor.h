/*
 * motor.h
 *
 *  Created on: 2024年8月28日
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#define DIR_L              (P21_2)
#define PWM_L              (ATOM0_CH1_P21_3)
#define DIR_R              (P21_4)
#define PWM_R              (ATOM0_CH3_P21_5)
#define BALANCE            51

struct pid
{
    float kp;                       // pid参数
    float ki;
    float kd;

    int32 actual;                   // 实际值(测量值)
    int32 target;                   // 目标值

    int32 error;                    // 当前误差
    int32 last_error;               // 上次误差
    int32 before_last_error;        // 上上次误差

    int32 increment;                // 增量(用于积分)
    int32 out;                      // 输出量
};

extern int32 up_flag;
extern int32 duty_l;
extern int32 duty_r;
extern int32 expect_speed;
extern struct pid angle;
extern struct pid gyro;
extern struct pid speed;
extern struct pid dir_camera;
extern struct pid dir_gyro;

void motor_init   (void);
void motor_drive  (void);
void dir_control  (float turn_error, float gyro);
void motor_control(float angle, float gyro);

#endif /* CODE_MOTOR_H_ */
