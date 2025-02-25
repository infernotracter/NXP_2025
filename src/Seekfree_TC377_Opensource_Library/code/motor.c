/*
 * motor.c
 *
 *  Created on: 2024??8??28??
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

int32 up_flag;
int32 duty_l;            // ????pwm
int32 duty_r;            // ????pwm
int32 duty_dir;
int32 expect_speed;
struct pid angle;        // ???????
struct pid gyro;         // ?????????
struct pid speed;        // ????
struct pid dir_camera;
struct pid dir_gyro;

//-------------------------------------------------------------------------------------------------------------------
//  ???????      ????????
//  ???????      void
//  ???????      void
//  ??????      motor_init();
//  ??????
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO ????????? ????????????
    pwm_init(PWM_L, 17000, 0);                                                 // PWM ??????????? 17KHz ??????? 0

    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO ????????? ????????????
    pwm_init(PWM_R, 17000, 0);                                                 // PWM ??????????? 17KHz ??????? 0
}

//-------------------------------------------------------------------------------------------------------------------
//  ???????      ???????
//  ???????      void
//  ???????      void
//  ??????      motor_drive();
//  ??????
//-------------------------------------------------------------------------------------------------------------------
void motor_drive(void)
{
    if (duty_l >= 0)
    {
        gpio_set_level(DIR_L, GPIO_HIGH);                                  // DIR???????
        pwm_set_duty(PWM_L, duty_l);                                       // ????????
    }
    else
    {
        gpio_set_level(DIR_L, GPIO_LOW);                                   // DIR???????
        pwm_set_duty(PWM_L, -duty_l);                                      // ????????
    }

    if (duty_r >= 0)
    {
        gpio_set_level(DIR_R, GPIO_HIGH);                                  // DIR???????
        pwm_set_duty(PWM_R, duty_r);                                       // ????????
    }
    else
    {
        gpio_set_level(DIR_R, GPIO_LOW);                                   // DIR???????
        pwm_set_duty(PWM_R, -duty_r);                                      // ????????
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  ???????      ???????
//  ???????      expect    ????????????????????
//  ???????      input     ??????
//  ???????      angle.out ?????????????????????????
//  ??????
//  ??????
//-------------------------------------------------------------------------------------------------------------------
int32 angle_ring(int32 expect, int32 input)
{
    angle.target = expect;
    angle.actual = input;

    angle.error = angle.target - angle.actual;
    angle.out = (int32)(angle.kp * angle.error + angle.kd * (angle.error - angle.last_error));

    angle.last_error = angle.error;

//  ???????

    return angle.out;
}

//-------------------------------------------------------------------------------------------------------------------
//  ???????      ?????????
//  ???????      expect    ??????????????????????
//  ???????      input     ???????
//  ???????      gyro.out  ????????????????????????
//  ??????
//  ??????
//-------------------------------------------------------------------------------------------------------------------
int32 gyro_ring(int32 expect, int32 input)
{
    gyro.target = expect;
    gyro.actual = input;

    gyro.error = gyro.actual - gyro.target;
    gyro.increment += gyro.error * gyro.ki;

//  ??????????
    gyro.increment = my_limit(gyro.increment, -3000, 3000);

    gyro.out = (int32)(gyro.kp * gyro.error + gyro.increment);

//  ???????

    gyro.last_error = gyro.error;

    return gyro.out;
}

//-------------------------------------------------------------------------------------------------------------------
//  ???????      ????
//  ???????      expect    ???????
//  ???????      input     ??????(?????????????)
//  ???????      speed.out ???????????????????????
//  ??????
//  ??????
//-------------------------------------------------------------------------------------------------------------------
int32 speed_ring(int32 expect, int32 input)
{
    speed.target = expect;
    speed.actual = input;

    speed.error = speed.actual - speed.target;
    speed.increment += speed.error * speed.ki;

    speed.increment = my_limit(speed.increment, -2000, 2000);

    speed.out = (int32)(speed.kp * speed.error + speed.increment);

    speed.last_error = speed.error;

    return speed.out;
}

//-------------------------------------------------------------------------------------------------------------------
//  ???????
//  ???????
//  ???????
//  ???????
//  ??????
//  ??????
//-------------------------------------------------------------------------------------------------------------------
int32 dir_in_ring(int32 except, int32 input)
{
    dir_gyro.target = except;
    dir_gyro.actual = input;

    dir_gyro.error = dir_gyro.target - dir_gyro.actual;
    dir_gyro.increment += dir_gyro.error * dir_gyro.ki;

    dir_gyro.increment = my_limit(dir_gyro.increment, -1500, 1500);

    dir_gyro.out = (int32)(dir_gyro.kp * dir_gyro.error + dir_gyro.increment);

    dir_gyro.last_error = dir_gyro.error;

    return dir_gyro.out;
}

//-------------------------------------------------------------------------------------------------------------------
//  ???????
//  ???????
//  ???????
//  ???????
//  ??????
//  ??????
//-------------------------------------------------------------------------------------------------------------------
int32 dir_out_ring(int32 except, int32 input)
{
    dir_camera.target = except;
    dir_camera.actual = input;

    dir_camera.error = dir_camera.actual - dir_camera.target;
    dir_camera.out = (int32)(dir_camera.kp * dir_camera.error + dir_camera.kd * (dir_camera.error - dir_camera.last_error));

    dir_camera.last_error = dir_camera.error;

    return dir_camera.out;
}

//-------------------------------------------------------------------------------------------------------------------
//  ???????
//  ???????
//  ???????
//  ???????
//  ??????
//  ??????
//-------------------------------------------------------------------------------------------------------------------
void dir_control(float turn_error, float gyro)
{
    static int32 flag_dir = 0;

    static int32 duty_camera = 0;
    static int32 duty_gyro   = 0;

    flag_dir++;

    if (flag_dir >= 2)
    {
        turn_error = get_turn_error();
        duty_camera = dir_out_ring(0, turn_error);
        flag_dir = 0;
    }

    duty_gyro = dir_in_ring(duty_camera, gyro);
    duty_dir  = duty_gyro;
}

//-------------------------------------------------------------------------------------------------------------------
//  ???????      ???????
//  ???????      angle     ?????
//  ???????      gyro      ?????
//  ???????      void
//  ??????
//  ??????
//-------------------------------------------------------------------------------------------------------------------
void motor_control(float angle, float gyro)
{
    static int32 flag_speed = 0;
    static int32 flag_angle = 0;

    static int32 duty_speed = 0;
    static int32 duty_angle = 0;
    static int32 duty_gyro  = 0;

    flag_speed++;
    flag_angle++;

    encoder_data_get();

    if (flag_speed == 50)
    {
        duty_speed = speed_ring(expect_speed, encoder_data_temp);
        flag_speed = 0;
        test = !test;
    }

    if (flag_angle == 5)
    {
        duty_angle = angle_ring(duty_speed + BALANCE, (int32)angle);
        flag_angle = 0;
    }

    duty_gyro = gyro_ring(duty_angle, (int32)gyro);

    if (duty_gyro >= 0)
    {
        duty_gyro += 100;
    }
    else
    {
        duty_gyro -= 100;
    }

    duty_l = duty_gyro - duty_dir;
    duty_r = duty_gyro + duty_dir;

    duty_l = my_limit(duty_l, -3000, 3000);
    duty_r = my_limit(duty_r, -3000, 3000);

    motor_drive();
}
