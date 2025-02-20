#include "headfile.h"
///////////////////////////////////////////////////////////////////////////////////////
//���ļ���ģ�黯С����

void integralAngle()           //���ֵó�ת��Ƕ�
{
    if(Flag_turn == true)
    {
        gyro_z = mpu_gyro_z / 16.4;
        angle_z += gyro_z * dtt;
    }
}

/*************************************************************************
* ����˵��: ת��һ���Ƕ�
* ����˵��:angleTurn  : ת��Ƕ�    �������
*          dir        ��ת����   1��ת -1��ת (��ֹ������ת����תbushi)
*          turn_V     :ת���ٶ�,��û�б�Ҫ����ʵ����
* �������أ���
*************************************************************************/
void turnAngle(int angleTurn,int dir,int turn_V)
{   
    _BEE_ON
    Flag_turn = open;
    Flag_dirPID = close;
    angle_z = 0;
    switch (dir)
    {
    case 1:
        while (angle_z < angleTurn)
        {
            dirOut = turn_V;
            run();
        }
        break;
    case -1:
        while (angle_z > -angleTurn)
        {
            dirOut = -turn_V;
            run();
        }
        break;
    default:
        break;
    }
    dirOut = 0;
    Flag_turn = close;
    Flag_dirPID = open;
    _BEE_OFF
}

void turnAngle2(int angleTurn,int dir,int KP)
{   
    _BEE_ON
    Flag_turn = open;
    Flag_dirPID = close;
    angle_z = 0;
    Encoder_accumulate = 0;
    switch (dir)
    {
    case 1:
        while (Encoder_accumulate < 30000)
        {
            dirOut = KP * (angleTurn - angle_z);
            run();
        }
        break;
    case -1:
        while (Encoder_accumulate < 30000)
        {
            dirOut = -KP * (angleTurn + angle_z);
            run();
        }
        break;
    default:
        break;
    }
    dirOut = 0;
    Flag_turn = close;
    Flag_dirPID = open;
    _BEE_OFF
}

/*****************************************�������(H��)*******************************************/
void MotorCtrl(int16_t motor1, int16_t motor2)
{
    if (motor1 > 0)
    {
        pwm_duty_updata(PWM_TIM, PWM_L_CH1, motor1);        // ����ռ�ձ�
        pwm_duty_updata(PWM_TIM, PWM_L_CH2, 0);                             // ͬһʱ�� һ�����ֻ�����һ��PWM ��һͨ�����ֵ͵�ƽ
    }
    else
    {
        pwm_duty_updata(PWM_TIM, PWM_L_CH1, 0);									// ͬһʱ�� һ�����ֻ�����һ��PWM ��һͨ�����ֵ͵�ƽ
		pwm_duty_updata(PWM_TIM, PWM_L_CH2, -motor1);		// ����ռ�ձ�
    }

    if (motor2 > 0)
    {
        pwm_duty_updata(PWM_TIM, PWM_R_CH1, motor2);		// ����ռ�ձ�
		pwm_duty_updata(PWM_TIM, PWM_R_CH2, 0);									// ͬһʱ�� һ�����ֻ�����һ��PWM ��һͨ�����ֵ͵�ƽ
    }
    else
    {
        pwm_duty_updata(PWM_TIM, PWM_R_CH1, 0);									// ͬһʱ�� һ�����ֻ�����һ��PWM ��һͨ�����ֵ͵�ƽ
		pwm_duty_updata(PWM_TIM, PWM_R_CH2, -motor2);		// ����ռ�ձ�
    }
}

/*****************************************�ȴ�����*******************************************/
void rushB()
{
    setSpeed = gear[0];
    while (Encoder_accumulate < 980)        //�ܵ������ʼrushB
    {
        speed_Left = tim_encoder_get_count(TIM_3);              //���� ע����λ�ã�
	    tim_counter_rst (TIM_3);
	    speed_Right = -tim_encoder_get_count(TIM_4);            //�ҵ�� 
	    tim_counter_rst (TIM_4);
        Encoder_accumulate = Encoder_accumulate + speed_Left + speed_Right;
        systick_delay_ms(50);
    }
    uart_putstr(UART_4,"x");                //X��ʾͣ����O��ʾ����(Сд��ĸ)
    uart_putstr(UART_4,"x");
    uart_putstr(UART_4,"x");
    setSpeed = 20;
}

/*****************************************�����ֿ���*******************************************/
void rushA()
{
    uart_putstr(UART_4,"o"); //X��ʾͣ����O��ʾ����
    uart_putstr(UART_4,"o");
    uart_putstr(UART_4,"o");
    oled_p6x8str(0, 0, "STOP!!!!");    
    while (1)
    {
        MotorCtrl(0, 0);
    }
}

/*****************************************����ǰ2m*******************************************/
void first2M()
{
    _BEE_ON
    Encoder_accumulate = 0;
    setSpeed = 150;
    while (Encoder_accumulate < 9600*15)
    {
        run();
    }
    setSpeed = gear[2];
    Flag_branch = false;
    Flag_circle = 0;
    _BEE_OFF
}

/*************************************************************************
 * ����˵��: ����ȷ������
 * ����˵��: ��
 * �������أ���
 * �ж���,Ҫд�ö�,����д��
*************************************************************************/
void keyInput(void)
{
    //д�˵�̫�鷳,����д��
}

