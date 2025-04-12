#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__

extern char txt[32];
//��������������������������������ȫ�ֱ�����������������������������������
#define _BEE_ON             GPIO_SetBits(GPIOD,GPIO_Pin_12);
#define _BEE_OFF            GPIO_ResetBits(GPIOD,GPIO_Pin_12);
#define open                1
#define close               0
#define LED1				B13
#define LED2				H2
#define KEY					D0
#define BUZZER_PIN			D12	
#define PWM_CH1				TIM_2_CH1_A00
#define PWM_CH2				TIM_2_CH2_A01
#define PWM_CH3				TIM_2_CH3_A02
#define PWM_CH4				TIM_2_CH4_A03
#define PWM_TIM				TIM_5
#define PWM_L_CH1			TIM_5_CH1_A00
#define PWM_L_CH2			TIM_5_CH2_A01
#define PWM_R_CH1			TIM_5_CH3_A02
#define PWM_R_CH2			TIM_5_CH4_A03
//���������ֵ
extern int Threshold_derailment;//������Ϊ����ֵ
extern int Threshold_branch;//������Ϊ����ֵ

//���������־λ
extern int8_t Flag_derailment;
extern int8_t Flag_branch;           //�ж��Ƿ�������·
extern int8_t Flag_branch_number;    //��������·����
extern int8_t Flag_circle;        //�ж��Ƿ��ڻ���
extern int8_t Flag_circle_number;
extern int8_t Flag_dirPID;
extern int8_t Flag_turn;

//�ٶȻ����
extern int  setSpeed;               //�趨�ٶ�
extern int gear[4]; //��λ
extern int  actualSpeed;            //�ٶȻ�PI�������
extern int  speedControlOutNew;
extern int  speedControlOutOld;
extern float  speedIntegral;
extern int  speedControlOut;
extern int  speedControlPeriod;
extern int  speed_Left,speed_Right;  //���������岶׽����,��ЧΪ�ٶ�
extern int  err_speed;

//�������
extern uint16 Vol_left1_actual;        //��Ȩ����������е�вɼ�ֵ
extern uint16 Vol_right1_actual;
extern uint16 Vol_left2_actual;        //��Ȩ����������е�вɼ�ֵ
extern uint16 Vol_right2_actual;
extern uint16 Vol_mid_actual;
extern double BiasIndActual;
extern double BiasIndActual_last,dvar;      //����PD�������
extern int dirControlOut; //����ռ�ձ�
extern int dirOut;
extern float w;

//
extern float  acc_x,acc_z;                  //���ٶȴ������򵥻��������
extern float  gyro_y,gyro_z,angle_z;;                       //�����Ǽ򵥻��������
extern float  angle_Filtering;
extern float  angle_acc;                 //�������ٶȷ����Ǻ�������ó����
extern float  angle_gyro;                //���ٶȻ��ּ���ó����
extern float  err_angle;                 //���սǶ�
extern float  anglespeed;                //���ٶ�
extern float  dtt;              //MPU6050ԭʼ���ݲ���ʱ��
extern float  K1;               //�Լ��ٶȼ�ȡֵ��Ȩ��
extern short  mpu_acc_x,mpu_acc_y,mpu_acc_z;
extern short  mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern float  angleControlOut;

extern int duty_Left;

extern int Encoder_accumulate;
extern long long Encoder_total;
extern int time_accumulate;
extern char signal;
extern long long lap_length;
/****************��������***************/
void ComplementaryFiltering();
void ComplementaryFiltering2();
void SpeedControl();
void OutputSpeedControl();
void directionControl();
void run();
void DetermineDirection();
void Branch();
void leftCircle();




#endif 
