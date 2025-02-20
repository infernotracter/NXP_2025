#include "headfile.h"
//���ϸ���

long long lap_length = 93000*20;        //��������

//PID                                 old       400       500
float P_angle = 700;     //ֱ����KP   400       700       600
float D_angle = 30;      //ֱ����KD   30        30        30
float P_direction = 1; //����KP      3        1         2
float D_direction = 20;  //����KD    10       20        30
float P_speed = 30;       //�ٶȻ�KP   8        30         6
float I_speed = 3;     //�ٶȻ�KI   0.2         3      0.15

//���������ֵ
int Threshold_derailment = 500; //������Ϊ����ֵ
int Threshold_branch = 2000;    //����·����ֵ
int Threshold_circle = 5000;    //circle����ֵ

//���������־λ
int8_t Flag_derailment = 0;     
int8_t Flag_branch = 0;        //�ж��Ƿ�������· 0���� 1��
int8_t Flag_branch_number = 0; //��������·���� ������ֹ�������
int8_t Flag_circle = 0;        //�ж��Ƿ��ڻ���
int8_t Flag_circle_number = 1; //�����ĵ�N�λ�����־λ
int8_t Flag_dirPID = 1;        //1Ϊ���򻷿��ƣ�0Ϊ���η���
int8_t Flag_turn = 0;          //ת���ñ�־λ 1�������� 0���λ���

//�Ƕȴ������
float angleControlOut = 0;
float angle_balance = 61; //ƽ��λ�ýǶ�      ��ֹʱ62.5
float acc_x = 0, acc_z = 0; //���ٶȴ������򵥻��������
float gyro_y = 0,gyro_z = 0,angle_z = 0;           //�����Ǽ򵥻��������
float angle_Filtering = 90; //�����˲��õ��Ƕ�
float angle_acc = 0;        //�������ٶȷ����Ǻ�������ó����
float angle_gyro = 0;       //���ٶȻ��ּ���ó����
float err_angle = 0;        //���սǶ�
float anglespeed = 0;       //���ٶ�
float dtt = 0.001;          //MPU9250ԭʼ���ݲ���ʱ��
float K1 = 0.005;           //�Լ��ٶȼ�ȡֵ��Ȩ��
float K2 = 0.2;             //�Լ��ٶȼ�ȡֵ��Ȩ��
float x1=0,x2=0,y1=0;       //�����м����

//�ٶȻ����
int setSpeed = 600;       //�趨�ٶ�
int gear[4] = {0, 200, 400, 500, 600}; //��λ
int actualSpeed = 0;    //�ٶȻ�PI�������
int speedControlOutNew = 0;
int speedControlOutOld = 0;
float speedIntegral = 0;
int speedControlOut = 0;
int speedControlPeriod = 0;
int speed_Left = 0, speed_Right = 0; //���������岶׽����,��ЧΪ�ٶ�
int err_speed = 0;

//�������
uint16 Vol_left1_actual = 0; //��Ȩ����������е�вɼ�ֵ
uint16 Vol_right1_actual = 0;
uint16 Vol_left2_actual = 0; //��Ȩ����������е�вɼ�ֵ
uint16 Vol_right2_actual = 0;
uint16 Vol_mid_actual = 0;
double BiasIndActual = 0;
double BiasIndActual_last = 0, dvar = 0; //����PD�������
int dirControlOut = 0; //����ռ�ձ�
int dirOut = 0;
float w = 1;

//ռ�ձ����
int duty_death_left_P = 1000;  //����������ռ�ձ�      ��������ã������ô󡣡���
int duty_death_right_P = 1000; //����������ռ�ձ�
int duty_death_left_N = -1000;  //����������ռ�ձ�
int duty_death_right_N = -1000; //����������ռ�ձ�
int duty_Max_P = 6000;         //���PWM�޷�����   10000
int duty_Max_N = -6000;
int duty_Left = 0;     //���ӵõ�������ռ�ձ�
int duty_Right = 0;    //���ӵõ�������ռ�ձ�

//����
int Encoder_accumulate = 0; //�Ӿ����,�ֶ�����10cm = 9200�����壬�����ر�ȷ
long long Encoder_total = 0;
int time_accumulate = 0;
char signal = 0;

/*************************************** �����˲����� ������ֱ����PD��*********************************************/
/*angle_Filtering = K1 * angle_m+ (1-K1) * (angle_Filtering + gyro_m * dt);
angle_Filtering ���ںϺ�ĽǶ�ֵ
angle_acc�Ǽ��ٶȲ����򵥼���õ��ĽǶ�
angle + angle_gyro * dt�������ǻ��ֵõ��ĽǶ�
dtΪ�������ڣ���λ��s��ȡ0.005
K1���˲���ϵ����ȡ0.001��
һ�׻����˲�Ҳ���Կ����Ǽ�Ȩƽ����*/
void ComplementaryFiltering()
{
    if (icm_acc_x >= 0)
        icm_acc_x = -1;                                 //tan90��
    acc_x = - icm_acc_x * 9.8 / 4096.0;                 //ת����λ
    acc_z = icm_acc_z * 9.8 / 4096.0;                   //ת����λ
    gyro_y = icm_gyro_y / 16.4;                         //ת����λ

    anglespeed = - (gyro_y - 0.25);                     //���ƫ��
    angle_acc = atan((acc_z) / acc_x) * 57.3;           //���ٶȵõ��ĽǶȣ��ɼ����ƫ��
    
    angle_gyro = angle_Filtering + anglespeed * dtt;    //���ٶȵõ��ĽǶ�
    angle_Filtering = K1 * angle_acc + (1 - K1) * angle_gyro;
    
    err_angle = angle_Filtering - angle_balance;
    angleControlOut = P_angle * err_angle + D_angle * anglespeed;
}

/*���׻����˲�(ԭ����ɶ?ѧ���˽̽���!)*/
void ComplementaryFiltering2()
{
    if (icm_acc_x >= 0)
        icm_acc_x = -1;                             
    acc_x = - icm_acc_x * 9.8 / 4096.0;             //ת����λ
    acc_z = icm_acc_z * 9.8 / 4096.0;               //ת����λ
    gyro_y = icm_gyro_y / 16.4;                     //ת����λ

    anglespeed = - (gyro_y - 0.25);                 //���ƫ��
    angle_acc = atan((acc_z) / acc_x) * 57.3;       //���ٶȵõ��ĽǶȣ��ɼ����ƫ��
    
    x1 = (angle_acc - angle_Filtering) * (1 - K2) * (1 - K2);
    y1 = y1 + x1 * dtt;
    x2 = y1 + 2 * (1 - K2) * (angle_acc - angle_Filtering) + anglespeed;
    angle_Filtering = angle_Filtering + x2 * dtt;
    
    err_angle = angle_Filtering - angle_balance;
    angleControlOut = P_angle * err_angle + D_angle * anglespeed;
}

/*****************************************�ٶȱջ������㷨��PI��*******************************************/
void SpeedControl()
{
    actualSpeed = (speed_Left + speed_Right) * 0.5;
    err_speed = setSpeed - actualSpeed;                         //�ٶ�ƫ��
    speedIntegral = speedIntegral + err_speed * I_speed;        //����

    if(speedIntegral>3000)  speedIntegral=3000;                 //�޷�
    if(speedIntegral<-2000)  speedIntegral=-2000;               //�޷�

    speedControlOutOld = speedControlOutNew;
    speedControlOutNew = err_speed * P_speed + speedIntegral;
}

void OutputSpeedControl()                           //�ٶȻ�ƽ�����
{
    float fValue;
    fValue = speedControlOutNew - speedControlOutOld;
    speedControlOut = fValue * (speedControlPeriod + 1) / 10.0 + speedControlOutOld;

    if (speedControlOut > 4500)
        speedControlOut = 4500;                     //�޷�
    if (speedControlOut < -4500)
        speedControlOut = -4500;                    //�޷�
}

/*****************************************����ջ������㷨(PD)*******************************************/
void directionControl()
{
    dvar = BiasIndActual - BiasIndActual_last;
    dirControlOut = P_direction * BiasIndActual + D_direction * dvar;
    BiasIndActual_last = BiasIndActual;
}

/*****************************************��ģ���п����㷨*******************************************/
void run()
{
    if (Vol_left1_actual + Vol_right1_actual > Threshold_circle && err_angle < 10 && err_angle > -10)
        dirControlOut *= 0.05;                                  //���λ���
    if (Flag_dirPID == open)
        dirOut = dirControlOut;                                 //���򻷿���
    
    duty_Left = angleControlOut - speedControlOut + dirOut;     //dirOutΪ������ת
    duty_Right = angleControlOut - speedControlOut - dirOut;

    if (duty_Left > 0)
        duty_Left = duty_Left + duty_death_left_P;
    if (duty_Left < 0)
        duty_Left = duty_Left + duty_death_left_N;
    if (duty_Right > 0)
        duty_Right = duty_Right + duty_death_left_P;
    if (duty_Right < 0)
        duty_Right = duty_Right + duty_death_left_N;

    if (duty_Left > duty_Max_P)
        duty_Left = duty_Max_P;
    if (duty_Right > duty_Max_P)
        duty_Right = duty_Max_P;
    if (duty_Left < duty_Max_N)
        duty_Left = duty_Max_N;
    if (duty_Right < duty_Max_N)
        duty_Right = duty_Max_N;

    MotorCtrl(-duty_Left, -duty_Right);                         //ֱ�������ǵ���ʹ����,����ȡ��
}


/*****************************************Ԫ�ر�־λ�ж�*********************************/
void DetermineDirection()
{
    //�ж�����·
    if (Vol_left1_actual + Vol_right1_actual < Threshold_branch 
        && Vol_left2_actual > Vol_left1_actual 
        && Vol_right2_actual > Vol_right1_actual)
    {
        Flag_branch = 1;
    }
    //����
//    else if (Vol_mid_actual > Threshold_circle && err_angle < 10 && err_angle > -10)
//    {
//        if (Vol_left2_actual > Vol_right2_actual + 500) //�󻷵�
//        {
//            Flag_circle = 1;
//        }
//        else if (Vol_right2_actual > Vol_left2_actual + 500) //�һ���
//        {
//            Flag_circle = 2;
//        }      
//        
//    }
    else if (Vol_left1_actual + Vol_right1_actual > Threshold_circle
            && err_angle < 5 && err_angle > -5)
    {
        Flag_circle = 3;
    }
    //�ж��Ƿ����
    else if ((Vol_left1_actual < Threshold_derailment) && (Vol_right1_actual < Threshold_derailment))
    {
        while(1)
            MotorCtrl(0,0);    //���ְɰ��棬̼�˶���ײ����
    }
}

/*****************************************����·*******************************************/
void Branch()
{
    if (Flag_branch == true)
    {
        _BEE_ON

//        turnAngle(20,1,800);              //������·planC        
//        //turnAngle2(30,1,60);
//        P_direction *= 1.5;

        Flag_dirPID = close;
        for (int i = 0;i<2000;i++)         //���·planB,�����ַ��������,����ʹ...
        {
            run();
            dirOut = 3000;
        }
        P_direction *= 8;
        D_direction *= 7;
        w = 1;
        Flag_dirPID = open;

        
//        Encoder_accumulate = 0;           //��ռ���,���·planA
//        while (Encoder_accumulate < 9600) //��5cm
//        {
//            Encoder_accumulate = Encoder_accumulate + Read_Encoder(1)  -Read_Encoder(2);
//            dirOut += 15; //Σ,ת�����ˣ������ȿ���
//            if(dirOut > 1000)
//            {
//                dirOut = 1000;
//            }
//            run();
//        }
        
        setSpeed = gear[1];
        Encoder_accumulate = 0;             //��ռ���
        while (Encoder_accumulate < 150000) //��1m+
        {
            
            run();
        }
        _BEE_OFF
        rushA();
    }
}

///*****************************************�󻷵�*******************************************/
//void leftCircle()
//{
//    if (Flag_circle == 1 && Flag_circle_number <= 4) //��ֹ����
//    {
//        switch (Flag_circle_number % 2)
//        {
//        case 1:
//            _BEE_ON
//            Flag_dirPID = close;   //�ӹ����򻷿���
//            for(int i=0;i<5000;i++)            
//            {
//                dirOut = 800;
//                run();
//            }
//            Flag_dirPID = open;    //���򻷿���
//            Flag_circle = 0;
//            Flag_branch = false;;   //��ֹ��������·
//            Flag_circle_number++;
//            _BEE_OFF
//            break;
//        case 2:
//            Encoder_accumulate = 0;
//            while (Encoder_accumulate < 40000)
//            {
//                dirOut *= 0; //���λ���
//                run();
//            }
//            Flag_circle_number++;
//            Flag_circle = 0;

//        default:
//            break;
//        }
//    }
//}

///*****************************************�һ���*******************************************/
//void rightCircle()
//{
//    if (Flag_circle == 2 && Flag_circle_number <= 4) //��ֹ����
//    {
//        switch (Flag_circle_number % 2)
//        {
//        case 1:
//            _BEE_ON
//            Flag_dirPID = close;   //�ӹ����򻷿���
//            for(int i=0;i<5000;i++)            
//            {
//                dirOut = -800;
//                run();
//            }
//            Flag_dirPID = open;    //���򻷿���
//            Flag_circle = 0;
//            Flag_branch = false;   //��ֹ��������·
//            Flag_circle_number++;
//            _BEE_OFF
//            break;
//        case 2:
//            Encoder_accumulate = 0;
//            while (Encoder_accumulate < 40000)
//            {
//                dirOut *= 0; //���λ���
//                run();
//            }
//            Flag_circle_number++;
//            Flag_circle = 0;
//            Flag_branch = false;

//        default:
//            break;
//        }
//    }
//}

/*****************************************�̶�����*******************************************/
/*��������Ҳûд��,���Ծ����˸��̶������ķ���,��ǰ���뻷��˳��,���ж�����*/
void fixedCircle()
{
    if (Flag_circle == 3 && Flag_circle_number <= 4) //��ֹ����
    {
        switch (Flag_circle_number % 2)
        {
        case 1:
            _BEE_ON
            Flag_dirPID = close;        //�ӹ����򻷿���
            for(int i=0;i<10000;i++)            
            {
                dirOut = -800;
                run();
            }
            Flag_dirPID = open;         //���򻷿���
            Flag_circle = 0;
            Flag_branch = false;;       //��ֹ��������·
            Flag_circle_number++;
            _BEE_OFF
            break;
        case 2:
            Encoder_accumulate = 0;
            while (Encoder_accumulate < 40000)
            {
                dirOut *= 0.1;          //���λ���
                run();
            }
            Flag_circle_number++;
            Flag_circle = 0;
            Flag_branch = false;
        default:
            break;
        }
    }
}


