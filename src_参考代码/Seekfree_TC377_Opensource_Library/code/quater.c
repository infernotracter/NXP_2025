/*
 * quater.c
 *
 *  Created on: 2024��9��2��
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

/**********************ʹ��˵��*************************/
 //Ĭ��ʹ�õ���imu963
 //���ʹ�õ���imu660��IMU_TYPE�궨��ֵ��Ϊ0


/*************************��������ʼ������*************************/
/*
#if IMU_TYPE
    imu963ra_init();
#else
    imu660ra_init();
#endif

//��������������Ư��
gyroOffsetInit();
*/

/**********************1ms�жϵ�������*************************/
//ע������������1ms��ȡһ��
//�ǶȽ���5ms����һ��
// **************************** PIT�жϺ��� ****************************
/*
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH0);

    static int imu_t;

    imu_t++;

    //��ȡ����������
     get_imu_data();

    if(imu_t % 5 == 0){
        imu_t = 0;
        //�����Ǵ���
        imu_task();
    }
}
*/

/******************************��������************************************/
//quater_info_struct quater = {1, 0, 0, 0};                //��Ԫ����ʼ��
double q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // ��ʼλ����̬��Ϊ��0��0��0����Ӧ��Ԫ��Ϊ��1��0��0��0
eulerAngle_info_struct eulerAngle;                            //ŷ����
gyroOffset_info_struct gyroOffset;
accOffset_info_struct accOffset;

double accoffsetx=120,accoffsety=-170,accoffsetz=-18;
double I_ex, I_ey, I_ez;                                  // ������
//double imu_kp= 0.3;   //0.17                                              //���ٶȼƵ��������ʱ�������
//double imu_ki= 0.004;        //0.004                                        //�������������ʵĻ�������

//1ms
//double imu_kp= 1.5;   //0.17                                              //���ٶȼƵ��������ʱ�������
//double imu_ki= 0.001;        //0.004                                        //�������������ʵĻ�������

//1ms
double imu_kp= 1.5;   //0.17                                              //���ٶȼƵ��������ʱ�������
double imu_ki= 0.0005;        //0.004                                        //�������������ʵĻ�������

/******************************��������************************************/

//�����ǽ�����̬������
void imu_task(void)
{
    //get_imu_data();
    acc_get_angle();
    gyro_get_angle();

    //���ݴ���
    imu_data_deal();

    //����Ƕ�
    Update_Angle();


//    Update_Angle(imu963ra_mag_x/3000.0,imu963ra_mag_y/3000.0,imu963ra_mag_z/3000.0);
}

/*
 * @brief ������������Ư
 * ͨ���ɼ�һ���������ֵ�������������ƫ��ֵ��
 * ������� �����Ƕ�ȡ������ - ��Ʈֵ������ȥ�����ƫ������
 */
#if IMU_TYPE
    //������ƫ����
    void gyroOffsetInit(void)
    {
        gyroOffset.Xdata = 0;
        gyroOffset.Ydata = 0;
        gyroOffset.Zdata = 0;

        for (uint16_t i = 0; i < 200; i++)
        {
            imu963ra_get_gyro();
//            gyroOffset.Xdata += imu963ra_gyro_x;
//            gyroOffset.Ydata += imu963ra_gyro_y;
            gyroOffset.Zdata += imu963ra_gyro_z;
            system_delay_ms(5);
        }
        gyroOffset.Xdata /= 200;
        gyroOffset.Ydata /= 200;
        gyroOffset.Zdata /= 200;
    }
    //���ٶȼ�ƫ����
    void accOffsetInit(void)
    {
       accOffset.Xdata = 0;
       accOffset.Ydata = 0;
       accOffset.Zdata = 0;

       for (uint16_t i = 0; i < 200; i++)
       {
           imu963ra_get_acc();
           accOffset.Xdata += imu963ra_acc_x;
           accOffset.Ydata += imu963ra_acc_y;
           accOffset.Zdata += imu963ra_acc_z;
           system_delay_ms(5);
       }

       accOffset.Xdata /= 200;
       accOffset.Ydata /= 200;
       accOffset.Zdata /= 200;
    }
#else
    void gyroOffsetInit(void)
    {
        gyroOffset.Xdata = 0;
        gyroOffset.Ydata = 0;
        gyroOffset.Zdata = 0;

        for (uint16_t i = 0; i < 200; ++i)
        {
            imu660ra_get_gyro();
//            gyroOffset.Xdata += imu660ra_gyro_x;
//            gyroOffset.Ydata += imu660ra_gyro_y;
            gyroOffset.Zdata += imu660ra_gyro_z;

            system_delay_ms(5);
        }

        gyroOffset.Xdata /= 200;
        gyroOffset.Ydata /= 200;
        gyroOffset.Zdata /= 200;
    }
    void accOffsetInit(void)
    {
      accOffset.Xdata = 0;
      accOffset.Ydata = 0;
      accOffset.Zdata = 0;

      for (uint16_t i = 0; i < 200; ++i)
      {
          imu660ra_get_acc();
          accOffset.Xdata += imu660ra_acc_x;
          accOffset.Ydata += imu660ra_acc_y;
          accOffset.Zdata += imu660ra_acc_z;

          system_delay_ms(5);
      }

      accOffset.Xdata /= 200;
      accOffset.Ydata /= 200;
      accOffset.Zdata /= 200;
   }
#endif

//imu���ݴ���
void imu_data_deal(void)
{
    float alpha = 0.2;  //0.35

#if IMU_TYPE
        //һ�׵�ͨ�˲�����λg
       imu.acc.acc[X] =  (imu963ra_acc_x - accOffset.Xdata) / ACC_SPL * alpha  + imu.acc.acc[X] * (1 - alpha);
       imu.acc.acc[Y] =  (imu963ra_acc_y - accOffset.Ydata) / ACC_SPL * alpha  + imu.acc.acc[Y] * (1 - alpha);
       imu.acc.acc[Z] =  (imu963ra_acc_z - accOffset.Zdata) / ACC_SPL * alpha  + imu.acc.acc[Z] * (1 - alpha);

       imu.gyro.gyro[X] = ANGLE_TO_RAD((imu963ra_gyro_x - gyroOffset.Xdata) / GYRO_SPL);
       imu.gyro.gyro[Y] = ANGLE_TO_RAD((imu963ra_gyro_y - gyroOffset.Ydata) / GYRO_SPL);
       imu.gyro.gyro[Z] = ANGLE_TO_RAD((imu963ra_gyro_z - gyroOffset.Zdata) / GYRO_SPL);

#else
       //һ�׵�ͨ�˲�����λg
      imu.acc.acc[X] =  (imu660ra_acc_x - accOffset.Xdata) / ACC_SPL * alpha  + imu.acc.acc[X] * (1 - alpha);
      imu.acc.acc[Y] =  (imu660ra_acc_y - accOffset.Ydata) / ACC_SPL * alpha  + imu.acc.acc[Y] * (1 - alpha);
      imu.acc.acc[Z] =  (imu660ra_acc_z - accOffset.Zdata) / ACC_SPL * alpha  + imu.acc.acc[Z] * (1 - alpha);

      imu.gyro.gyro[X] = ANGLE_TO_RAD((imu660ra_gyro_x - gyroOffset.Xdata) / GYRO_SPL);
      imu.gyro.gyro[Y] = ANGLE_TO_RAD((imu660ra_gyro_y - gyroOffset.Ydata) / GYRO_SPL);
      imu.gyro.gyro[Z] = ANGLE_TO_RAD((imu660ra_gyro_z - gyroOffset.Zdata) / GYRO_SPL);
#endif
}

/**
 * @brief �û����˲��㷨������������̬(�����ü��ٶȼ����������ǵĻ������)
 * ���ٶȼƶ���֮��������Ƚ����У��������ݼ��������̬���ţ������Ƕ������������У��������ݿ��ţ�
 * ������ʹ�û����������(�ڲ������㷨�Ŵ�̬���)��
 * ���ʹ����̬�����˲����������������ǣ��������ż��ٶȼơ�
 */
double ex, ey, ez;     // ��ǰ���ټƲ�õ��������ٶ��������ϵķ������õ�ǰ��̬��������������������ϵķ��������
void Update_Angle(void)
{
    double halfT = 0.5 * delta_T;    // ��������һ��

    double vx, vy, vz;     // ��ǰ��̬��������������������ϵķ���

    double q0q0 = q0 * q0;//����ˣ������������
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    //double q0q3 = q0 * q3;//δʹ��
    double q1q1 = q1 * q1;
    //double q1q2 = q1 * q2;//δʹ��
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;

    // ������ֹ״̬Ϊ-g ����������
    if(imu.acc.acc[X] * imu.acc.acc[Y] * imu.acc.acc[Z] == 0) // ���ٶȼƴ�����������״̬ʱ(��ʱg = 0)��������̬���㣬��Ϊ�������ĸ���������
        return;

    // �Լ��ٶ����ݽ��й�һ���õ���λ���ٶ�
    //���ٶȼ�<����>���������ٶ�����(��������ϵ)
    double norm = 1/sqrt(imu.acc.acc[X] *imu.acc.acc[X] + imu.acc.acc[Y]*imu.acc.acc[Y] + imu.acc.acc[Z]* imu.acc.acc[Z]);
    imu.acc.acc[X] = imu.acc.acc[X] * norm;
    imu.acc.acc[Y] = imu.acc.acc[Y] * norm;
    imu.acc.acc[Z] = imu.acc.acc[Z] * norm;

    //�����ǻ���<����>��������(��������ϵ)
    // ��������ϵ���������������ϵķ���
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //�����ٶȼƻ�õ�����������һ�����ֵ����ȡ����̬���������������˻�ȡ��̬���
    ex = imu.acc.acc[Y] * vz - imu.acc.acc[Z]* vy;
    ey = imu.acc.acc[Z] * vx - imu.acc.acc[X]* vz;
    ez = imu.acc.acc[X] * vy - imu.acc.acc[Y]* vx;

    //�������л���
    I_ex += ex;
    I_ey += ey;
    I_ez += ez;

    //���������PID���������������ǲ�õĽ��ٶ���ӣ��������ٶ�ֵ
    imu.gyro.gyro[X] = imu.gyro.gyro[X] + imu_kp* ex + imu_ki* I_ex;
    imu.gyro.gyro[Y] = imu.gyro.gyro[Y] + imu_kp* ey + imu_ki* I_ey;
    imu.gyro.gyro[Z] = imu.gyro.gyro[Z] + imu_kp* ez + imu_ki* I_ez;

    // һ����������������Ԫ��΢�ַ��̣�����halfTΪ�������ڵ�1/2
    q0 = q0 + (-q1 * imu.gyro.gyro[X] - q2 * imu.gyro.gyro[Y] - q3 * imu.gyro.gyro[Z]) * halfT;
    q1 = q1 + ( q0 * imu.gyro.gyro[X] + q2 * imu.gyro.gyro[Z] - q3 * imu.gyro.gyro[Y]) * halfT;
    q2 = q2 + ( q0 * imu.gyro.gyro[Y] - q1 * imu.gyro.gyro[Z] + q3 * imu.gyro.gyro[X]) * halfT;
    q3 = q3 + ( q0 * imu.gyro.gyro[Z] + q1 * imu.gyro.gyro[Y] - q2 * imu.gyro.gyro[X]) * halfT;

    // ��λ����Ԫ���ڿռ���תʱ�������죬������ת�Ƕȣ������㷨�������Դ�����������任
    norm = 1/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    eulerAngle.pitch  = RAD_TO_ANGLE(asin(2 * q0 * q2 - 2 * q1 * q3));                                        //-90~90
    eulerAngle.roll   = RAD_TO_ANGLE(atan2(2 * q2 * q3 + 2* q0 * q1, q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)); //-180~180
    eulerAngle.yaw    = RAD_TO_ANGLE(atan2(2 * q1 * q2 + 2 * q0 * q3,  q0 * q0 +q1 * q1 - q2 * q2 - q3 * q3));//0~360

}


