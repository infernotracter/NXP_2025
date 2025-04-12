/*
 * quater.h
 *
 *  Created on: 2024��9��2��
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

#ifndef CODE_QUATER_H_
#define CODE_QUATER_H_

//===================================================�궨��===================================================
#define delta_T  0.005f
#define IMU_TYPE 0
#if IMU_TYPE
    #define GYRO_SPL 14.3
    #define ACC_SPL  4098.0
#else
    #define GYRO_SPL 16.4
    #define ACC_SPL  4096.0
#endif

//===================================================�궨��===================================================

//ŷ���ǽṹ��
typedef struct
{
    double pitch;
    double roll;
    double yaw;
}eulerAngle_info_struct;

//��Ԫ���ṹ��
typedef struct {
    double q0;
    double q1;
    double q2;
    double q3;
}quater_info_struct;

//���ٶ�Ư����
typedef struct
{
    double Xdata;
    double Ydata;
    double Zdata;
} gyroOffset_info_struct;

//���ٶȼ��ٶ�Ư����
typedef struct
{
    double Xdata;
    double Ydata;
    double Zdata;
} accOffset_info_struct;

//===================================================��������===================================================
//extern quater_info_struct quater;
extern eulerAngle_info_struct eulerAngle;
extern gyroOffset_info_struct gyroOffset;
extern accOffset_info_struct accOffset;
extern double imu_kp;                                             //���ٶȼƵ��������ʱ�������
extern double imu_ki;                                             //�������������ʵĻ�������
extern double I_ex, I_ey, I_ez;                                  // ������
extern double ex, ey, ez;
//===================================================��������===================================================

//===================================================��������===================================================
void imu_task(void);
void Update_Angle(void);
//void Update_Angle(double mx,double my,double mz);
void imu_data_deal(void);
void gyroOffsetInit(void);
void accOffsetInit(void);
//===================================================��������===================================================



#endif /* CODE_QUATER_H_ */
