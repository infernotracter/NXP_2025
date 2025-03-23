/*
 * gyro.h
 *
 *  Created on: 2024��8��28��
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

#ifndef CODE_GYRO_H_
#define CODE_GYRO_H_

#define ZERO_CALC_COUNT  1000

//����
typedef enum
{
    X,
    Y,
    Z,
    NUM_XYZ
}imu_info_enum;

//������
typedef struct
{

   bool zero_calc_flag;//��Ư�����־

   double angle[NUM_XYZ];//�򵥻��ֵõ��ĽǶ�

   double last_angle[NUM_XYZ];//��һ�νǶ�

   double zero_angle[NUM_XYZ];//��Ư����

   double gyro[NUM_XYZ];//���ٶ�

   int count;

}gyroscope_info_struct;

//���ٶȼ�
typedef struct
{
    double angle[NUM_XYZ];//���ٶȼƵõ��ĽǶ�
    double acc[NUM_XYZ];//�Ǽ��ٶ�
}acc_info_struct;

//���ٶȼ�
typedef struct
{
    double mag[NUM_XYZ];//�Ǽ��ٶ�
}mag_info_struct;

//imu
typedef struct
{
    gyroscope_info_struct gyro; //������
    acc_info_struct acc; //���ٶȼ�
    mag_info_struct mag;//������
}imu_info_struct;

//===================================================��������===================================================
extern imu_info_struct imu;
extern double acc_ratio;      //���ٶȼƱ���
extern double gyro_ratio;    //�����Ǳ���
//===================================================��������===================================================

//===================================================��������===================================================
void get_imu_data(void);
void gyro_get_angle(void);
void acc_get_angle(void);
void gyro_data_init(void);
double angle_calc(double acc_angle, double gyro);
//===================================================��������===================================================

#endif /* CODE_GYRO_H_ */
