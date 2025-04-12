/*
 * gyro.c
 *
 *  Created on: 2024年8月28日
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

imu_info_struct imu;

void gyro_data_init(void)
{
    imu.gyro.count = 0;
    imu.gyro.zero_calc_flag = true;
}

void get_imu_data(void)
{
#if IMU_TYPE
    imu963ra_get_acc();         // 获取   963的加速度测量数值
    imu963ra_get_gyro();        // 获取963的角速度测量数值

#else
    imu660ra_get_acc();         // 获取963的加速度测量数值
    imu660ra_get_gyro();        // 获取963的角速度测量数值
#endif
}

//660ra
//陀螺仪获取角度
void gyro_get_angle(void)
{
#if IMU_TYPE
        // 简单积分
        imu.gyro.angle[Z] -= imu963ra_gyro_z /  GYRO_SPL * 0.005;
        imu.gyro.angle[Y] += imu963ra_gyro_y /  GYRO_SPL * 0.005;
        imu.gyro.angle[X] += imu963ra_gyro_x /  GYRO_SPL * 0.005;
#else
        // 简单积分
        imu.gyro.angle[Z] -= imu660ra_gyro_z / GYRO_SPL* 0.005;
        imu.gyro.angle[Y] += imu660ra_gyro_y / GYRO_SPL* 0.005;
        imu.gyro.angle[X] += imu660ra_gyro_x / GYRO_SPL* 0.005;
#endif


    if (imu.gyro.zero_calc_flag) // 计算零漂标志
    {
        // 计算零漂
        if (imu.gyro.count < ZERO_CALC_COUNT) // 零漂计算次数
        {
            imu.gyro.zero_angle[Z] += imu.gyro.angle[Z] - imu.gyro.last_angle[Z];
            imu.gyro.last_angle[Z] = imu.gyro.angle[Z];

            imu.gyro.zero_angle[Y] += imu.gyro.angle[Y] - imu.gyro.last_angle[Y];
            imu.gyro.last_angle[Y] = imu.gyro.angle[Y];

            imu.gyro.zero_angle[X] += imu.gyro.angle[X] - imu.gyro.last_angle[X];
            imu.gyro.last_angle[X] = imu.gyro.angle[X];

            imu.gyro.count++;
        }
        // 算出零漂
        else
        {
            imu.gyro.angle[Z] -= imu.gyro.zero_angle[Z];
            imu.gyro.zero_angle[Z] /= ZERO_CALC_COUNT;

            imu.gyro.angle[Y] -= imu.gyro.zero_angle[Y];
            imu.gyro.zero_angle[Y] /= ZERO_CALC_COUNT;

            imu.gyro.angle[X] -= imu.gyro.zero_angle[X];
            imu.gyro.zero_angle[X] /= ZERO_CALC_COUNT;

            imu.gyro.zero_calc_flag = false;
            imu.gyro.count = 0;
        }
    }
    // 减去零漂
    else
    {
        imu.gyro.angle[Z] -= imu.gyro.zero_angle[Z];
        imu.gyro.angle[Y] -= imu.gyro.zero_angle[Y];
        imu.gyro.angle[X] -= imu.gyro.zero_angle[X];

        if(imu.gyro.angle[Z] > 360)
            imu.gyro.angle[Z] -= 360;
        else if(imu.gyro.angle[Z] < 0)
            imu.gyro.angle[Z] += 360;
    }
}

//加速计获取角度
void acc_get_angle(void)
{
#if IMU_TYPE
        double acc_x = imu963ra_acc_x / ACC_SPL;
        double acc_y = imu963ra_acc_y / ACC_SPL;
        double acc_z = imu963ra_acc_z / ACC_SPL;

        imu.acc.angle[X]= RAD_TO_ANGLE(atan( acc_y / acc_z));
        imu.acc.angle[Y]= -RAD_TO_ANGLE(atan( acc_x / sqrt(acc_y * acc_y +acc_z * acc_z)));
#else
        double acc_x = imu660ra_acc_x / ACC_SPL;
        double acc_y = imu660ra_acc_y / ACC_SPL;
        double acc_z = imu660ra_acc_z / ACC_SPL;

        imu.acc.angle[X] = RAD_TO_ANGLE (atan( acc_y / acc_z));
        imu.acc.angle[Y] = RAD_TO_ANGLE (atan(acc_x / sqrt(acc_y * acc_y +acc_z * acc_z)));
#endif

}



