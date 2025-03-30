from machine import Pin
from seekfree import IMU963RA
import math

def my_limit(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value

class IMUHandler:
    def __init__(self):
        self.imu = IMU963RA()
        self.delta_T = 0.005  # 时间间隔（秒）
        # 零飘校准参数
        self.gyro_offset = [0.0, 0.0, 0.0]
        self.acc_offset = [0.0, 0.0, 0.0]
        
        # 四元数参数
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        
        # 误差积分
        self.I_ex = 0.0
        self.I_ey = 0.0
        self.I_ez = 0.0
        
        # 当前欧拉角
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        
        self.alpha = 0.5  # 低通滤波系数

        self.imu_kp = 1500  # 比例增益（调整滤波响应速度）
        self.imu_ki = 10  # 积分增益（调整积分速度）

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0

    def calibrate(self, samples=100):
        """零飘校准"""
        acc_temp = [0.0, 0.0, 0.0]
        gyro_temp = [0.0, 0.0, 0.0]
        
        for _ in range(samples):
            data = self.imu.get()
            for i in range(3):
                acc_temp[i] += data[i]
                gyro_temp[i] += data[i+3]
            # 添加适当延时确保采样间隔
        
        for i in range(3):
            self.acc_offset[i] = acc_temp[i] / samples
            self.gyro_offset[i] = gyro_temp[i] / samples

    def update(self):
        """更新姿态数据"""
        raw_data = self.imu.get()
        
        # 加速度计处理
        self.ax = (raw_data[0] - self.acc_offset[0]) / 4096.0
        self.ay = (raw_data[1] - self.acc_offset[1]) / 4096.0
        self.az = (raw_data[2] - self.acc_offset[2]) / 4096.0
        
        # 低通滤波
        self.ax = self.alpha * self.ax + (1 - self.alpha) * self.ax
        self.ay = self.alpha * self.ay + (1 - self.alpha) * self.ay
        self.az = self.alpha * self.az + (1 - self.alpha) * self.az
        
        # 陀螺仪处理（转换为弧度）
        self.gx = math.radians((raw_data[3] - self.gyro_offset[0]) / 16.4)
        self.gy = math.radians((raw_data[4] - self.gyro_offset[1]) / 16.4)
        self.gz = math.radians((raw_data[5] - self.gyro_offset[2]) / 16.4)
        
    def quaternion_update(self):
        """四元数姿态解算"""
        ax = self.ax
        ay = self.ay
        az = self.az
        gx = self.gx
        gy = self.gy
        gz = self.gz
        q0 = self.q0
        q1 = self.q1
        q2 = self.q2
        q3 = self.q3
        I_ex = self.I_ex
        I_ey = self.I_ey
        I_ez = self.I_ez
        imu_kp = self.imu_kp
        imu_ki = self.imu_ki
        delta_T = self.delta_T

        if ax == 0 or ay == 0 or az == 0:
            return

        # 归一化加速度计数据
        norm = math.sqrt(ax ** 2 + ay ** 2 + az ** 2)
        # if norm == 0:
        #     return
        ax /= norm
        ay /= norm
        az /= norm

        # 计算预测的重力方向
        vx = 2 * (q1 * q3 - q0 * q2)
        vy = 2 * (q0 * q1 + q2 * q3)
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

        # 计算误差（叉积）
        ex = (ay * vz - az * vy)
        ey = (az * vx - ax * vz)
        ez = (ax * vy - ay * vx)

        # 误差积分
        # 原代码直接累加，没有乘以imu_ki
        # 修正后与网上代码一致
        I_ex += ex * imu_ki
        I_ey += ey * imu_ki
        I_ez += ez * imu_ki

        # 限幅(-50, 50)可以试试
        my_limit(I_ex, -100, 100)
        my_limit(I_ey, -100, 100)
        my_limit(I_ez, -100, 100)

        # 调整陀螺仪数据（弧度制）
        gx = math.radians(gx) + imu_kp * ex + I_ex
        gy = math.radians(gy) + imu_kp * ey + I_ey
        gz = math.radians(gz) + imu_kp * ez + I_ez

        # 四元数积分（一阶龙格库塔法）
        half_T = 0.5 * delta_T
        q0_temp = (-q1 * gx - q2 * gy - q3 * gz) * half_T
        q1_temp = (q0 * gx + q2 * gz - q3 * gy) * half_T
        q2_temp = (q0 * gy - q1 * gz + q3 * gx) * half_T
        q3_temp = (q0 * gz + q1 * gy - q2 * gx) * half_T

        # 更新四元数
        q0 += q0_temp
        q1 += q1_temp
        q2 += q2_temp
        q3 += q3_temp

        # 四元数归一化
        norm = math.sqrt(q0 ** 2 + q1 ** 2 + q2 ** 2 + q3 ** 2)
        q0 /= norm
        q1 /= norm
        q2 /= norm
        q3 /= norm

        # 计算欧拉角
        current_pitch = math.degrees(math.asin(2 * (q0 * q2 - q1 * q3)))
        current_roll = math.degrees(math.atan2(
            2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2)))
        current_yaw = math.degrees(math.atan2(
            2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2)))


        
    def _update_euler_angles(self):
        """更新欧拉角"""
        # 根据四元数计算欧拉角
        self.roll = math.degrees(math.atan2(
            2 * (self.q0 * self.q1 + self.q2 * self.q3),
            1 - 2 * (self.q1**2 + self.q2**2)
        ))
        self.pitch = math.degrees(math.asin(
            2 * (self.q0 * self.q2 - self.q3 * self.q1)
        ))
        self.yaw = math.degrees(math.atan2(
            2 * (self.q0 * self.q3 + self.q1 * self.q2),
            1 - 2 * (self.q2**2 + self.q3**2)
        ))

    @property
    def angles(self):
        """获取当前欧拉角"""
        return (self.roll, self.pitch, self.yaw)

    @property
    def raw_data(self):
        """获取原始传感器数据"""
        return self.imu.get()