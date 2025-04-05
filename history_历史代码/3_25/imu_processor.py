import math

def my_limit(value, min_val, max_val):
    return max(min_val, min(value, max_val))

# class IMUProcessor:
#     def __init__(self):
#         # 四元数初始化
#         self.q0 = 1.0
#         self.q1 = 0.0
#         self.q2 = 0.0
#         self.q3 = 0.0
        
#         # 误差积分项
#         self.I_ex = 0.0
#         self.I_ey = 0.0
#         self.I_ez = 0.0
        
#         # 欧拉角输出
#         self.current_pitch = 0.0
#         self.current_roll = 0.0
#         self.current_yaw = 0.0
        
#         # 控制参数（示例值，需根据实际情况设置）
#         self.imu_kp = 0.5
#         self.imu_ki = 0.1
#         self.delta_T = 0.01

#     def quaternion_update(self, ax, ay, az, gx, gy, gz):
#         if ax == 0 or ay == 0 or az == 0:
#             return

#         # 归一化加速度计数据
#         norm = math.sqrt(ax**2 + ay**2 + az**2)
#         ax /= norm
#         ay /= norm
#         az /= norm

#         # 计算预测重力方向
#         vx = 2 * (self.q1 * self.q3 - self.q0 * self.q2)
#         vy = 2 * (self.q0 * self.q1 + self.q2 * self.q3)
#         vz = self.q0**2 - self.q1**2 - self.q2**2 + self.q3**2

#         # 计算误差
#         ex = (ay * vz - az * vy)
#         ey = (az * vx - ax * vz)
#         ez = (ax * vy - ay * vx)

#         # 误差积分
#         self.I_ex += ex * self.imu_ki
#         self.I_ey += ey * self.imu_ki
#         self.I_ez += ez * self.imu_ki

#         # 积分限幅
#         self.I_ex = max(min(self.I_ex, 100), -100)
#         self.I_ey = max(min(self.I_ey, 100), -100)
#         self.I_ez = max(min(self.I_ez, 100), -100)

#         # 调整陀螺仪数据
#         gx = math.radians(gx) + self.imu_kp * ex + self.I_ex
#         gy = math.radians(gy) + self.imu_kp * ey + self.I_ey
#         gz = math.radians(gz) + self.imu_kp * ez + self.I_ez

#         # 四元数积分
#         half_T = 0.5 * self.delta_T
#         self.q0 += (-self.q1 * gx - self.q2 * gy - self.q3 * gz) * half_T
#         self.q1 += (self.q0 * gx + self.q2 * gz - self.q3 * gy) * half_T
#         self.q2 += (self.q0 * gy - self.q1 * gz + self.q3 * gx) * half_T
#         self.q3 += (self.q0 * gz + self.q1 * gy - self.q2 * gx) * half_T

#         # 四元数归一化
#         norm = math.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)
#         self.q0 /= norm
#         self.q1 /= norm
#         self.q2 /= norm
#         self.q3 /= norm

#         # 计算欧拉角
#         self.current_pitch = math.degrees(math.asin(2 * (self.q0 * self.q2 - self.q1 * self.q3)))
#         self.current_roll = math.degrees(math.atan2(
#             2 * (self.q0 * self.q1 + self.q2 * self.q3), 
#             1 - 2 * (self.q1**2 + self.q2**2)))
#         self.current_yaw = math.degrees(math.atan2(
#             2 * (self.q0 * self.q3 + self.q1 * self.q2), 
#             1 - 2 * (self.q2**2 + self.q3**2)))
    



# 姿态角度计算函数
def quaternion_update(ax, ay, az, gx, gy, gz):
    # 四元数姿态解算相关变量   #kp=50 ki=0.0001
    q0 = 1.0
    q1 = q2 = q3 = 0.0
    I_ex = I_ey = I_ez = 0.0
    imu_kp = 250  # 比例增益（调整滤波响应速度）
    imu_ki = 0.0001  # 积分增益（调整积分速度）
    delta_T = 0.001  # 采样周期（与1ms中断对应）
    current_pitch = 0  # 当前俯仰角
    current_roll = 0  # 当前横滚角
    current_yaw = 0  # 当前偏航角

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
    
    return current_pitch, current_roll, current_yaw


