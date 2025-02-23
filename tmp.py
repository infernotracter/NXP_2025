import math
# 四元数姿态解算相关变量
q0 = 1.0
q1 = 0.0
q2 = 0.0
q3 = 0.0
I_ex = 0.0
I_ey = 0.0
I_ez = 0.0
imu_kp = 1.5       # 比例增益（调整滤波响应速度）
imu_ki = 0.0005    # 积分增益（调整积分速度）
delta_T = 0.003    # 采样周期（与3ms中断对应）
current_pitch = 0  # 当前俯仰角
current_roll = 0   # 当前横滚角
current_yaw = 0    # 当前偏航角

def quaternion_update(ax, ay, az, gx, gy, gz):
    global q0, q1, q2, q3, I_ex, I_ey, I_ez, current_pitch, current_roll, current_yaw
    
    # 归一化加速度计数据
    norm = math.sqrt(ax**2 + ay**2 + az**2)
    if norm == 0:
        return
    ax /= norm
    ay /= norm
    az /= norm

    # 计算预测的重力方向
    vx = 2 * (q1*q3 - q0*q2)
    vy = 2 * (q0*q1 + q2*q3)
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3

    # 计算误差（叉积）
    ex = (ay * vz - az * vy)
    ey = (az * vx - ax * vz)
    ez = (ax * vy - ay * vx)

    # 误差积分
    I_ex += ex * imu_ki
    I_ey += ey * imu_ki
    I_ez += ez * imu_ki

    # 调整陀螺仪数据（弧度制）
    gx = math.radians(gx) + imu_kp * ex + I_ex
    gy = math.radians(gy) + imu_kp * ey + I_ey
    gz = math.radians(gz) + imu_kp * ez + I_ez

    # 四元数积分（一阶龙格库塔法）
    half_T = 0.5 * delta_T
    q0_temp = (-q1*gx - q2*gy - q3*gz) * half_T
    q1_temp = ( q0*gx + q2*gz - q3*gy) * half_T
    q2_temp = ( q0*gy - q1*gz + q3*gx) * half_T
    q3_temp = ( q0*gz + q1*gy - q2*gx) * half_T

    # 更新四元数
    q0 += q0_temp
    q1 += q1_temp
    q2 += q2_temp
    q3 += q3_temp

    # 四元数归一化
    norm = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
    q0 /= norm
    q1 /= norm
    q2 /= norm
    q3 /= norm

    # 计算欧拉角
    current_pitch = math.degrees(math.asin(2 * (q0*q2 - q1*q3)))
    current_roll = math.degrees(math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)))
    current_yaw = math.degrees(math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)))

    # 在ticker_flag_3ms的处理部分修改：
if (ticker_flag_3ms):
    # 读取IMU数据（假设imu_data包含[ax, ay, az, gx, gy, gz]）
    imu_data = imu.get()  
    ax = imu_data[0]
    ay = imu_data[1]
    az = imu_data[2]
    gx = imu_data[3]  # 陀螺仪X轴（可能需要根据坐标系调整）
    gy = imu_data[4]  # 陀螺仪Y轴
    gz = imu_data[5]  # 陀螺仪Z轴
    
    # 执行四元数更新
    quaternion_update(ax, ay, az, gx, gy, gz)
    
    # 后续可以直接使用current_pitch等变量
    # 例如替换原来的current_pitch计算：
    # current_pitch = calculate_pitch(...) 
    
    ticker_flag_3ms = False

















class QuaternionFilter:
    def __init__(self, kp=1.5, ki=0.0005, dt=0.003):
        # 四元数状态
        self.q = [1.0, 0.0, 0.0, 0.0]  # q0, q1, q2, q3
        
        # 滤波器参数
        self.kp = kp      # 比例增益
        self.ki = ki      # 积分增益
        self.dt = dt      # 采样周期
        
        # 误差积分
        self.I_ex = 0.0
        self.I_ey = 0.0
        self.I_ez = 0.0
        
        # 欧拉角输出
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

    def update(self, accel, gyro):
        """ 
        更新姿态解算 
        :param accel: [ax, ay, az] 单位g
        :param gyro:  [gx, gy, gz] 单位deg/s
        """
        ax, ay, az = accel
        gx, gy, gz = map(math.radians, gyro)  # 转换为弧度
        
        # ---- 归一化加速度 ----
        norm = math.sqrt(ax**2 + ay**2 + az**2)
        if norm < 1e-6:
            return
        ax /= norm
        ay /= norm
        az /= norm

        # ---- 预测重力方向 ----
        q0, q1, q2, q3 = self.q
        vx = 2 * (q1*q3 - q0*q2)
        vy = 2 * (q0*q1 + q2*q3)
        vz = q0**2 - q1**2 - q2**2 + q3**2

        # ---- 误差计算 ----
        ex = (ay * vz - az * vy)
        ey = (az * vx - ax * vz)
        ez = (ax * vy - ay * vx)

        # ---- 积分项 ----
        self.I_ex += ex * self.ki
        self.I_ey += ey * self.ki
        self.I_ez += ez * self.ki

        # ---- 补偿陀螺仪 ----
        gx += self.kp * ex + self.I_ex
        gy += self.kp * ey + self.I_ey
        gz += self.kp * ez + self.I_ez

        # ---- 四元数积分 ----
        half_dt = 0.5 * self.dt
        q0_temp = (-q1*gx - q2*gy - q3*gz) * half_dt
        q1_temp = ( q0*gx + q2*gz - q3*gy) * half_dt
        q2_temp = ( q0*gy - q1*gz + q3*gx) * half_dt
        q3_temp = ( q0*gz + q1*gy - q2*gx) * half_dt
        
        self.q = [
            q0 + q0_temp,
            q1 + q1_temp,
            q2 + q2_temp,
            q3 + q3_temp
        ]

        # 归一化
        norm = math.sqrt(sum(x**2 for x in self.q))
        self.q = [x/norm for x in self.q]

        # ---- 计算欧拉角 ----
        q0, q1, q2, q3 = self.q
        self.pitch = math.degrees(math.asin(2*(q0*q2 - q1*q3)))
        self.roll = math.degrees(math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)))
        self.yaw = math.degrees(math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)))

    def get_euler(self):
        """ 获取欧拉角 (degrees) """
        return self.pitch, self.roll, self.yaw