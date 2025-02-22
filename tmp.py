# 在IMU处理部分添加以下类和方法
class QuaternionFilter:
    def __init__(self):
        # 四元数初始化
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        
        # 误差积分
        self.I_ex = 0.0
        self.I_ey = 0.0
        self.I_ez = 0.0
        
        # 滤波器参数
        self.imu_kp = 1.5    # 比例增益
        self.imu_ki = 0.0005 # 积分增益
        self.delta_T = 0.005 # 采样周期（根据实际中断间隔调整）
        
        # 校准参数
        self.gyro_offset = (0, 0, 0)
        self.acc_offset = (0, 0, 0)

    def calibrate_sensors(self, imu_samples=200):
        """传感器校准"""
        # 陀螺仪零偏校准
        gx, gy, gz = 0, 0, 0
        for _ in range(imu_samples):
            data = imu.get()  # 假设返回[ax, ay, az, gx, gy, gz]
            gx += data[3]
            gy += data[4]
            gz += data[5]
            time.sleep_ms(5)
        self.gyro_offset = (gx/imu_samples, gy/imu_samples, gz/imu_samples)

        # 加速度计校准
        ax, ay, az = 0, 0, 0
        for _ in range(imu_samples):
            data = imu.get()
            ax += data[0]
            ay += data[1]
            az += data[2]
            time.sleep_ms(5)
        self.acc_offset = (ax/imu_samples, ay/imu_samples, az/imu_samples)

    def update(self, ax, ay, az, gx, gy, gz):
        """四元数姿态解算"""
        # 去除零偏
        ax -= self.acc_offset[0]
        ay -= self.acc_offset[1]
        az -= self.acc_offset[2]
        gx -= self.gyro_offset[0]
        gy -= self.gyro_offset[1]
        gz -= self.gyro_offset[2]

        # 加速度归一化
        norm = math.sqrt(ax**2 + ay**2 + az**2)
        if norm == 0:
            return
        ax /= norm
        ay /= norm
        az /= norm

        # 预测重力向量
        vx = 2*(self.q1*self.q3 - self.q0*self.q2)
        vy = 2*(self.q0*self.q1 + self.q2*self.q3)
        vz = self.q0**2 - self.q1**2 - self.q2**2 + self.q3**2

        # 计算误差
        ex = ay*vz - az*vy
        ey = az*vx - ax*vz
        ez = ax*vy - ay*vx

        # 误差积分
        self.I_ex += ex
        self.I_ey += ey
        self.I_ez += ez

        # 角速度补偿
        gx += self.imu_kp * ex + self.imu_ki * self.I_ex
        gy += self.imu_kp * ey + self.imu_ki * self.I_ey
        gz += self.imu_kp * ez + self.imu_ki * self.I_ez

        # 转换为弧度
        gx = math.radians(gx)
        gy = math.radians(gy)
        gz = math.radians(gz)

        # 四元数更新（龙格库塔法）
        half_T = 0.5 * self.delta_T
        q0_temp = (-self.q1*gx - self.q2*gy - self.q3*gz) * half_T
        q1_temp = (self.q0*gx + self.q2*gz - self.q3*gy) * half_T
        q2_temp = (self.q0*gy - self.q1*gz + self.q3*gx) * half_T
        q3_temp = (self.q0*gz + self.q1*gy - self.q2*gx) * half_T

        self.q0 += q0_temp
        self.q1 += q1_temp
        self.q2 += q2_temp
        self.q3 += q3_temp

        # 四元数归一化
        norm = math.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)
        self.q0 /= norm
        self.q1 /= norm
        self.q2 /= norm
        self.q3 /= norm

        # 计算欧拉角
        pitch = math.degrees(math.asin(2*(self.q0*self.q2 - self.q1*self.q3)))
        roll = math.degrees(math.atan2(2*(self.q2*self.q3 + self.q0*self.q1), 
                                     self.q0**2 - self.q1**2 - self.q2**2 + self.q3**2))
        yaw = math.degrees(math.atan2(2*(self.q1*self.q2 + self.q0*self.q3),
                                    self.q0**2 + self.q1**2 - self.q2**2 - self.q3**2))
        
        return pitch, roll, yaw

# 在初始化部分添加
quat_filter = QuaternionFilter()
quat_filter.calibrate_sensors()  # 上电时执行一次校准

# 修改原来的IMU处理部分
if ticker_flag_3ms:
    imu_data = imu.get()
    ax, ay, az = imu_data[0], imu_data[1], imu_data[2]
    gx, gy, gz = imu_data[3], imu_data[4], imu_data[5]
    
    # 使用四元数滤波
    current_pitch, current_roll, _ = quat_filter.update(ax, ay, az, gx, gy, gz)
    
    # 替换原来的互补滤波
    # current_pitch = calculate_pitch(ax, ay, az, gy, dt, prev_pitch) 
    
    ticker_flag_3ms = False











#//////////////////////////////////////////////////////////
# 不同版本
import math
import time

class IMUProcessor:
    def __init__(self, imu_type=0):
        self.imu_type = imu_type  # 0:IMU660
        # 四元数初始化
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        # 校准参数
        self.gyro_offset = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        self.acc_offset = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        # 滤波器参数
        self.alpha = 0.2  # 低通滤波系数
        self.imu_kp = 1.5  # 比例增益
        self.imu_ki = 0.0005  # 积分增益
        self.I_ex = self.I_ey = self.I_ez = 0.0
        # 中断标志
        self._calibrating = False

    def gyro_calibration(self, get_gyro_data):
        """陀螺仪校准（可中断）"""
        self._calibrating = True
        try:
            for _ in range(200):
                if not self._calibrating:  # 中断检查点
                    print("Calibration interrupted")
                    return
                # 假设get_gyro_data返回(x,y,z)
                _, _, z = get_gyro_data()
                self.gyro_offset['Z'] += z
                time.sleep(0.005)
            self.gyro_offset['Z'] /= 200
        finally:
            self._calibrating = False

    def stop_calibration(self):
        """中断校准过程"""
        self._calibrating = False

    def update(self, acc_raw, gyro_raw, dt=0.005):
        """处理新数据并更新姿态
        :param acc_raw: (x,y,z) 原始加速度数据
        :param gyro_raw: (x,y,z) 原始角速度数据
        :param dt: 采样时间间隔(秒)
        """
        # 1. 数据预处理
        acc = self._lowpass_filter(acc_raw)
        gyro = self._process_gyro(gyro_raw, dt)
        
        # 2. 互补滤波姿态解算
        self._update_quaternion(acc, gyro, dt)
        
        # 3. 转换为欧拉角
        return self._quaternion_to_euler()

    def _lowpass_filter(self, raw_data):
        """一阶低通滤波"""
        if not hasattr(self, 'acc_filtered'):
            self.acc_filtered = list(raw_data)
        else:
            self.acc_filtered = [
                self.alpha * r + (1-self.alpha)*f
                for r, f in zip(raw_data, self.acc_filtered)
            ]
        return self.acc_filtered

    def _process_gyro(self, raw_data, dt):
        """处理陀螺仪数据
        公式：$\omega = \frac{\text{raw} - \text{offset}}{\text{scale}}$
        """
        # IMU660量程±2000dps时，scale=16.4[4](@ref)
        scale = 16.4  # 根据实际量程调整
        return [
            math.radians((r - o)/scale)  # 转换为rad/s
            for r, o in zip(raw_data, self.gyro_offset.values())
        ]

    def _update_quaternion(self, acc, gyro, dt):
        """四元数更新核心算法"""
        # 归一化加速度向量
        norm = 1 / math.sqrt(sum(a**2 for a in acc))
        ax, ay, az = [a * norm for a in acc]

        # 计算重力向量误差
        vx = 2*(self.q1*self.q3 - self.q0*self.q2)
        vy = 2*(self.q0*self.q1 + self.q2*self.q3)
        vz = self.q0**2 - self.q1**2 - self.q2**2 + self.q3**2

        ex = ay*vz - az*vy
        ey = az*vx - ax*vz
        ez = ax*vy - ay*vx

        # 积分误差
        self.I_ex += ex
        self.I_ey += ey
        self.I_ez += ez

        # 修正角速度
        gx = gyro[0] + self.imu_kp*ex + self.imu_ki*self.I_ex
        gy = gyro[1] + self.imu_kp*ey + self.imu_ki*self.I_ey
        gz = gyro[2] + self.imu_kp*ez + self.imu_ki*self.I_ez

        # 四元数更新（龙格库塔法）
        half_dt = 0.5 * dt
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        
        self.q0 += (-q1*gx - q2*gy - q3*gz) * half_dt
        self.q1 += ( q0*gx + q2*gz - q3*gy) * half_dt
        self.q2 += ( q0*gy - q1*gz + q3*gx) * half_dt
        self.q3 += ( q0*gz + q1*gy - q2*gx) * half_dt

        # 四元数归一化
        norm = 1 / math.sqrt(sum(q**2 for q in (self.q0, self.q1, self.q2, self.q3)))
        self.q0 *= norm
        self.q1 *= norm
        self.q2 *= norm
        self.q3 *= norm

    def _quaternion_to_euler(self):
        """四元数转欧拉角"""
        pitch = math.asin(2 * self.q0 * self.q2 - 2 * self.q1 * self.q3)
        roll = math.atan2(2 * self.q2 * self.q3 + 2 * self.q0 * self.q1,
                         self.q0**2 - self.q1**2 - self.q2**2 + self.q3**2)
        yaw = math.atan2(2 * self.q1 * self.q2 + 2 * self.q0 * self.q3,
                        self.q0**2 + self.q1**2 - self.q2**2 - self.q3**2)
        
        # 转换为角度
        return (
            math.degrees(pitch),  # 俯仰角
            math.degrees(roll),   # 横滚角
            math.degrees(yaw)     # 航向角（需磁力计校正）
        )

# 测试用例
if __name__ == "__main__":
    imu = IMUProcessor()
    
    # 模拟数据输入（需替换为实际传感器读取）
    def mock_gyro():
        return (0.1, 0.05, 0.02)  # 模拟陀螺仪数据
    
    # 执行校准（可中断）
    imu.gyro_calibration(mock_gyro)
    
    # 模拟处理数据
    for _ in range(100):
        acc_data = (0.01, 0.02, 1.0)  # 模拟加速度计数据
        gyro_data = (0.1, 0.05, 0.02)
        pitch, roll, yaw = imu.update(acc_data, gyro_data)
        print(f"Pitch: {pitch:.2f}°, Roll: {roll:.2f}°, Yaw: {yaw:.2f}°")