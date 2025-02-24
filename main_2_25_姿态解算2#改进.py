import numpy as np
from numba import njit  # 使用JIT编译加速

# 配置参数
DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi
MAG_DECLINATION = 5.0  # 根据实际地理位置设置磁偏角

@njit(cache=True)
class FastMadgwick:
    def __init__(self, sample_freq=100, beta=0.1):
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.beta = beta
        self.inv_sample_freq = 1.0 / sample_freq
        self.acc_lpf = np.zeros(3)
        self.mag_ref = np.array([1.0, 0.0, 0.0])  # 需校准
        
    def update(self, acc, gyro, mag):
        # 加速度计低通滤波
        alpha = 0.2
        self.acc_lpf = alpha * acc + (1 - alpha) * self.acc_lpf
        
        # 单位化加速度
        ax, ay, az = self._normalize(self.acc_lpf)
        
        # 磁力计校准和投影
        mx, my, mz = self._calibrate_mag(mag)
        mx, my, mz = self._normalize(np.array([mx, my, mz]))
        
        # 提取四元数
        q1, q2, q3, q4 = self.q
        
        # 预计算常用项 (展开关键计算)
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4
        
        # 磁力计投影优化
        hx = mx*(q1q1 + q2q2 - q3q3 - q4q4) + 2*(q2*q3 - q1*q4)*my + 2*(q2*q4 + q1*q3)*mz
        hy = 2*(q2*q3 + q1*q4)*mx + my*(q1q1 - q2q2 + q3q3 - q4q4) + 2*(q3*q4 - q1*q2)*mz
        hz = 2*(q2*q4 - q1*q3)*mx + 2*(q3*q4 + q1*q2)*my + mz*(q1q1 - q2q2 - q3q3 + q4q4)
        
        # 地磁参考向量
        bx = np.sqrt(hx**2 + hy**2)
        bz = hz
        
        # 梯度下降优化 (展开矩阵运算)
        s1 = -_2q3*(2*(q2*q4 - q1*q3) - ax) + _2q2*(2*(q1*q2 + q3*q4) - ay)
        s2 = _2q4*(2*(q2*q4 - q1*q3) - ax) + _2q1*(2*(q1*q2 + q3*q4) - ay) - 4*q2*(1 - 2*(q2q2 + q3q3) - az)
        s3 = -_2q1*(2*(q2*q4 - q1*q3) - ax) + _2q4*(2*(q1*q2 + q3*q4) - ay) - 4*q3*(1 - 2*(q2q2 + q3q3) - az)
        s4 = _2q2*(2*(q2*q4 - q1*q3) - ax) + _2q3*(2*(q1*q2 + q3*q4) - ay)
        
        # 归一化梯度
        norm = np.sqrt(s1**2 + s2**2 + s3**2 + s4**2)
        if norm > 1e-6:
            s1 /= norm
            s2 /= norm
            s3 /= norm
            s4 /= norm
        
        # 陀螺仪数据转换
        gx, gy, gz = gyro * DEG2RAD
        
        # 四元数微分方程 (展开优化)
        qDot1 = 0.5 * (-q2*gx - q3*gy - q4*gz) - self.beta * s1
        qDot2 = 0.5 * ( q1*gx + q3*gz - q4*gy) - self.beta * s2
        qDot3 = 0.5 * ( q1*gy - q2*gz + q4*gx) - self.beta * s3
        qDot4 = 0.5 * ( q1*gz + q2*gy - q3*gx) - self.beta * s4
        
        # 二阶龙格-库塔积分
        k1 = np.array([qDot1, qDot2, qDot3, qDot4]) * self.inv_sample_freq
        k2 = (k1 + 0.5 * k1 * self.inv_sample_freq) * self.inv_sample_freq
        self.q += k2
        
        # 四元数归一化
        self.q /= np.linalg.norm(self.q)
        return self.get_euler()
    
    def get_euler(self):
        q0, q1, q2, q3 = self.q
        
        # 横滚角 (绕X轴)
        roll = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
        
        # 俯仰角 (绕Y轴)
        sinp = 2*(q0*q2 - q3*q1)
        sinp = np.clip(sinp, -1.0, 1.0)
        pitch = np.arcsin(sinp)
        
        # 偏航角 (绕Z轴) 含磁偏角补偿
        yaw = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)) + MAG_DECLINATION * DEG2RAD
        
        return np.array([roll, pitch, yaw]) * RAD2DEG
    
    @staticmethod
    @njit
    def _normalize(v):
        norm = np.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        return v / norm if norm > 1e-6 else v
    
    @njit
    def _calibrate_mag(self, mag):
        return mag - self.mag_ref

# 使用示例
if __name__ == "__main__":
    # 初始化滤波器 (100Hz)
    filter = FastMadgwick(sample_freq=100)
    
    # 模拟输入 (需替换为实际传感器数据)
    acc = np.array([0.0, 0.0, 9.81])
    gyro = np.array([0.0, 0.0, 0.0])
    mag = np.array([0.2, 0.0, 0.4])
    
    # 运行滤波器
    angles = filter.update(acc, gyro, mag)
    print(f"Roll: {angles[0]:.2f}°, Pitch: {angles[1]:.2f}°, Yaw: {angles[2]:.2f}°")