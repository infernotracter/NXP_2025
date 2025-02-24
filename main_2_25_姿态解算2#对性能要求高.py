import numpy as np
from dataclasses import dataclass
from math import sqrt, atan2, asin, degrees

@dataclass
class Axis3f:
    x: float
    y: float
    z: float

class MadgwickAHRS:
    def __init__(self, beta=0.1, sample_period=0.01):
        self.beta = beta          # 融合系数
        self.sample_period = sample_period  # 采样周期(秒)
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # 四元数 [w, x, y, z]
        self.last_valid_acc = np.zeros(3)  # 历史有效加速度数据
        self.last_valid_mag = np.zeros(3)  # 历史有效磁力计数据

    def update(self, acc: Axis3f, gyro: Axis3f, mag: Axis3f) -> Axis3f:
        """
        更新姿态估计
        :param acc: 加速度计 (m/s²)
        :param gyro: 陀螺仪 (deg/s)
        :param mag: 磁力计 (μT)
        :return: 欧拉角 (度)
        """
        # ========== 数据预处理 ==========
        # 转换陀螺仪数据为弧度/秒
        g = np.radians([gyro.x, gyro.y, gyro.z])
        
        # 处理加速度计数据
        a = self._process_accelerometer([acc.x, acc.y, acc.z])
        
        # 处理磁力计数据
        m = self._process_magnetometer([mag.x, mag.y, mag.z])
        
        # ========== 磁力计投影计算 ==========
        h = self._compute_magnetic_field(a, m)
        
        # ========== 梯度下降优化 ==========
        gradient = self._compute_gradient(a, h)
        
        # ========== 四元数更新 ==========
        self._update_quaternion(g, gradient)
        
        # ========== 转换为欧拉角 ==========
        return self._quat_to_euler()

    def _process_accelerometer(self, acc):
        """加速度计数据预处理"""
        acc = np.array(acc)
        norm = np.linalg.norm(acc)
        
        if norm < 1e-6:  # 无效数据
            return self.last_valid_acc
        else:
            acc_norm = acc / norm
            self.last_valid_acc = acc_norm
            return acc_norm

    def _process_magnetometer(self, mag):
        """磁力计数据预处理"""
        mag = np.array(mag)
        norm = np.linalg.norm(mag)
        
        if norm < 1e-6:  # 无效数据
            return self.last_valid_mag
        else:
            mag_norm = mag / norm
            self.last_valid_mag = mag_norm
            return mag_norm

    def _compute_magnetic_field(self, a, m):
        """计算地磁参考向量 (优化磁力计投影)"""
        q = self.q
        h = np.array([
            m[0]*(q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2) 
            + 2*m[1]*(q[1]*q[2] - q[0]*q[3]) 
            + 2*m[2]*(q[1]*q[3] + q[0]*q[2]),
            
            2*m[0]*(q[1]*q[2] + q[0]*q[3]) 
            + m[1]*(q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2)
            + 2*m[2]*(q[2]*q[3] - q[0]*q[1]),
            
            2*m[0]*(q[1]*q[3] - q[0]*q[2]) 
            + 2*m[1]*(q[2]*q[3] + q[0]*q[1])
            + m[2]*(q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2)
        ])
        
        # 地磁水平分量
        b = np.array([sqrt(h[0]**2 + h[1]**2), 0, h[2]])
        return b / np.linalg.norm(b)

    def _compute_gradient(self, a, h):
        """计算梯度下降方向 (优化计算效率)"""
        q = self.q
        j = np.array([
            [-2*q[2],  2*q[3], -2*q[0], 2*q[1]],
            [ 2*q[1], 2*q[0],  2*q[3], 2*q[2]],
            [     0, -4*q[1],  -4*q[2],     0],
            
            [-2*q[3], -2*q[2],  2*q[1], 2*q[0]],
            [ 2*q[0], -2*q[3],  2*q[2], 2*q[1]],
            [ 2*q[1],  2*q[0],  2*q[3], 2*q[2]],
            
            [ 2*q[2],  2*q[3], 2*q[0], 2*q[1]],
            [-2*q[0], -2*q[1], 2*q[2], 2*q[3]],
            [     0,     0,      0,      0    ]
        ])
        
        # 目标向量
        v = np.concatenate([a, h])
        
        # 计算误差
        error = j @ q - v
        gradient = j.T @ error
        
        # 归一化梯度
        if np.linalg.norm(gradient) > 0:
            gradient /= np.linalg.norm(gradient)
            
        return gradient

    def _update_quaternion(self, gyro, gradient):
        """四元数更新 (改进积分方法)"""
        # 陀螺仪积分项
        q_dot = 0.5 * np.array([
            -self.q[1]*gyro[0] - self.q[2]*gyro[1] - self.q[3]*gyro[2],
             self.q[0]*gyro[0] + self.q[2]*gyro[2] - self.q[3]*gyro[1],
             self.q[0]*gyro[1] - self.q[1]*gyro[2] + self.q[3]*gyro[0],
             self.q[0]*gyro[2] + self.q[1]*gyro[1] - self.q[2]*gyro[0]
        ])
        
        # 梯度修正项
        q_dot -= self.beta * gradient
        
        # 二阶龙格-库塔积分
        k1 = q_dot * self.sample_period
        k2 = (q_dot + 0.5*k1) * self.sample_period
        self.q += k2
        
        # 四元数归一化
        self.q /= np.linalg.norm(self.q)

    def _quat_to_euler(self):
        """四元数转欧拉角 (改进万向节锁处理)"""
        w, x, y, z = self.q
        
        # 计算旋转矩阵元素
        r11 = 1 - 2*y**2 - 2*z**2
        r12 = 2*(x*y - w*z)
        r13 = 2*(x*z + w*y)
        r23 = 2*(y*z - w*x)
        r33 = 1 - 2*x**2 - 2*y**2
        
        # 计算欧拉角
        roll = atan2(r23, r33)
        pitch = atan2(-r13, sqrt(r11**2 + r12**2))
        yaw = atan2(r12, r11)
        
        return Axis3f(
            x=degrees(roll),
            y=degrees(pitch),
            z=degrees(yaw)
        )

# 使用示例
if __name__ == "__main__":
    # 初始化滤波器
    filter = MadgwickAHRS(beta=0.1, sample_period=0.01)
    
    # 模拟传感器输入
    acc = Axis3f(0.0, 0.0, 9.81)  # 静止状态
    gyro = Axis3f(0.0, 0.0, 0.0)  # 无旋转
    mag = Axis3f(0.2, 0.0, 0.4)   # 假设地磁北向
    
    # 运行滤波器
    angle = filter.update(acc, gyro, mag)
    print(f"Roll: {angle.x:.2f}°, Pitch: {angle.y:.2f}°, Yaw: {angle.z:.2f}°")