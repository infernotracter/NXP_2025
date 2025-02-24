# 从 machine 库包含所有内容
from machine import *

from collections import namedtuple

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 从 seekfree 库包含 IMU963RA
from seekfree import IMU963RA

# 包含 gc 类
import gc
import math
# 开发板上的 C19 是拨码开关
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
end_state = end_switch.value()

# 调用 IMU963RA 模块获取 IMU963RA 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 可以不填 默认参数为 1 调整这个参数相当于调整采集分频
imu = IMU963RA()

# 单位换算用
ACC_SPL = 4096.0
GYRO_SPL = 16.4

ticker_flag = False
ticker_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)
# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
pit1.capture_list(imu)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(1)


Axis3f = namedtuple('Axis3f', ['x', 'y', 'z'])

class MadgwickFilter:
    def __init__(self, beta=0.1):
        self.beta = beta
        self.q1 = 1.0  # 初始四元数 (w, x, y, z)
        self.q2 = 0.0
        self.q3 = 0.0
        self.q4 = 0.0
        self.DEG2RAD = math.pi / 180.0
        self.RAD2DEG = 180.0 / math.pi

    def update(self, acc, gyro, mag, dt):
        # 归一化加速度计和磁力计数据
        ax, ay, az = self._normalize(acc)
        mx, my, mz = self._normalize(mag)
        gx = gyro.x * self.DEG2RAD
        gy = gyro.y * self.DEG2RAD
        gz = gyro.z * self.DEG2RAD

        # 计算中间变量
        q1, q2, q3, q4 = self.q1, self.q2, self.q3, self.q4
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = 2.0 * q1 * q3
        _2q3q4 = 2.0 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # 计算磁力计反馈
        _2q1mx = 2.0 * q1 * mx
        _2q1my = 2.0 * q1 * my
        _2q1mz = 2.0 * q1 * mz
        _2q2mx = 2.0 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = (-_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 
                - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4)
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        # 梯度下降算法校正步长
        s1 = (-_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay)
              - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
              + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))
        
        s2 = (_2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay)
              - 4.0 * q2 * (1 - 2.0 * q2q2 - 2.0 * q3q3 - az)
              + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
              + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))
        
        s3 = (-_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay)
              - 4.0 * q3 * (1 - 2.0 * q2q2 - 2.0 * q3q3 - az)
              + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
              + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))
        
        s4 = (_2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay)
              + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
              + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        # 归一化步长
        norm = math.sqrt(s1**2 + s2**2 + s3**2 + s4**2)
        if norm == 0:
            return Axis3f(0, 0, 0)  # 避免除以零
        s1 /= norm
        s2 /= norm
        s3 /= norm
        s4 /= norm

        # 计算四元数导数
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # 更新四元数
        self.q1 += qDot1 * dt
        self.q2 += qDot2 * dt
        self.q3 += qDot3 * dt
        self.q4 += qDot4 * dt

        # 归一化四元数
        norm = math.sqrt(self.q1**2 + self.q2**2 + self.q3**2 + self.q4**2)
        if norm == 0:
            return Axis3f(0, 0, 0)
        self.q1 /= norm
        self.q2 /= norm
        self.q3 /= norm
        self.q4 /= norm

        # 转换为欧拉角
        angle_x = -math.asin(2.0 * (self.q2 * self.q4 - self.q1 * self.q3)) * self.RAD2DEG
        angle_y = math.atan2(2.0 * (self.q1 * self.q2 + self.q3 * self.q4),
                            self.q1**2 - self.q2**2 - self.q3**2 + self.q4**2) * self.RAD2DEG
        angle_z = math.atan2(2.0 * (self.q2 * self.q3 + self.q1 * self.q4),
                            self.q1**2 + self.q2**2 - self.q3**2 - self.q4**2) * self.RAD2DEG

        return Axis3f(angle_x, angle_y, angle_z)

    def _normalize(self, v):
        norm = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        if norm == 0:
            return 0.0, 0.0, 0.0
        norm = 1.0 / norm
        return v.x * norm, v.y * norm, v.z * norm
    

# 初始化滤波器
filter = MadgwickFilter(beta=0.1)
# 零飘定义
gyrooffsetx = 0
gyrooffsety = 0
gyrooffsetz = 0
accoffsetx = 0
accoffsety = 0
accoffsetz = 0
OFFSETNUM = 1000

def imuoffsetinit():
    global accoffsetx,accoffsety,accoffsetz,gyrooffsetx,gyrooffsety,gyrooffsetz,OFFSETNUM
    for _ in range(OFFSETNUM):
        imu.get()
        accoffsetx += imu_data[0]
        accoffsety += imu_data[1]
        accoffsetz += imu_data[2]
        gyrooffsetx += imu_data[3]
        gyrooffsety += imu_data[4]
        gyrooffsetz += imu_data[5]
    gyrooffsetx /= OFFSETNUM
    gyrooffsety /= OFFSETNUM
    gyrooffsetz /= OFFSETNUM
    accoffsetx /= OFFSETNUM
    accoffsety /= OFFSETNUM
    accoffsetz /= OFFSETNUM
    return gyrooffsetx, gyrooffsety, gyrooffsetz, accoffsetx, accoffsety, accoffsetz
# 辅助滤波
imuoffsetinit()
last_imu_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,0, 0.0, 0.0]
while True:
    if (ticker_flag and ticker_count % 1 == 0):
        # 获取数据并强制转换为浮点数列表
        imu_data = [float(x) for x in imu.get()]
        
        # 低通滤波处理（加速度计）
        alpha = 0.8 # 0.65
        for i in range(3):
            # 先进行零偏校正和单位转换
            current_processed = (imu_data[i] - [accoffsetx, accoffsety, accoffsetz][i]) / ACC_SPL
            # 再应用滤波，使用上一次的滤波结果
            imu_data[i] = alpha * current_processed + (1 - alpha) * last_imu_data[i]
            # 更新历史值为当前滤波结果
            last_imu_data[i] = imu_data[i]

        # 陀螺仪单位转换（减去偏移后除以灵敏度）
        for i in range(3, 6):
            imu_data[i] = math.radians((imu_data[i] - [gyrooffsetx, gyrooffsety, gyrooffsetz][i-3]) / GYRO_SPL)

        acc = Axis3f(imu_data[0], imu_data[1], imu_data[2])
        gyro = Axis3f(imu_data[3], imu_data[4], imu_data[5])
        mag = Axis3f(imu_data[6], imu_data[7], imu_data[8])

        # 更新时间步长0.001秒
        dt = 0.001

        # 更新滤波器并获取欧拉角
        angle = filter.update(acc, gyro, mag, dt)

        print(f"{angle.x:.2f},{angle.y:.2f},{angle.z:.2f}")  
        ticker_flag = False
    if end_switch.value() != end_state:
        pit1.stop()
        print("Ticker stop.")
        break
    gc.collect()
