from basic_data import imu
import math
import time

class IMUHander:
    def __init__(self):
        self.data = [0.0] * 9
        self.offset_data = [0.0] * 9  # 存储各轴偏移量
        self.last_data = [0.0] * 9  # 保存上一次的IMU数据
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.current_yaw = 0.0
        self.alpha = 0.95
        self.last_time = time.time()
        self.calibrate_offsets()

    def calibrate_offsets(self, num_samples = 100):
        # 初始化上一次数据
        self.last_data = imu.get()
        # 累加差值
        for _ in range(num_samples):
            current_data = imu.get()
            for i in range(6):
                self.offset_data[i] += (current_data[i] - self.last_data[i])
                self.last_data[i] = current_data[i]
        # 计算平均值
        for i in range(6):
            self.offset_data[i] /= num_samples

    def update(self):
        # 获取原始数据并应用偏移
        self.data = imu.get()
        for i in range(6):
            self.data[i] -= self.offset_data[i]
        self.last_time = time.time()

    def get_pitch(self):
        self.update()
        current_time = time.time()
        delta_t = current_time - self.last_time
        pitch_gyro = self.current_pitch + self.data[4] * delta_t
        self.current_pitch = self.alpha * self.data[0] + (1 - self.alpha) * pitch_gyro
        self.last_time = current_time
        return self.current_pitch

imuhander = IMUHander()