from machine import Pin
from seekfree import IMU963RA
import math
import utime
import time
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
        self.offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
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

        self.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.get_offset(100)

    def get_offset(self, samples = 100):
        """获取IMU的零飘偏移量"""
        for _ in range(samples):
            data = self.imu.get()
            for i in range(6):
                self.offset[i] += (data[i] - self.last_data[i])
                self.last_data[i] = data[i]
        for i in range(6):
            self.offset[i] /= samples

    def update(self):
        """更新姿态数据"""
        raw_data = self.imu.get()
        
        # 加速度计处理
        for i in range(3):
            self.data[i] = (raw_data[i] - self.offset[i]) / 4096.0
            self.data[i] = self.alpha * self.data[i] + (1 - self.alpha) * self.last_data[i]
            self.last_data[i] = self.data[i]

        # 陀螺仪处理（转换为弧度）
        for i in range(3, 6):
            self.data[i] = math.radians((raw_data[i] - self.offset[i - 3]) / 16.4)
        
    def quaternion_update(self):
        """四元数姿态解算"""
        ax = self.data[0]
        ay = self.data[1]
        az = self.data[2]
        gx = self.data[3]
        gy = self.data[4]
        gz = self.data[5]
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
        self.pitch = math.degrees(math.asin(2 * (q0 * q2 - q1 * q3)))
        self.roll = math.degrees(math.atan2(
            2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2)))
        self.yaw = math.degrees(math.atan2(
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
    


class PID:
    def __init__(self, kp=0, ki=0, kd=0,
                 integral_limits=None, output_limits=None,
                 output_adjustment=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.integral_limits = integral_limits
        self.output_limits = output_limits
        self.output_adjustment = output_adjustment

    def calculate(self, target, current):
        error = target - current

        # 积分项处理
        self.integral += error * self.ki
        if self.integral_limits:
            self.integral = my_limit(self.integral, *self.integral_limits)

        # 微分项计算
        derivative = error - self.prev_error

        # PID输出计算
        output = (self.kp * error) + self.integral + (self.kd * derivative)

        # 输出限幅
        if self.output_limits:
            output = my_limit(output, *self.output_limits)

        # 特殊输出调整
        if self.output_adjustment:
            output = self.output_adjustment(output)

        self.prev_error = error
        return output
    

class TickerProfiler:
    def __init__(self, name, expected_interval_ms):
        self.name = name                # Ticker名称（如 "1ms"）
        self.expected_us = expected_interval_ms * 1000  # 预期间隔（微秒）
        self.last_ticks = 0             # 上一次触发时间戳
        self.first_trigger = True       # 首次触发标志

    def update(self):
        """更新并打印时间间隔（需在每次ticker触发时调用）"""
        current_ticks = utime.ticks_us()
        
        if not self.first_trigger:
            # 计算实际间隔（自动处理计数器溢出）
            actual_interval_us = utime.ticks_diff(current_ticks, self.last_ticks)
            
            # 打印带颜色标记的调试信息（可选）
            error = abs(actual_interval_us - self.expected_us)
            status = "OK" if error < self.expected_us * 0.1 else "WARN"
            color_code = "\033[32m" if status == "OK" else "\033[31m"
            print(f"{color_code}[{self.name} Ticker] 预期: {self.expected_us}us, 实际: {actual_interval_us}us\033[0m")
        
        # 更新状态
        self.last_ticks = current_ticks
        self.first_trigger = False