# 基础库、NXP库、第三方库
from math import *
from menutext import *
import gc
import time
import utime
import math

# 单位换算用
ACC_SPL = 4096.0
GYRO_SPL = 16.4

pit_cont_pid = 0

# 定义一个回调函数
ticker_flag_gyro = False
ticker_flag_imu = False
ticker_flag_4ms = False
ticker_flag_5ms = False
ticker_flag_8ms = False
ticker_flag_angle = False
ticker_flag_speed = False
ticker_flag_menu = False


def time_pit_pid_handler(time):
    global ticker_flag_gyro, ticker_flag_angle, ticker_flag_speed, ticker_flag_menu, pit_cont_pid
    pit_cont_pid += 5
    if (pit_cont_pid % 5 == 0):
        ticker_flag_gyro = True
    if (pit_cont_pid % 10 == 0):
        ticker_flag_menu = True
    if (pit_cont_pid % 25 == 0):
        ticker_flag_angle = True
    if (pit_cont_pid >= 125):
        ticker_flag_speed = True
        pit_cont_pid = 0


# 实例化 PIT ticker 模块
pit0 = ticker(0)
pit0.capture_list(ccd, key, encoder_l, encoder_r)
pit0.callback(time_pit_pid_handler)
pit0.start(5)


def time_pit_imu_handler(time):
    global ticker_flag_imu
    ticker_flag_imu = True


pit1 = ticker(1)
pit1.capture_list(imu)
pit1.callback(time_pit_imu_handler)
pit1.start(5)


def time_pit_5ms_handler(time):
    global ticker_flag_5ms
    ticker_flag_5ms = True


# # 实例化 PIT ticker 模块
pit2 = ticker(2)
pit2.capture_list(encoder_l, encoder_r)
pit2.callback(time_pit_5ms_handler)
pit2.start(5)

pit_cont_dir = 0


def time_pit_turnpid_handler(time):
    global ticker_flag_4ms, ticker_flag_8ms, pit_cont_dir
    pit_cont_dir += 5
    if (pit_cont_dir % 20 == 0):
        ticker_flag_4ms = True
    if (pit_cont_dir >= 40):
        ticker_flag_8ms = True
        pit_cont_dir = 0


pit3 = ticker(3)
pit3.capture_list()
pit3.callback(time_pit_turnpid_handler)
pit3.start(5)


# n = 0  # 元素判断用
# m = 0
# error_k = 1  # 直接传error2后的比例
# speed_d = 50  # 速度增量
# 限幅函数

class TickerProfiler:
    def __init__(self, name, expected_interval_ms):
        self.name = name  # Ticker名称（如 "1ms"）
        self.expected_us = expected_interval_ms * 1000  # 预期间隔（微秒）
        self.last_ticks = 0  # 上一次触发时间戳
        self.first_trigger = True  # 首次触发标志

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


# 四元数姿态解算相关变量   #kp=50 ki=0.0001
q0 = 1.0
q1 = q2 = q3 = 0.0
I_ex = I_ey = I_ez = 0.0
imu_kp = 1500  # 比例增益（调整滤波响应速度）
imu_ki = 10  # 积分增益（调整积分速度）
delta_T = 0.001  # 采样周期（与1ms中断对应）
current_pitch = 0  # 当前俯仰角
current_roll = 0  # 当前横滚角
current_yaw = 0  # 当前偏航角


def quaternion_update(ax, ay, az, gx, gy, gz):
    global q0, q1, q2, q3, I_ex, I_ey, I_ez, current_pitch, current_roll, current_yaw

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


# 零飘定义
gyrooffsetx = 0
gyrooffsety = 0
gyrooffsetz = 0
accoffsetx = 0
accoffsety = 0
accoffsetz = 0
OFFSETNUM = 100
last_ax = 0
last_ay = 0
last_az = 0
last_gx = 0
last_gy = 0
last_gz = 0


def imuoffsetinit():
    global accoffsetx, accoffsety, accoffsetz, gyrooffsetx, gyrooffsety, gyrooffsetz, last_ax, last_ay, last_az, last_gx, last_gy, last_gz
    for _ in range(OFFSETNUM):
        imu_data = imu.get()
        accoffsetx += (imu_data[0] - last_ax)
        accoffsety += (imu_data[1] - last_ay)
        accoffsetz += (imu_data[2] - last_az)
        gyrooffsetx += (imu_data[3] - last_gx)
        gyrooffsety += (imu_data[4] - last_gy)
        gyrooffsetz += (imu_data[5] - last_gz)
        last_ax = imu_data[0]
        last_ay = imu_data[1]
        last_az = imu_data[2]
        last_gx = imu_data[3]
        last_gy = imu_data[4]
        last_gz = imu_data[5]
    accoffsetx /= OFFSETNUM
    accoffsety /= OFFSETNUM
    accoffsetz /= OFFSETNUM
    gyrooffsetx /= OFFSETNUM
    gyrooffsety /= OFFSETNUM
    gyrooffsetz /= OFFSETNUM


# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

# 在主程序初始化阶段创建实例
profiler_1ms = TickerProfiler("1ms", expected_interval_ms=1)
profiler_5ms = TickerProfiler("5ms", expected_interval_ms=5)
profiler_gyro = TickerProfiler("Gyro", expected_interval_ms=1)  # 示例值
profiler_angle = TickerProfiler("Angle", expected_interval_ms=5)
profiler_speed = TickerProfiler("Speed", expected_interval_ms=10)
profiler_4ms = TickerProfiler("4ms", expected_interval_ms=4)
profiler_8ms = TickerProfiler("8ms", expected_interval_ms=8)

imuoffsetinit()
stop_flag = 1
last_imu_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0]
data_wave = [0, 0, 0, 0, 0, 0, 0, 0]

key_data = key.get()
imu_data = imu.get()
imu_data_filtered = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0]

while True:
    if (current_roll >= 75) or (current_roll <= 20):
        stop_flag = 0

    motor_l.duty(my_limit(gyro_pid_out - dir_in_out, -3000, 3000))
    motor_r.duty(my_limit(gyro_pid_out + dir_in_out, -3000, 3000))

    # 拨码开关关中断
    if end_switch.value() == 1:
        pit1.stop()  # pit1关闭
        pit2.stop()  # pit2关闭
        pit3.stop()  # pit3关闭
        break  # 跳出判断

    # 1ms中断标志位
    if (ticker_flag_imu):
        profiler_1ms.update()
        imu_data = [float(x) for x in imu.get()]
        ticker_flag_imu = False

    if (ticker_flag_5ms):
        # profiler_5ms.update()
        encl_data = encoder_l.get()  # 读取左编码器的数据
        encr_data = encoder_r.get()  # 读取右编码器的数据
        # 原函数此时为圆环处理

        # 低通滤波处理（加速度计）
        alpha = 0.5
        for i in range(3):
            # 先进行零偏校正和单位转换
            current_processed = (
                                        imu_data[i] - [accoffsetx, accoffsety, accoffsetz][i]) / ACC_SPL
            # 再应用滤波，使用上一次的滤波结果
            imu_data_filtered[i] = alpha * current_processed + \
                                   (1 - alpha) * last_imu_data[i]
            # 更新历史值为当前滤波结果
            last_imu_data[i] = imu_data_filtered[i]

        # 陀螺仪单位转换（减去偏移后除以灵敏度）
        for i in range(3, 6):
            imu_data_filtered[i] = math.radians(
                (imu_data[i] - [gyrooffsetx, gyrooffsety, gyrooffsetz][i - 3]) / GYRO_SPL)
        # 四元数更新（使用解包后的变量）

        ax, ay, az = imu_data_filtered[0], imu_data_filtered[1], imu_data_filtered[2]
        gx, gy, gz = imu_data_filtered[3], imu_data_filtered[4], imu_data_filtered[5]
        quaternion_update(ax, ay, az, gx, gy, gz)

        ticker_flag_5ms = False

    if (ticker_flag_gyro):  # kp=100.1  ki=2.0000001
        profiler_gyro.update()
        gyro_pid_out = gyro_pid.calculate(angle_pid_out, imu_data[3])
        ticker_flag_gyro = False

    if (ticker_flag_angle):
        # profiler_angle.update()
        angle_pid_out = angle_pid.calculate(
            speed_pid_out + MedAngle, current_roll)
        ticker_flag_angle = False
    if (ticker_flag_menu):
        menu(key_data)
        key_data = key.get()
        ticker_flag_menu = False

    if (ticker_flag_speed):
        # profiler_speed.update()
        speed_pid_out = speed_pid.calculate(
            aim_speed, (encl_data + encr_data) / 2)
        ticker_flag_speed = False

    if (ticker_flag_4ms):
        # profiler_4ms.update()
        # dir_in_out = dir_in.calculate(dir_out_out, imu[4])
        ticker_flag_4ms = False

    if (ticker_flag_8ms):
        # profiler_8ms.update()
        # 定期进行数据解析
        data_flag = wireless.data_analysis()
        for i in range(0, 8):
            # 判断哪个通道有数据更新
            if (data_flag[i]):
                # 数据更新到缓冲
                data_wave[i] = wireless.get_data(i)
                # 将更新的通道数据输出到 Thonny 的控制台
                print("Data[{:<6}] updata : {:<.3f}.\r\n".format(
                    i, data_wave[i]))
                gyro_pid.kp = data_wave[0]
                gyro_pid.kd = data_wave[1]
                angle_pid.kp = data_wave[2]
                angle_pid.kd = data_wave[3]
                speed_pid.kp = data_wave[4]
                speed_pid.ki = data_wave[5]
        # 将数据发送到示波器
        wireless.send_oscilloscope(
            gyro_pid.kp, gyro_pid.kd, angle_pid.kp, angle_pid.kd,
            speed_pid.kp, current_yaw, current_roll, motor_l.duty())

        # dir_out_out = dir_out.calculate(0, (error1 + error2) * error_k)
        ticker_flag_8ms = False





