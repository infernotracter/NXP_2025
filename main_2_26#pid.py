# 基础库、NXP库、第三方库
from machine import *
from display import *
from smartcar import *
from seekfree import *
from math import *
import gc
import time
import math
import os
import io

# 单位换算用
ACC_SPL = 4096.0
GYRO_SPL = 16.4

# 蜂鸣器初始化
buzzer = Pin('C9', Pin.OUT, pull=Pin.PULL_UP_47K, value=0)

# 发车初始化
Go = Pin('C21', Pin.IN, pull=Pin.PULL_UP_47K, value=0)

# 实例化 MOTOR_CONTROLLER 电机驱动模块
motor_l = MOTOR_CONTROLLER(
    MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty=0, invert=True)
motor_r = MOTOR_CONTROLLER(
    MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty=0, invert=True)

# 实例化 encoder 模块
encoder_l = encoder("D0", "D1", True)
encoder_r = encoder("D2", "D3")

# # 实例化 bldc 模块
# bldc1 = BLDC_CONTROLLER(BLDC_CONTROLLER.PWM_B26, freq = 300, highlevel_us = 1100)
# bldc2 = BLDC_CONTROLLER(BLDC_CONTROLLER.PWM_B27, freq = 300, highlevel_us = 1100)
# high_level_us = 1300
# dir = 1

# 实例化 lcd 模块
cs = Pin('C5', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
cs.high()
cs.low()
rst = Pin('B9', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc = Pin('B8', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
drv = LCD_Drv(SPI_INDEX=1, BAUDRATE=60000000, DC_PIN=dc,
              RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
lcd = LCD(drv)
lcd.color(0xFFFF, 0x0000)
lcd.mode(2)
lcd.clear(0x0000)

# 实例化 IMU660RA 模块
imu = IMU660RA()

# 核心板上的LED
led1 = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)

# 拨码开关2
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 拨码开关4
switch_3 = Pin('B14', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 拨码开关3
switch_4 = Pin('B15', Pin.IN, pull=Pin.PULL_UP_47K, value=True)

# 调用 TSL1401 模块获取 CCD 实例
ccd = TSL1401(3)

# 实例化 KEY_HANDLER 模块
key = KEY_HANDLER(10)
# 零号ticker记数，用于三个pid
pit_cont0 = 0

# 定义一个回调函数
ticker_flag_2ms = False
ticker_flag_1ms = False
ticker_flag_4ms = False
ticker_flag_5ms = False
ticker_flag_8ms = False
ticker_flag_10ms = False
ticker_flag_50ms = False


def time_pit_pid_handler(time):
    global ticker_flag_2ms, ticker_flag_10ms, ticker_flag_50ms, pit_cont0
    pit_cont0 += 1
    if (pit_cont0 == 2):
        ticker_flag_2ms = True
    if (pit_cont0 == 10):
        ticker_flag_10ms = True
    if (pit_cont0 == 50):
        ticker_flag_50ms = True
    if (pit_cont0 > 50):
        pit_cont0 = 1


# 实例化 PIT ticker 模块
pit0 = ticker(0)
pit0.capture_list(ccd, imu, key, encoder_l, encoder_r)
pit0.callback(time_pit_pid_handler)
pit0.start(1)


def time_pit_1ms_handler(time):
    global ticker_flag_1ms
    ticker_flag_1ms = True


pit1 = ticker(1)
pit1.capture_list(imu, key)
pit1.callback(time_pit_1ms_handler)
pit1.start(1)  # 之前为3，现在改为1


def time_pit_5ms_handler(time):
    global ticker_flag_5ms
    ticker_flag_5ms = True


# 实例化 PIT ticker 模块
pit2 = ticker(2)
pit2.capture_list(ccd, encoder_l, encoder_r)
pit2.callback(time_pit_5ms_handler)
pit2.start(5)

pit_cont3 = 0


def time_pit_turnpid_handler(time):
    global ticker_flag_4ms, ticker_flag_8ms
    pit_cont3 += 1
    if (pit_cont3 == 4):
        ticker_flag_4ms = True
    if (pit_cont3 == 8):
        ticker_flag_8ms = True


pit3 = ticker(3)
pit3.capture_list(ccd, imu)
pit3.callback(time_pit_turnpid_handler)
pit3.start(1)

# 初始化变量
ccd_data1 = [0] * 128  # ccd1原始数组
ccd_data2 = [0] * 128  # ccd2原始数组
encl_data = 0  # 左编码器数据
encr_data = 0  # 右数据编码器
# out = 0  # 舵机输出值
aim_speed = 100  # 之后要可以使用KEY手动修改
aim_speed_l = 0  # 左轮期望速度
aim_speed_r = 0  # 右轮期望速度
output_encl = 0  # 左轮编码器滤波
output_encr = 0  # 右轮编码器滤波
out_l = 0  # 左轮输出值
out_r = 0  # 右轮输出值
MedAngle = 0
n = 0  # 元素判断用
m = 0
error_k = 1  # 直接传error2后的比例


def my_limit(value, minn, maxn):
    if value < minn:
        value = minn
    if value > maxn:
        value = maxn


# 速度环
class speed_ring:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.err = 0
        self.err_last = 0
        self.out = 0
        self.increment = 0

    def pid_standard_integral(self, aim_speed, speed):
        self.err = aim_speed - speed
        self.increment += self.err * self.ki
        my_limit(self.increment, -2000, 2000)  # 限幅
        self.out = self.kp * self.err + self.increment
        self.err_last = self.err
        my_limit(self.out, -500, 500)
        return self.out


# 角速度环
class gyro_ring:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.err = 0
        self.err_last = 0
        self.out = 0
        self.increment = 0

    def pid_standard_integral(self, aim_gyro, gyro):
        self.err = aim_gyro - gyro
        self.increment += self.err * self.ki
        my_limit(self.increment, -2000, 2000)  # 限幅
        self.out = self.kp * self.err + self.increment
        self.err_last = self.err
        if self.out >= 0:
            self.out += 100
        else:
            self.out -= 100
        return self.out


# 角度环
class angle_ring:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.err = 0
        self.err_last = 0
        self.out = 0

    def pid_standard_integral(self, aim_angle, angle):
        self.err = aim_angle - angle
        self.out = self.kp * self.err + self.kd * (self.err - self.err_last)
        self.err_last = self.err
        return self.out


# PID实例化
speed_pid = speed_ring(kp=10.0, ki=0.6)
angle_pid = angle_ring(kp=10.0, kd=0.6)
gyro_pid = gyro_ring(kp=1.0, ki=0.01)


class dir_out_ring:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.err = 0
        self.err_last = 0
        self.out = 0

    def pid_standard_integral(self, aim_dir, dir):
        self.err = aim_dir - dir
        self.out = self.kp * self.err + self.kd * (self.err - self.err_last)
        self.err_last = self.err
        return self.out


class dir_in_ring:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.err = 0
        self.err_last = 0
        self.increment = 0
        self.out = 0

    def pid_standard_integral(self, aim_dir, dir):
        self.err = aim_dir - dir
        self.increment += self.err * self.ki
        my_limit(self.increment, -2000, 2000)  # 限幅
        self.out = self.kp * self.err + self.increment
        self.err_last = self.err
        # my_limit(self.out, -500, 500)
        return self.out


dir_in = dir_in_ring(kp=10.0, ki=0.6)
dir_out = dir_out_ring(kp=10.0, kd=0.6)

# 四元数姿态解算相关变量
q0 = 1.0
q1 = q2 = q3 = 0.0
I_ex = I_ey = I_ez = 0.0
imu_kp = 1.5  # 比例增益（调整滤波响应速度）
imu_ki = 0.0005  # 积分增益（调整积分速度）
delta_T = 0.001  # 采样周期（与1ms中断对应）
current_pitch = 0  # 当前俯仰角
current_roll = 0  # 当前横滚角
current_yaw = 0  # 当前偏航角


# 姿态角度计算函数
def quaternion_update(ax, ay, az, gx, gy, gz):
    global q0, q1, q2, q3, I_ex, I_ey, I_ez, current_pitch, current_roll, current_yaw

    # 归一化加速度计数据
    norm = math.sqrt(ax ** 2 + ay ** 2 + az ** 2)
    if norm == 0:
        return
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
    I_ex += ex * imu_ki
    I_ey += ey * imu_ki
    I_ez += ez * imu_ki

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
    current_roll = math.degrees(math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2)))
    current_yaw = math.degrees(math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2)))




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


# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
imuoffsetinit()  # 零飘校准
last_imu_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,0, 0.0, 0.0]
while True:
    motor_l.duty(gyro_pid.out - dir_in.out)
    motor_r.duty(gyro_pid.out + dir_in.out)

    ccd_data1 = ccd.get(0)  # 读取ccd1的数据
    ccd_data2 = ccd.get(1)  # 读取ccd2的数据

    # 计算动态阈值
    threshold1 = ccd_get_threshold(ccd_data1)
    threshold2 = ccd_get_threshold(ccd_data2)

    # 对ccd数据二值化处理
    image_value1 = ccd_image_value(ccd_data1, threshold1)
    image_value2 = ccd_image_value(ccd_data2, threshold2)

    # ccd滤波
    ccd_filter(1), ccd_filter(2)

    # 数组会存储最近十次的中线位置，并保证更新
    # 这个mid_line_long 的计算方法是，目前中线占10%权重，历史中线占90%权重，最后+n补充，目前n=0
    # 看看之后加不加    (●'◡'●)
    # 进行中点计算
    Mid_point1 = get_ccd1_mid_point(image_value1)
    Mid_point2 = get_ccd2_mid_point(image_value2)

    # 进行偏差计算
    error1 = get_offset(Mid_point1)
    error2 = get_offset(Mid_point2)

    # 元素识别
    # search_element()

    # 拨码开关关中断
    if end_switch.value() == 0:
        pit1.stop()  # pit1关闭
        pit2.stop()  # pit2关闭
        pit3.stop()  # pit3关闭
        break  # 跳出判断

    # 1ms中断标志位
    if (ticker_flag_1ms):
        imu_data = [float(x) for x in imu.get()]

        # 低通滤波处理（加速度计）
        alpha = 0.5
        for i in range(3):
            # 先进行零偏校正和单位转换
            current_processed = (imu_data[i] - [accoffsetx, accoffsety, accoffsetz][i]) / ACC_SPL
            # 再应用滤波，使用上一次的滤波结果
            imu_data[i] = alpha * current_processed + (1 - alpha) * last_imu_data[i]
            # 更新历史值为当前滤波结果
            last_imu_data[i] = imu_data[i]

        # 陀螺仪单位转换（减去偏移后除以灵敏度）
        for i in range(3, 6):
            imu_data[i] = math.radians((imu_data[i] - [gyrooffsetx, gyrooffsety, gyrooffsetz][i - 3]) / GYRO_SPL)
        # 四元数更新（使用解包后的变量）
        ax, ay, az = imu_data[0], imu_data[1], imu_data[2]
        gx, gy, gz = imu_data[3], imu_data[4], imu_data[5]
        quaternion_update(ax, ay, az, gx, gy, gz)
        gc.collect()
        ticker_flag_1ms = False

    if (ticker_flag_5ms):
         ncl_data = encoder_l.get()  # 读取左编码器的数据
         encr_data = encoder_r.get()  # 读取右编码器的数据
         key_data = key.get()  # 读取按键的数据
         # 原函数此时为圆环处理
         ticker_flag_5ms = False

    if (ticker_flag_2ms):
        gyro_pid.pid_standard_integral(angle_pid.out, imu_data[4])
        ticker_flag_2ms = False

    if (ticker_flag_10ms):
    # menu()                           # 菜单显示
        encl_data = encoder_l.get()  # 读取左编码器的数据
        encr_data = encoder_r.get()  # 读取右编码器的数据
        key_data = key.get()  # 读取按键的数据
        speed_pid.pid_standard_integral(aim_speed, (output_encl + output_encr) / 2)
        ticker_flag_10ms = False

    if (ticker_flag_50ms):
        angle_pid.pid_standard_integral(speed_pid.out + MedAngle, current_pitch)
        ticker_flag_50ms = False

    if (ticker_flag_4ms):
       # dir_in.pid_standard_integral(dir_out.out, imu[4])
        ticker_flag_4ms = False

    if (ticker_flag_8ms):
       # dir_out.pid_standard_integral(0, (error1 + error2) * error_k)
        ticker_flag_8ms = False