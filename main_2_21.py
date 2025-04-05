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

# 定义一个回调函数
ticker_flag = False
def time_pit_handler(time):
    global ticker_flag
    ticker_flag = True
# 实例化 PIT ticker 模块
pit1 = ticker(1)
pit1.capture_list(ccd, tof, encoder_l, encoder_r)
pit1.callback(time_pit_handler)
pit1.start(5)


# 定义一个回调函数
ticker_flag_3ms = False
def time_pit_3ms_handler(time):
    global ticker_flag_3ms
    ticker_flag_3ms = True
# 实例化 PIT ticker 模块
pit2 = ticker(2)
pit2.capture_list(imu, key)
pit2.callback(time_pit_3ms_handler)
pit2.start(3)


# 定义一个回调函数
ticker_flag_1000ms = False
def time_pit_1000ms_handler(time):
    global ticker_flag_1000ms
    ticker_flag_1000ms = True
# 实例化 PIT ticker 模块
pit3 = ticker(3)
pit3.capture_list()
pit3.callback(time_pit_1000ms_handler)
pit3.start(1000)

# 定义一个回调函数
ticker_flag_2ms = False
def time_pit_2ms_handler(time):
    global ticker_flag_2ms
    ticker_flag_2ms = True
# 实例化 PIT ticker 模块
pit4 = ticker(4)
pit4.capture_list()
pit4.callback(time_pit_2ms_handler)
pit4.start(10)

# 定义一个回调函数
ticker_flag_10ms = False
def time_pit_10ms_handler(time):
    global ticker_flag_10ms
    ticker_flag_10ms = True
# 实例化 PIT ticker 模块
pit5 = ticker(5)
pit5.capture_list()
pit5.callback(time_pit_10ms_handler)
pit5.start(10)


# 定义一个回调函数
ticker_flag_50ms = False
def time_pit_50ms_handler(time):
    global ticker_flag_50ms
    ticker_flag_50ms = True
# 实例化 PIT ticker 模块
pit6 = ticker(6)
pit6.capture_list()
pit6.callback(time_pit_50ms_handler)
pit6.start(50)


# 初始化变量
ccd_data1 = [0] * 128  # ccd1原始数组
ccd_data2 = [0] * 128  # ccd2原始数组
encl_data = 0  # 左编码器数据
encr_data = 0  # 右数据编码器
# tof_data = 0  # TOF数据
key_data = [0] * 4  # 按键数据
threshold1 = 0  # ccd1阈值
threshold2 = 0  # ccd2阈值
image_value1 = [0] * 128  # ccd1的二值化数组
image_value2 = [0] * 128  # ccd2的二值化数组
Mid_point1 = 0  # ccd1的中点
Mid_point2 = 0  # ccd2的中点
error1 = 0  # ccd1的误差
error2 = 0  # ccd2的误差
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
turn_k = 1  # 直接传error2后的比例
current_pitch = 0  # 当前俯仰角

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
        my_limit(self.increment, -2000, 2000) # 限幅
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
        my_limit(self.increment, -2000, 2000) # 限幅
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
speed_pid = speed_ring(ki = 0.6, kp = 10.0)
angle_pid = angle_ring(ki = 0.6, kd = 10.0)
gyro_pid = gyro_ring(ki = 0.01, kp = 1.0)

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
        my_limit(self.increment, -2000, 2000) # 限幅
        self.out = self.kp * self.err + self.increment
        self.err_last = self.err
        # my_limit(self.out, -500, 500)
        return self.out

# 加速度计计算俯仰角（单位：弧度）
def calculate_pitch(ax, ay, az, gy, dt, prev_pitch, alpha=0.98):
    pitch_acc = math.atan2(ax, math.sqrt(ay**2 + az**2))  # 使用 ax 和 ay/az 的组合
    
    # 转换为角度（可选）
    pitch_acc_deg = math.degrees(pitch_acc)
    
    # 陀螺仪积分计算角度变化
    pitch_gyro = prev_pitch + gy * dt  # gy 是绕Y轴的角速度（单位：度/秒）
    
    # 互补滤波融合
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc_deg
    return pitch

# 卡尔曼滤波器参数
kfp_var_l = {
    'P': 1,      # 估算方差
    'G': 0.0,    # 卡尔曼增益
    'Q': 0.001,  # 过程噪声,Q增大,动态响应变快,收敛稳定性变坏
    'R': 0.38,   # 测量噪声,R增大,动态响应变慢,收敛稳定性变好
    'Output': 0  # 卡尔曼滤波器输出
}
kfp_var_r = {
    'P': 1,      # 估算方差
    'G': 0.0,    # 卡尔曼增益
    'Q': 0.001,  # 过程噪声,Q增大,动态响应变快,收敛稳定性变坏
    'R': 0.35,   # 测量噪声,R增大,动态响应变慢,收敛稳定性变好
    'Output': 0  # 卡尔曼滤波器输出
}


def kalman_filter(kfp, input):
    # 估算方差方程
    kfp['P'] = kfp['P'] + kfp['Q']

    # 卡尔曼增益方程
    kfp['G'] = kfp['P'] / (kfp['P'] + kfp['R'])

    # 更新最优值方程
    kfp['Output'] = kfp['Output'] + kfp['G'] * (input - kfp['Output'])

    # 更新方差方程
    kfp['P'] = (1 - kfp['G']) * kfp['P']

    return kfp['Output']

# 偏差计算
def get_offset(mid_point):
    # 计算误差并返回结果
    offset = 64.0 - mid_point
    return offset

# 用户函数：
# 二值化处理
def ccd_image_value(ccd_data, value):
    ccd_value = [0] * len(ccd_data)   # 定义一个空二值化列表
    for i in range(len(ccd_data)):    # 遍历0-127点
        temp_num = ccd_data[i]        # 暂存列表
        if temp_num > value:          # 比较阈值大小
            ccd_value[i] = 1          # 大于阈值为1
        else:
            ccd_value[i] = 0          # 小于为0
    return ccd_value

# 获得动态阈值
def ccd_get_threshold(ccd_data):
    value_max = ccd_data[4]          # 从第5个元素开始考虑最大值
    value_min = ccd_data[4]          # 从第5个元素开始考虑最小值

    # 遍历5-122
    for i in range(5, 123):
        value_max = max(value_max, ccd_data[i])   # 限幅在最大传入数据和第5个元素值上
        value_min = min(value_min, ccd_data[i])   # 限幅在最小传入数据和第5个元素值上

    threshold = (value_max + value_min) / 2      # 计算阈值
    threshold = min(max(75, threshold), 255)     # 阈值限幅在75-256之间
    return threshold

# ccd滤波
def ccd_filter(ccd_n):
    if ccd_n == 1:  # 对CCD1采集的图像进行滤波
        for i in range(1, 126):      # 防止越界
            if image_value1[i] == 1 and image_value1[i - 1] == 0 and image_value1[i + 1] == 0:
                image_value1[i] = 0  # 如果当前为1，前后都为0，则将当前设置为0
            elif image_value1[i] == 0 and image_value1[i - 1] == 1 and image_value1[i + 1] == 1:
                image_value1[i] = 1  # 如果当前为0，前后都为1，则将当前设置为1

    elif ccd_n == 2:  # 对CCD2采集的图像进行滤波
        for i in range(1, 126):      # 防止越界
            if image_value2[i] == 1 and image_value2[i - 1] == 0 and image_value2[i + 1] == 0:
                image_value2[i] = 0  # 如果当前为1，前后都为0，则将当前设置为0
            elif image_value2[i] == 0 and image_value2[i - 1] == 1 and image_value2[i + 1] == 1:
                image_value2[i] = 1  # 如果当前为0，前后都为1，则将当前设置为1

# 得到中点
left_point_1 = False
right_point_1 = False
last_mid_point_1 = 0  # 上次中点，应在函数外部初始化
mid_point_1 = 60      # 当前中点，应在函数外部初始化

def get_ccd1_mid_point(bin_ccd):
    global last_mid_point_1, mid_point_1, left_point_1, right_point_1

    # 搜索左边点，以上次中点作为这次的起搜点
    for l_point1 in range(last_mid_point_1, 1, -1):
        if bin_ccd[l_point1 - 1] == 0 and bin_ccd[l_point1] == 1:    # 判断是否与左边点一致
            left_point_1 = l_point1     # 左边点找到
            break
        elif l_point1 == 1:     # 如果找到1都没找到
            left_point_1 = 0    # 强制令左边点为0
            break

    # 搜索右边点，以上次中点作为这次的起搜点
    for r_point1 in range(last_mid_point_1, 126):  # 注意这里应该是128，因为索引是从0开始的
        if bin_ccd[r_point1] == 1 and bin_ccd[r_point1 + 1] == 0:    # 判断是否与右边点一致
            right_point_1 = r_point1    # 右边点找到
            break
        elif r_point1 == 126:      # 如果找到126都没找到
            right_point_1 = 127    # 强制右左边点为127
            break

    # 计算中点
    mid_point_1 = (left_point_1 + right_point_1) / 2
    # 以这次中点作为下次的上次中点
    last_mid_point_1 = int(mid_point_1)
    # 返回当前中点
    return mid_point_1


left_point_2 = False
right_point_2 = False
last_mid_point_2 = 0  # 上次中点，应在函数外部初始化
mid_point_2 = 60       # 当前中点，应在函数外部初始化


def get_ccd2_mid_point(bin_ccd):
    global last_mid_point_2, mid_point_2, left_point_2, right_point_2, n, N

    # 搜索左边点，以上次中点作为这次的起搜点
    for l_point2 in range(last_mid_point_2, 1, -1):
        if bin_ccd[l_point2 - 1] == 0 and bin_ccd[l_point2] == 1:    # 判断是否与左边点一致
            left_point_2 = l_point2     # 左边点找到
            break
        elif l_point2 == 1:     # 如果找到1都没找到
            left_point_2 = 0    # 强制令左边点为0
            break

    # 搜索右边点，以上次中点作为这次的起搜点
    for r_point2 in range(last_mid_point_2, 126):  # 注意这里应该是128，因为索引是从0开始的
        if bin_ccd[r_point2] == 1 and bin_ccd[r_point2 + 1] == 0:    # 判断是否与右边点一致
            right_point_2 = r_point2    # 右边点找到
            break
        elif r_point2 == 126:      # 如果找到126都没找到
            right_point_2 = 127    # 强制右左边点为127
            break

    # ccd2左圆环补线

    # 右边的线补左边的线
    if (n == 2 or n == 3):
        left_point_2 = right_point_2 - 55

    # 左边线补右边线
    if (n == 4):
        right_point_2 = left_point_2 + 68

    # 圆环内丢线补线
    if (n == 6):
        left_point_2 = right_point_2 - 55

    # 出环右边线丢失
    if (n == 7):
        right_point_2 = left_point_2 + 68

    # 找到右边线,让右边线补左边线直行出环
    if (n == 8):
        left_point_2 = right_point_2 - 55

    # ccd2右圆环补线
    if (m == 2) or (m == 3):
        right_point_2 = left_point_2 + 55
    if (m == 4):
        left_point_2 = right_point_2 - 68
    if (m == 6):
        right_point_2 = left_point_2 + 55
    if (m == 7):
        left_point_2 = right_point_2 - 68
    if (m == 8):
        right_point_2 = left_point_2 + 55

    # 计算中点
    mid_point_2 = (left_point_2 + right_point_2) / 2
    # 以这次中点作为下次的上次中点
    last_mid_point_2 = int(mid_point_2)
    # 返回当前中点
    return mid_point_2


# ccd2为近端ccd   ccd1为远端
flag = False  # 斑马线标志
zebra = 0  # 斑马线个数


def search_element():
    global error2
    global right_point_1
    global left_point_1
    global left_point_2
    global right_point_2
    global flag, zebra
    global ccd_data2

    # 斑马线检测
    for i in range(54, len(ccd_data2) - 54):
        if (abs(ccd_data2[i] - ccd_data2[i + 2]) >= 50):
            zebra += 1
    if zebra >= 8:
        flag = True
    else:
        zebra = 0

    # 远端左边丢线状态
    if (left_point_1 < 7) and (30 <= left_point_2 <= 42) and (79 <= right_point_1 <= 99) and (abs(right_point_1-right_point_2) <= 10):
        n = 1
    # 近端左边丢线，用右边线补左线
    if (left_point_2 < 7) and (79 <= right_point_2 <= 99) and (abs(right_point_1-right_point_2) <= 10) and n == 1:
        n = 2
    # 近端ccd找到左边线的最大值
    if (20 <= left_point_2 <= 42) and (abs(right_point_1-right_point_2) <= 10) and n == 2:
        n = 3

    # 正在进入圆环，右丢线，进入补线状态
    if (20 <= left_point_2 <= 42) and (right_point_2 > 108) and n == 3:
        n = 4

    # 能找到左右边线，说明已进入圆环
    if (20 <= left_point_2 <= 42) and (88 <= right_point_2 <= 108) and n == 4:
        n = 5
        step_error = error2
    # 在圆环中，左边线可能丢线,让右边线补左边线
    if (left_point_2 < 10) and (79 <= right_point_2 <= 99) and (n == 5):
        n = 6
    else:
        n = 4
    # 此时已出圆环,右边线会丢失,让error2暂时变为之前的赛道误差
    if (20 <= left_point_2 <= 42) and (right_point_2 >= 110) and (n >= 4):
        n = 7
        error2 = step_error
    # 找到右边线，让右边线补左边线,直行出环
    if (88 <= right_point_2 <= 108) and n == 7:
        n = 8

#    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#    下面是右圆环

    # 远端右边丢线状态
    if (right_point_1 > 108) and (90 <= right_point_2 <= 108) and (29 <= left_point_1 <= 49) and (abs(left_point_1-left_point_2) <= 10):
        m = 1
    # 近端右边丢线，用左边线补右线
    if (right_point_2 > 108) and (29 <= left_point_2 <= 49) and (abs(left_point_1-left_point_2) <= 10) and m == 1:
        m = 2
    # 近端ccd找到右边线的最大值
    if (79 <= right_point_2 <= 104) and (abs(left_point_1-left_point_2) <= 10) and m == 2:
        m = 3

    # 进入补线模式
    if (79 <= right_point_2 <= 104) and (left_point_2 < 7) and n == 3:
        n = 4

    # 能找到左右边线，说明已进入圆环
    if (20 <= left_point_2 <= 42) and (88 <= right_point_2 <= 108) and m == 4:
        m = 5
        step_error = error2
    # 在圆环中，右边线可能丢线
    if (right_point_2 > 108) and (20 <= left_point_2 <= 49) and (m == 5):
        m = 6
    else:
        m = 4
    # 此时已出圆环,左边线会丢失,让error2暂时变为之前的赛道误差
    if (left_point_2 < 11) and (78 <= right_point_2 <= 108) and (m >= 4):
        m = 7
        error2 = step_error
    # 找到左边线，让左边线补右边线,直行出环
    if (29 <= left_point_2 <= 49) and m == 7:
        m = 8


main_point_item = 30
main_menu_flag = 1
car_go_flag = 0
speed_flag = 0
element_flag = 0
angle_pd_flag = 0
speed_pi_flag = 0
gyro_pi_flag = 0
ccd_image_flag = 0
parameter_flag = 0
screen_off_flag = 0
save_para_flag = 0


def menu():
    global main_menu_flag, car_go_flag, speed_flag, element_flag, angle_pd_flag, speed_pi_flag, gyro_pi_flag, ccd_image_flag, screen_off_flag, save_para_flag
    if (main_menu_flag == 1):
        main_menu()
    if (car_go_flag == 1):
        sec_menu_01()
    if (speed_flag == 1):
        sec_menu_02()
    if (element_flag == 1):
        sec_menu_03()
    if (angle_pd_flag == 1):
        sec_menu_04()
    if (speed_pi_flag == 1):
        sec_menu_05()
    if (gyro_pi_flag == 1):
        sec_menu_06()
    if (ccd_image_flag == 1):
        sec_menu_07()
    if (parameter_flag == 1):
        sec_menu_08()
    if (screen_off_flag == 1):
        sec_menu_09()
    if (save_para_flag == 1):
        sec_menu_10()

    gc.collect()


def main_menu():  # 一级菜单
    global main_point_item, main_menu_flag, car_go_flag, speed_flag, element_flag, angle_pd_flag, speed_pi_flag, gyro_pi_flag, ccd_image_flag, screen_off_flag, save_para_flag
    lcd.str24(60, 0, "main_menu", 0x07E0)
    lcd.str16(16, 30, "car_go", 0xFFFF)
    lcd.str16(16, 46, "speed", 0xFFFF)
    lcd.str16(16, 62, "element", 0xFFFF)
    lcd.str16(16, 78, "turn_pd", 0xFFFF)
    lcd.str16(16, 94, "vertical_pd", 0xFFFF)
    lcd.str16(16, 110, "motor_pi", 0xFFFF)
    lcd.str16(16, 126, "ccd_image", 0xFFFF)
    lcd.str16(16, 142, "parameter", 0xFFFF)
    lcd.str16(16, 158, "screen_off", 0xFFFF)
    lcd.str16(16, 174, "save_para", 0xFFFF)

    lcd.str12(0, main_point_item, ">", 0xF800)
    if key_data[0]:
        main_point_item += 16
        key.clear(1)
        if main_point_item == 190:
            main_point_item = 30

    if key_data[1]:
        main_point_item -= 16
        key.clear(2)
        if main_point_item == 14:
            main_point_item = 174

    if main_point_item == 30 and key_data[2]:
        main_menu_flag = 0
        car_go_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 46 and key_data[2]:
        main_menu_flag = 0
        speed_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 62 and key_data[2]:
        main_menu_flag = 0
        element_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 78 and key_data[2]:
        main_menu_flag = 0
        angle_pd_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 94 and key_data[2]:
        main_menu_flag = 0
        speed_pi_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 110 and key_data[2]:
        main_menu_flag = 0
        gyro_pi_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 126 and key_data[2]:
        main_menu_flag = 0
        ccd_image_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 142 and key_data[2]:
        main_menu_flag = 0
        parameter_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 158 and key_data[2]:
        main_menu_flag = 0
        screen_off_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 174 and key_data[2]:
        main_menu_flag = 0
        save_para_flag = 1
        main_point_item = 30
        key.clear(3)

    gc.collect()


def sec_menu_01():
    global aim_speed, speed_flag, main_menu_flag
    lcd.str24(60, 0, "car_go", 0x07E0)
    lcd.str16(16, 30, "return", 0xFFFF)
    lcd.str16(16, 46, "It's mygo", 0xFFFF)

    lcd.str12(0, main_point_item, ">", 0xF800)
    if key_data[0]:
        main_point_item += 16
        key.clear(1)
        if main_point_item == 62:
            main_point_item = 30
    if key_data[1]:
        main_point_item -= 16
        key.clear(2)
        if main_point_item == 14:
            main_point_item = 46
    if key_data[2] and main_point_item == 30:
        car_go_flag = 0
        main_menu_flag = 1
        key.clear(3)
        main_point_item = 30
    if key_data[2] and main_point_item == 46:
        aim_speed = 100
        key.clear(3)

    gc.collect()


def sec_menu_02():
    global speed_flag, main_menu_flag, aim_speed_l, aim_speed_r
    lcd.str24(60, 0, "speed", 0x07E0)
    lcd.str16(16, 62, "return", 0xFFFF)
    lcd.str16(16, 30, "aim_speed_l={}".format(aim_speed_l), 0xFFFF)
    lcd.str16(16, 46, "aim_speed_r={}".format(aim_speed_r), 0xFFFF)

    if key_data[0]:
        main_point_item += 16
        key.clear(1)
        if main_point_item == 78:
            main_point_item = 30

    if key_data[1]:
        main_point_item -= 16
        key.clear(2)
        if main_point_item == 14:
            main_point_item = 62

    if main_point_item == 30:
        if key_data[2]:
            aim_speed_l += 5
            key.clear(3)
        if key_data[3]:
            aim_speed_l -= 5
            key.clear(4)

    if main_point_item == 46:
        if key_data[2]:
            aim_speed_r += 5
            key.clear(3)
        if key_data[3]:
            aim_speed_r -= 5
            key.clear(4)
    if main_point_item == 62 and key_data[2]:
        main_menu_flag = 1
        speed_flag = 0
        main_point_item = 30
        key.clear(3)

    gc.collect()


def sec_menu_03():  # 目前没用
    lcd.str24(60, 0, "element", 0x07E0)  # 二级菜单标题
    lcd.str16(16, 30, "return", 0xFFFF)  # 返回主菜单

    if (flag is '左圆环1'):
        lcd.str16(16, 110, 'left_round1', 0xFFFF)  # 左圆环1
    elif (flag is '左圆环2'):
        lcd.str16(16, 110, 'left_round1', 0xFFFF)  # 左圆环2
    elif (flag is '右圆环1'):
        lcd.str16(16, 110, 'right_round1', 0xFFFF)  # 右圆环1
    elif (flag is '右圆环2'):
        lcd.str16(16, 126, 'right_round2', 0xFFFF)  # 右圆环2
    elif (flag is '斑马线'):
        lcd.str16(16, 142, 'zebra', 0xFFFF)  # 斑马线

    gc.collect()


def sec_menu_04():
    global main_point_item, main_menu_flag, angle_pd_flag
    lcd.str24(60, 0, "angle_pd", 0x07E0)
    lcd.str16(16, 30, "angle_KP={}".format(angle_pid.kp), 0xFFFF)
    lcd.str16(16, 46, "angle_Kp+/- 0.01", 0xFFFF)
    lcd.str16(16, 62, "angle_Kp+/- 0.1", 0xFFFF)
    lcd.str16(16, 78, "angle_KD={}".format(angle_pid.kd), 0xFFFF)
    lcd.str16(16, 94, "angle_KD+/- 0.01", 0xFFFF)
    lcd.str16(16, 110, "angle_KD+/- 0.1", 0xFFFF)

    lcd.str16(16, 126, "return", 0xFFFF)

    if key_data[0]:
        main_point_item += 16
        key.clear(1)
        if main_point_item == 142:
            main_point_item = 30

    if key_data[1]:
        main_point_item -= 16
        key.clear(2)
        if main_point_item == 14:
            main_point_item = 126

    if main_point_item == 46:
        if key_data[2]:
            angle_pid.kp += 0.01
            key.clear(3)
        if key_data[3]:
            angle_pid.kp -= 0.01
            key.clear(4)
    if main_point_item == 62:
        if key_data[2]:
            angle_pid.kp += 0.1
            key.clear(3)
        if key_data[3]:
            angle_pid.kp -= 0.1
            key.clear(4)
    if main_point_item == 94:
        if key_data[2]:
            angle_pid.kd += 0.01
            key.clear(3)
        if key_data[3]:
            angle_pid.kd -= 0.01
            key.clear(4)
    if main_point_item == 110:
        if key_data[2]:
            angle_pid.kd += 0.1
            key.clear(3)
        if key_data[3]:
            angle_pid.kd -= 0.1
            key.clear(4)
    if main_point_item == 126:
        if key_data[2]:
            key.clear(3)
            angle_pd_flag = 0
            main_menu_flag = 1
            main_point_item = 30

    gc.collect()


def sec_menu_05():
    global speed_pi_flag, main_menu_flag, main_point_item
    lcd.str24(60, 0, "speed_pi", 0x07E0)
    lcd.str16(16, 30, "speed_kp={}".format(speed_pid.kp), 0xFFFF)
    lcd.str16(16, 46, "speed_kp +/- 0.01", 0xFFFF)
    lcd.str16(16, 62, "speed_kp +/- 0.1", 0xFFFF)
    lcd.str16(16, 78, "speed_kd={}".format(speed_pid.ki), 0xFFFF)
    lcd.str16(16, 94, "speed_kd +/- 0.01", 0xFFFF)
    lcd.str16(16, 110, "speed_kd +/- 0.1", 0xFFFF)

    lcd.str16(16, 126, "return", 0xFFFF)
    if key_data[0]:
        main_point_item += 16
        key.clear(1)
        if main_point_item == 142:
            main_point_item = 30
    if key_data[1]:
        main_point_item -= 16
        key.clear(2)
        if main_point_item == 14:
            main_point_item = 126
    if main_point_item == 46:
        if key_data[2]:
            speed_pid.kp += 0.01
            key.clear(3)
        if key_data[3]:
            speed_pid.kp -= 0.01
            key.clear(4)
    if main_point_item == 62:
        if key_data[2]:
            speed_pid.kp += 0.1
            key.clear(3)
        if key_data[3]:
            speed_pid.kp -= 0.1
            key.clear(4)
    if main_point_item == 94:
        if key_data[2]:
            speed_pid.ki += 0.01
            key.clear(3)
        if key_data[3]:
            speed_pid.ki -= 0.01
            key.clear(4)
    if main_point_item == 110:
        if key_data[2]:
            speed_pid.ki += 0.1
            key.clear(3)
        if key_data[3]:
            speed_pid.ki -= 0.1
            key.clear(4)
    if main_point_item == 126:
        if key_data[2]:
            speed_pi_flag = 0
            main_menu_flag = 1
            main_point_item = 30
            key.clear(3)

    gc.collect()


def sec_menu_06():
    global main_point_item, main_menu_flag, gyro_pi_flag
    lcd.str24(60, 0, "gyro_pi", 0x07E0)
    lcd.str16(16, 30, "gyro_kp={}".format(gyro_pid.kp), 0xFFFF)
    lcd.str16(16, 46, "gyro_kp+/- 0.01", 0xFFFF)
    lcd.str16(16, 62, "gyro_kp+/- 0.1", 0xFFFF)
    lcd.str16(16, 78, "gyro_ki={}".format(gyro_pid.ki), 0xFFFF)
    lcd.str16(16, 94, "gyro_ki+/- 0.01", 0xFFFF)
    lcd.str16(16, 110, "gyro_ki+/- 0.1", 0xFFFF)

    lcd.str16(16, 126, "return", 0xFFFF)
    if key_data[0]:
        main_point_item += 16
        key.clear(1)
        if main_point_item == 142:
            main_point_item = 30
    if key_data[1]:
        main_point_item -= 16
        key.clear(2)
        if main_point_item == 14:
            main_point_item = 126
    if main_point_item == 46:
        if key_data[2]:
            gyro_pid.kp += 0.01
            key.clear(3)
        if key_data[3]:
            gyro_pid.kp -= 0.01
            key.clear(4)
    if main_point_item == 62:
        if key_data[2]:
            gyro_pid.kp += 0.1
            key.clear(3)
        if key_data[3]:
            gyro_pid.kp -= 0.1
            key.clear(4)
    if main_point_item == 94:
        if key_data[2]:
            gyro_pid.ki += 0.01
            key.clear(3)
        if key_data[3]:
            gyro_pid.ki -= 0.01
            key.clear(4)
    if main_point_item == 110:
        if key_data[2]:
            gyro_pid.ki += 0.1
            key.clear(3)
        if key_data[3]:
            gyro_pid.ki -= 0.1
            key.clear(4)
    if main_point_item == 126:
        if key_data[2]:
            gyro_pi_flag = 0
            main_menu_flag = 1
            main_point_item = 30
            key.clear(3)

    gc.collect()


def sec_menu_07():
    global main_point_item, main_menu_flag, ccd_image_flag, ccd_data1, ccd_data2
    lcd.str24(60, 0, "ccd,image", 0x07E0)
    lcd.str16(16, 30, "return", 0xFFFF)
    lcd.wave(0, 64, 128, 64, ccd_data1)
    lcd.wave(0, 64, 128, 64, ccd_data2)
    lcd.line(64, 64, 64, 192, color=0x001F, thick=1)
    if key_data[2]:
        ccd_image_flag = 0
        main_menu_flag = 1
        key.clear(3)

    gc.collect()


def sec_menu_08():
    global main_point_item, parameter_flag, error1, error2
    global Mid_point1, Mid_point2, tof_data, out_l, out_r, encl_data, encr_data
    global left_point_1, right_point_1, left_point_2, right_point_2
    lcd.str24(60, 0, "parameter", 0x07E0)  # 二级菜单标题
    lcd.str16(16, 30, "return", 0xFFFF)  # 返回主菜单

    lcd.str16(16, 46, "tof = {:<4d}".format(tof_data), 0xFFFF)  # TOF数据
    lcd.str16(16, 62, "out_l = {:<4d}".format(out_l), 0xFFFF)  # 左环pid输出
    lcd.str16(16, 78, "out_r = {:<4d}".format(out_r), 0xFFFF)  # 右环pid输出
    lcd.str16(16, 94, "encl = {:<4d}".format(encl_data), 0xFFFF)  # 左编码器值
    lcd.str16(16, 110, "encr = {:<4d}".format(encr_data), 0xFFFF)  # 右编码器值
    lcd.str16(16, 126, "Mid_point1 = {:<.2f}".format(
        Mid_point1), 0xFFFF)  # ccd1中点
    lcd.str16(16, 142, "Mid_point2 = {:<.2f}".format(
        Mid_point2), 0xFFFF)  # ccd2中点
    lcd.str16(16, 158, "error1 = {:<.2f}".format(error1), 0xFFFF)  # ccd1误差
    lcd.str16(16, 174, "error2 = {:<.2f}".format(error2), 0xFFFF)  # ccd2误差
    lcd.str16(16, 190, "left_point_1 = {:<3d}".format(
        left_point_1), 0xFFFF)  # 上摄像头左边点
    lcd.str16(16, 206, "right_point_1 = {:<3d}".format(
        right_point_1), 0xFFFF)  # 上摄像头右边点
    lcd.str16(16, 222, "left_point_2 = {:<3d}".format(
        left_point_2), 0xFFFF)  # 下摄像头左边点
    lcd.str16(16, 238, "right_point_2 = {:<3d}".format(
        right_point_2), 0xFFFF)  # 下摄像头右边点
    lcd.str16(16, 254, "width_1 = {:<3d}".format(
        right_point_1 - left_point_1), 0xFFFF)  # ccd1计算赛道宽度
    lcd.str16(16, 270, "width_2 = {:<3d}".format(
        right_point_2 - left_point_2), 0xFFFF)  # ccd2计算赛道宽度

    if main_point_item == 30:
        if key_data[2]:
            parameter_flag = 0
            main_menu_flag = 1
            key.clear(3)

    gc.collect()


def sec_menu_09():
    lcd.clear(0x0000)
    return


def sec_menu_10():
    write_flash()  # 写入缓冲区

    buzzer.value(1)  # 蜂鸣器开
    lcd.clear(0xF800)  # 清屏
    time.sleep_ms(100)  # 延时
    main_menu_item = 1  # 返回一级菜单
    buzzer.value(0)  # 蜂鸣器关


# 写入缓冲区
def write_flash():
    global aim_speed_l, aim_speed_r
    os.chdir("/flash")  # 切换到 /flash 目录
    try:
        # 通过 try 尝试打开文件 因为 r+ 读写模式不会新建文件
        user_file = io.open("user_data.txt", "r+")
    except:
        # 如果打开失败证明没有这个文件 所以使用 w+ 读写模式新建文件
        user_file = io.open("user_data.txt", "w+")

    # 将指针移动到文件头 0 偏移的位置
    user_file.seek(0, 0)
    # 使用 write 方法写入数据到缓冲区

    user_file.write("%.4f\n" % (angle_pid.kp))
    user_file.write("%.4f\n" % (angle_pid.kd))
    user_file.write("%.4f\n" % (speed_pid.kp))
    user_file.write("%.4f\n" % (speed_pid.ki))
    user_file.write("%d\n" % (aim_speed_l))
    user_file.write("%d\n" % (aim_speed_r))

    # 将缓冲区数据写入到文件 清空缓冲区 相当于保存指令
    user_file.flush()

    # 将指针重新移动到文件头
    user_file.seek(0, 0)
    # 读取三行数据 到临时变量 分别强制转换回各自类型
    data1 = float(user_file.readline())
    data2 = float(user_file.readline())
    data3 = float(user_file.readline())
    data4 = float(user_file.readline())
    data5 = int(user_file.readline())
    data6 = int(user_file.readline())

    # 最后将文件关闭即可
    user_file.close()

#\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
while True:
    motor_l.duty(gyro_pid.out)
    motor_r.duty(out_l)

    # 拨码开关关中断
    if end_switch.value() == 0:
        pit1.stop()    # pit1关闭
        pit2.stop()    # pit2关闭
        pit3.stop()    # pit3关闭
        break          # 跳出判断

    # 编码器卡尔曼滤波
    output_encl = kalman_filter(kfp_var_l, encl_data)
    output_encr = kalman_filter(kfp_var_r, encr_data)


    # 3ms中断标志位
    if (ticker_flag_3ms):
        # menu()                           # 菜单显示

        ccd_data1 = ccd.get(0)           # 读取ccd1的数据
        ccd_data2 = ccd.get(1)           # 读取ccd2的数据
        imu_data = imu.get()             # 读取陀螺仪的数据

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

        # 读取IMU数据（假设 imu_data 包含 [ax, ay, az, gx, gy, gz]）
        ax = imu_data[0]
        ay = imu_data[1]
        az = imu_data[2]
        gy = imu_data[4]  # Y轴角速度
        
        # 计算时间间隔（假设3ms中断）
        dt = 0.003
        
        # 计算俯仰角
        current_pitch = calculate_pitch(ax, ay, az, gy, dt, prev_pitch)
        prev_pitch = current_pitch  # 更新历史值
        
        gc.collect()
        ticker_flag_3ms = False

    # 1s中断标志位
    if (ticker_flag_1000ms):
        ccd_data1 = [0] * 128          # 清空ccd1原始数据
        ccd_data2 = [0] * 128          # 清空ccd2原始数据
        image_value1 = [0] * 128       # 清空ccd1二值化数据
        image_value2 = [0] * 128       # 清空ccd2二值化数据
        ticker_flag_1000ms = False

    # 10ms中断标志位
    if (ticker_flag):
        encl_data = encoder_l.get()     # 读取左编码器的数据
        encr_data = encoder_r.get()     # 读取右编码器的数据
        # tof_data = tof.get()            # 读取TOF的数据
        key_data = key.get()            # 读取按键的数据

        # 之后加入圆环）元素的控制

        ticker_flag = False
    
    if (ticker_flag_2ms):
        gyro_pid.pid_standard_integral(angle_pid.out, imu_data[4])
        ticker_flag_2ms = False

    if (ticker_flag_10ms):
        speed_pid.pid_standard_integral(aim_speed, (output_encl + output_encr) / 2)
        ticker_flag_10ms = False

    if (ticker_flag_50ms):
        angle_pid.pid_standard_integral(speed_pid.out + MedAngle, current_pitch)
        ticker_flag_50ms = False

    # tun_kd(比较重要，能起到修正作用，使其走直线) 与 velicity_kp 是一个数量级（大小差不多）
    # turn_kp主要作用是放大
