# 基础库、NXP库、第三方库
from machine import *
from display import *
from smartcar import *
from seekfree import *
from math import *
import gc
import time
# import math
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

ticker_flag = False


# 定义一个回调函数
def time_pit_handler(time):
    global ticker_flag
    ticker_flag = True


# 实例化 PIT ticker 模块
pit1 = ticker(1)
pit1.capture_list(ccd, tof, encoder_l, encoder_r)
pit1.callback(time_pit_handler)
pit1.start(5)


ticker_flag_3ms = False
# 定义一个回调函数


def time_pit_3ms_handler(time):
    global ticker_flag_3ms
    ticker_flag_3ms = True


# 实例化 PIT ticker 模块
pit2 = ticker(2)
pit2.capture_list(imu, key)
pit2.callback(time_pit_3ms_handler)
pit2.start(3)


ticker_flag_1000ms = False
# 定义一个回调函数


def time_pit_1000ms_handler(time):
    global ticker_flag_1000ms
    ticker_flag_1000ms = True


# 实例化 PIT ticker 模块
pit3 = ticker(3)
pit3.capture_list()
pit3.callback(time_pit_1000ms_handler)
pit3.start(1000)


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
aim_speed = 100 #之后要可以使用KEY手动修改
aim_speed_l = 0  # 左轮期望速度
aim_speed_r = 0  # 右轮期望速度
output_encl = 0  # 左轮编码器滤波
output_encr = 0  # 右轮编码器滤波
out_l = 0  # 左轮输出值
out_r = 0  # 右轮输出值
MedAngle = 0
n = 0 #元素判断用
m = 0
turn_k = 1 #直接传error2后的比例


# 直立算法   修改self.Angle - self.Mid_Angle为self.Mid_Angle - self.Angle
class Vertical_PID:
    def __init__(self, Vertical_Kp, Vertical_Kd):
        self.Vertical_Kp = Vertical_Kp
        self.Vertical_Kd = Vertical_Kd
        # self.Mid_Angle = 0  # 机械中值，待测量
        # self.Angle = 0  # 传回的角度
        # self.gyro_Y = 0  # 传回的角速度
        self.Vertical_out = 0  # 输出值

    def Vertical(self, Mid_Angle, Angle, gyro_Y):
        self.Vertical_out = self.Vertical_Kp * (Mid_Angle - Angle) + self.Vertical_Kd * gyro_Y
        return self.Vertical_out

# 速度环
class Velocity_PID:
    def __init__(self, Velocity_KP, Velocity_KI):
        self.Velocity_KP = Velocity_KP
        self.Velocity_KI = Velocity_KI
        # self.aim_speed = 0
        # self.encoder_l = 0
        # self.encoder_r = 0
        self.speed_err = 0
        self.speed_lowout = 0
        self.speed_lowout_last = 0
        self.Velocity_out = 0

    def Velocity(self, aim_speed, encoder_l, encoder_r):
        # 计算偏差值
        self.speed_err = (encoder_l + encoder_r) - aim_speed
        # 低通滤波
        self.speed_lowout = 0.7 * self.speed_err + 0.3 * self.speed_lowout_last
        self.speed_lowout_last = self.speed_lowout
        # 积分
        self.encoder_s += self.speed_lowout
        # 积分限幅
        if self.encoder_s > 20000:
            self.encoder_s = 20000
        if self.encoder_s < -20000:
            self.encoder_s = -20000

        self.Velocity_out = self.Velocity_KP * self.speed_lowout + self.Velocity_KI * self.encoder_s
        return self.Velocity_out

# 转向环  添加了走直线的指令，走直线时抑制拐弯
class Turn_PID:
    def __init__(self, Turn_KP, Turn_KD):
        self.Turn_KP = Turn_KP
        self.Turn_KD = Turn_KD
        # self.gyro_Z = 0
        # self.aim_turn = 0
        self.Turn_out = 0

    def Turn(self, gyro_Z, aim_turn):
        #不需要用   实际值 - 目标值  因为陀螺仪测量偏航角的误差非常大
        if self.aim_turn == 0:
            self.Turn_out = self.Turn_KD * gyro_Z
        else:
            self.Turn_out = self.Turn_KP * aim_turn + self.Turn_KD * self.gyro_Z
            return self.Turn_out

#目前这一部分加入到主函数里去了，应该会方便点
#
#
# def Control():
#     verticalpid = Vertical_PID(Vertical_Kp=200, Vertical_Kd=0.5)
#     velocitypid = Velocity_PID(Velocity_KP=200, Velocity_KI=0.5)
#     turnpid = Turn_PID(Turn_KP=200, Turn_KD=0.5)

#     velocityout = velocitypid.Velocity(aim_speed, encoder_l, encoder_r)
#     Verticalout = verticalpid.Vertical(
#         velocityout + MedAngle, imu_data[5], imu_data[4])
#     turnout = turnpid.Turn(imu_data[6], aim_turn)
#     PWM_out = Verticalout
#     MOTOR_l = PWM_out - turnout
#     MOTOR_r = PWM_out + turnout

#     # 之后加上限幅


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
    if (left_point_1 < 7) and (30 <= left_point_2 <= 42)and (79 <= right_point_1 <= 99) and (abs(right_point_1-right_point_2) <= 10):
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


# speed_pid_l = motor_PID(kp_motor=10.0, ki_motor=0.6, kd_motor=0)  # 左电机PID初始化
# speed_pid_r = motor_PID(kp_motor=10.0, ki_motor=0.6, kd_motor=0)
verticalpid = Vertical_PID(Vertical_Kp=180.0, Vertical_Kd=2.3)
velocitypid = Velocity_PID(Velocity_KP=0.6, Velocity_KI=0.003)
turnpid = Turn_PID(Turn_KP=10.0, Turn_KD=0.6)  # p d待测
while True:

    # # 计算路程
    # distance_T += (encl_data + encr_data) / 2 / 1024 * 30 / 50 * 0.05 * 3.1415926

    # if (distance_T >= 27):
    #     motor_l.duty(0)
    #     motor_r.duty(0)
    # else:
    #     motor_l.duty(out_r)
    #     motor_r.duty(out_l)
        
    motor_l.duty(out_r)
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

    #turn_kd(比较重要，能起到修正作用，使其走直线) 与 velicity_kp 是一个数量级（大小差不多）
    #turn_kp主要作用是放大
    #------------------------------------------------------------------------
    #\\\\\\\\\\\\\\PID核心控制\\\\\\\\\\\\
    #------------------------------------------------------------------------
    velocityout = velocitypid.Velocity(aim_speed, output_encl, output_encr)
    Verticalout = verticalpid.Vertical(velocityout + MedAngle, imu_data[5], imu_data[4])
    if abs(error2 >= 10.0):
        turnout = turnpid.Turn(imu_data[6], error2 * turn_k) #turn_k  需要根据实际情况修改
    else:
        turnout = 0
    PWM_out = Verticalout
    MOTOR_l = PWM_out - turnout # * 4
    MOTOR_r = PWM_out + turnout # * 4

    # # 速度控制   转向系数待测
    # if (error2 >= 10.0):  # 左转
    #     aim_speed_l = T-(s_t.Turn(gyro_Z, error2)*4)
    #     aim_speed_r = T+(s_t.Turn(gyro_Z, error2)*4)
    # if (-10.0 <= error2 <= 10.0):  # 直行
    #     aim_speed_l = T+(s_t.Turn(gyro_Z, 0)*4)     # 无差速状态
    #     aim_speed_r = T+(s_t.Turn(gyro_Z, 0)*4)
    # if (error2 <= -10.0):  # 右转
    #     aim_speed_l = T - (s_t.Turn(gyro_Z, error2) * 4)
    #     aim_speed_r = T + (s_t.Turn(gyro_Z, error2) * 4)


    # 电机PID计算
    # out_l = speed_pid_l.motor_control(aim_speed = aim_speed_l, speed = output_encl)
    # out_r = speed_pid_r.motor_control(aim_speed = aim_speed_r, speed = output_encr)

    # 电机占空比
    # motor_l.duty(out_r)
    # motor_r.duty(out_l)

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
        search_element()

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
