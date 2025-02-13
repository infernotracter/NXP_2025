# 基础库、NXP库、第三方库
from machine import *
from display import *
from smartcar import *
from seekfree import *
from math import *
import gc
import time
#import math
import os
import io


# 蜂鸣器初始化
buzzer = Pin('C9', Pin.OUT, pull = Pin.PULL_UP_47K, value = 0)

# 发车初始化
Go = Pin('C21', Pin.IN, pull = Pin.PULL_UP_47K, value = 0)

# 实例化 MOTOR_CONTROLLER 电机驱动模块
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty = 0, invert = True)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty = 0, invert = True)

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
led1 = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)

# 拨码开关2
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
# 拨码开关4
switch_3 = Pin('B14', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
# 拨码开关3
switch_4 = Pin('B15', Pin.IN, pull=Pin.PULL_UP_47K, value = True)

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
# threshold1 = 0  # ccd1阈值
# threshold2 = 0  # ccd2阈值
image_value1 = [0] * 128  # ccd1的二值化数组
image_value2 = [0] * 128  # ccd2的二值化数组
Mid_point1 = 0  # ccd1的中点
Mid_point2 = 0  # ccd2的中点
error1 = 0  # ccd1的误差
error2 = 0  # ccd2的误差
#out = 0  # 舵机输出值
aim_speed_l = 0  # 左轮期望速度
aim_speed_r = 0  # 右轮期望速度
output_encl = 0  # 左轮编码器滤波
output_encr = 0  # 右轮编码器滤波
out_l = 0  # 左轮输出值
out_r = 0  # 右轮输出值


class motor_PID:
    def __init__(self, kp_motor, ki_motor, kd_motor):
        self.kp_motor = kp_motor  # kp系数
        self.ki_motor = ki_motor  # ki系数
        self.kd_motor = kd_motor  # ki系数
        self.aim_speed = 0  # 期望速度
        self.speed = 0  # 实际速度
        self.speed_err = 0  # 误差
        self.last_speed = 0  # 上次误差
        self.last_speed2 = 0
        self.out = 0  # 输出值

    def motor_control(self, aim_speed, speed):
        self.speed_err = aim_speed - speed  # 计算误差

        # 比例项
        P = self.kp_motor * (self.speed_err - self.last_speed)
        # 积分项
        I = self.ki_motor * self.speed_err
        # 微分项
        D = self.kd_motor * (self.speed_err - 2 * self.last_speed + self.last_speed2)

        # 进行增量式PID运算
        self.out += P + I

        # 保存上次误差
        self.last_speed = self.speed_err
        self.last_speed2 = self.last_speed

        # 限幅占空比
        if self.out > 4500:
            self.out = 4500
        elif self.out < -4500:
            self.out = -4500

        return int(self.out)

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

def ccd_image(ccd_data,value):
    side_flag=0  #边界标识 0代表尚未查到边界，1代表查到左边界
    for i in range(0,122):
        sar=abs(ccd_data[i]-ccd_data[i+5])*100/(ccd_data[i]+ccd_data[i+5])
        if sar>value and side_flag==0:     #如果差和比大于阈值，则记录为边界
            leftside=i+5
            side_flag=1
        if sar>value and side_flag==1:
            rightside=i+5
    return leftside,rightside


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


#获得动态阈值
def ccd_get_threshold(ccd_data):
    value_max = ccd_data[4]          # 从第5个元素开始考虑最大值
    value_min = ccd_data[4]          # 从第5个元素开始考虑最小值
    
    #遍历5-122
    for i in range(5, 123):
        value_max = max(value_max, ccd_data[i])   # 限幅在最大传入数据和第5个元素值上
        value_min = min(value_min, ccd_data[i])   # 限幅在最小传入数据和第5个元素值上
        
    threshold = (value_max + value_min) / 2      # 计算阈值
    threshold = min(max(75, threshold), 255)     # 阈值限幅在75-256之间
    return threshold
 

#ccd滤波
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
    if (n == 2):
        left_point_2 = right_point_2 - 55
    if (N == 2):
        right_point_2 = left_point_2 + 68
    if (n == 4):
        left_point_2 = right_point_2 - 55
    
        
    # ccd2右圆环补线
    if (m == 2):
        right_point_2 = left_point_2 + 55
    if (M == 2):
        left_point_2 = right_point_2 - 68
    if (m == 4):
        right_point_2 = left_point_2 + 55
        
   
    # 计算中点
    mid_point_2 = (left_point_2 + right_point_2) / 2  
    # 以这次中点作为下次的上次中点  
    last_mid_point_2 = int(mid_point_2)
    # 返回当前中点  
    return mid_point_2
 
    
    
speed_pid_l = motor_PID(kp_motor=10.0, ki_motor=0.6, kd_motor=0)  # 左电机PID初始化
speed_pid_r = motor_PID(kp_motor=10.0, ki_motor=0.6, kd_motor=0)
while True:

    # 计算路程
    distance_T += (encl_data + encr_data) / 2 / 1024 * 30 / 50 * 0.05 * 3.1415926
    if(distance_T >= 27) :
        T = 0      # 停车速度给0
    else:
        T = 500    # 速度给500

    # 拨码开关关中断
    if end_switch.value() == 0:
        pit1.stop()    # pit1关闭
        pit2.stop()    # pit2关闭
        pit3.stop()    # pit3关闭
        break          # 跳出判断

    #速度控制
    aim_speed_l = T
    aim_speed_r = T #原代码这里有对舵机的方向再来输出，咱现在不需要

    # 编码器卡尔曼滤波
    output_encl = kalman_filter(kfp_var_l, encl_data)
    output_encr = kalman_filter(kfp_var_r, encr_data)
            
    # 电机PID计算
    out_l = speed_pid_l.motor_control(aim_speed = aim_speed_l, speed = output_encl)
    out_r = speed_pid_r.motor_control(aim_speed = aim_speed_r, speed = output_encr)
            
    # 电机占空比
    motor_l.duty(out_r)
    motor_r.duty(out_l)




    # 3ms中断标志位
    if(ticker_flag_3ms):
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
    if(ticker_flag_1000ms):
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
        
        # # 舵机PD控制
        # out = servo_pid.Direction_pid(error2) + angle  
        # # 获取舵机角度对应占空比
        # duty = int(duty_angle(pwm_servo_hz, out))
        # pwm_servo.duty_u16(duty)
        
        
        # # 左圆环处理
        # if (flag is '左圆环1') and n == 0 and (distance_T >= 1.3):
        #     n = 1
        # if (flag is '左圆环2') and n == 1:
        #     n = 2
        #     N = 1
        # if (n == 2):
        #     flag is not  '左圆环1'
        #     # 计算第一个路口距离(距离是固定的)
        #     distance_lh_1 += (encl_data + encr_data) / 2 / 1024 * 30 / 50 * 0.05 * 3.1415926
        #     if(distance_l_1 >= 0.28 ):
        #         n = 3
        #         N = 2
        #         distance_l_1 = 0
        # if (n == 3):
        #     flag is not  '左圆环1'
        #     flag is not  '左圆环2'
        # if (N == 2):
        #     #环内陀螺仪积分
        #     grzo_z += imu_data[5]  * 0.002 / 14.3
        #     if (grzo_z >= 57):
        #         N = 3
        #         n = 4
        # if (n == 4):
        #     grzo_z = 0
        #     # 计算补线距离仍是一个路口距离
        #     distance_l_2 += (encl_data + encr_data) / 2 / 1024 * 30 / 50 * 0.05 * 3.1415926
        #     if(distance_l_2 >= 0.35):
        #         n = 5
        #         N = 0
        #         distance_l_2 = 0


