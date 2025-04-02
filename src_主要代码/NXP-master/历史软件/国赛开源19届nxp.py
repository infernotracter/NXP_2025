#第十九届智能车竞赛NXP组
#作者：可达鸭
#2024年8月23日
#本代码免费开源给大家，严禁出售！



# 基础库、NXP库、第三方库
from machine import *
from display import *
from smartcar import *
from seekfree import *
from math import *
import gc
import time
import os
import io



# 实例化：
# 实例化 WIRELESS_UART 模块 参数是波特率
wireless = WIRELESS_UART(115200)

# 蜂鸣器初始化
buzzer = Pin('C9', Pin.OUT, pull = Pin.PULL_UP_47K, value = 0)

# 发车初始化
Go = Pin('C21', Pin.IN, pull = Pin.PULL_UP_47K, value = 0)

# 实例化 SERVO 舵机驱动模块
pwm_servo_hz = 300
angle = 94.0
# 定义一个角度与占空比换算的函数
def duty_angle (freq, Servo_Center):
    return (65535.0 / (1000.0 / freq) * (0.5 + Servo_Center / 90.0))
duty = int(duty_angle(pwm_servo_hz, angle))
pwm_servo = PWM("C20", pwm_servo_hz, duty_u16 = duty)

# 实例化 MOTOR_CONTROLLER 电机驱动模块
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty = 0, invert = True)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty = 0, invert = True)

# 实例化 encoder 模块
encoder_l = encoder("D0", "D1", True)
encoder_r = encoder("D2", "D3")

# 实例化 bldc 模块
bldc1 = BLDC_CONTROLLER(BLDC_CONTROLLER.PWM_B26, freq = 300, highlevel_us = 1100)
bldc2 = BLDC_CONTROLLER(BLDC_CONTROLLER.PWM_B27, freq = 300, highlevel_us = 1100)
high_level_us = 1300
dir = 1

# 实例化 lcd 模块
cs = Pin('C5', Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
cs.high()
cs.low()
rst = Pin('B9', Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
dc  = Pin('B8', Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
blk = Pin('C4', Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
drv = LCD_Drv(SPI_INDEX = 1, BAUDRATE = 60000000, DC_PIN = dc, RST_PIN = rst, LCD_TYPE = LCD_Drv.LCD200_TYPE)
lcd = LCD(drv)
lcd.color(0xFFFF, 0x0000)
lcd.mode(1)
lcd.clear(0x0000)

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
pit1.capture_list(ccd, encoder_l, encoder_r)
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
pit2.start(1)   

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
ccd_data1 = [0] * 128         # ccd1原始数组
ccd_data2 = [0] * 128         # ccd2原始数组
encl_data = 0                 # 左编码器数据
encr_data = 0                 # 右数据编码器
key_data = [0] * 4            # 按键数据
threshold1 = 0                # ccd1阈值
threshold2 = 0                # ccd2阈值
image_value1 = [0] * 128      # ccd1的二值化数组
image_value2 = [0] * 128      # ccd2的二值化数组
Mid_point1 = 0                # ccd1的中点
Mid_point2 = 0                # ccd2的中点
error1 = 0                    # ccd1的误差
error2 = 0                    # ccd2的误差
out = 0                       # 舵机输出值
output_encl = 0               # 左轮编码器滤波
output_encr = 0               # 右轮编码器滤波
out_l = 0                     # 左轮输出值
out_r = 0                     # 右轮输出值

data_change_flag = 0          # 数据修改标志位
main_menu_item = 1            # 主菜单选择第几行
sec_menu_item = 2             # 二级菜单选择的第几行


n = N = 0            # 左圆环标志
m = M = 0            # 右圆环标志
distance_l_1 = 0     # 左圆环路程标志
distance_l_2 = 0     # 左圆环路程标志
distance_r_1 = 0     # 右圆环路程标志
distance_r_2 = 0     # 右圆环路程标志
distance_T = 0       # 总路程标志
grzo_z = 0           # 陀螺仪z轴值
grzo_z1 = 0 
distance_a = 0
imu_data = [0] * 6


#第十九届智能车竞赛NXP组
#作者：可达鸭
#2024年8月23日
#本代码免费开源给大家，严禁出售！


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
        right_point_2 = left_point_2 + 72
    if (n == 4):
        left_point_2 = right_point_2 - 55
    
        
    # ccd2右圆环补线
    if (m == 2):
        right_point_2 = left_point_2 + 55
    if (M == 2):
        left_point_2 = right_point_2 - 72
    if (m == 4):
        right_point_2 = left_point_2 + 55
        
   
    # 计算中点
    mid_point_2 = (left_point_2 + right_point_2) / 2  
    # 以这次中点作为下次的上次中点  
    last_mid_point_2 = int(mid_point_2)
    # 返回当前中点  
    return mid_point_2
 
    
# 偏差计算
def get_offset(mid_point):  
    # 计算误差并返回结果
    offset = 60.0 - mid_point
    return offset  


# 舵机位置式PID
class turn_PID:  
    def __init__(self, kp_turn, kd_turn):  
        self.kp_turn = kp_turn     # kp系数
        self.kd_turn = kd_turn     # kd系数
        self.err = 0               # 误差
        self.last_err = 0          # 上次误差
        self.out = 0               # 输出值
    def Direction_pid(self, err):
        #比例项
        P = err * self.kp_turn
        #微分项
        D = self.kd_turn * (err - self.last_err)
        
        # 使用公式计算转角值
        self.out = P + D

        #更新err
        self.last_err = err
    
        # 限幅在13.0°-10.5°之间
        if self.out >= 10.5:
            self.out = 10.5
        elif self.out <= -13.0:
            self.out = -13.0
        
        return self.out  


# 电机增量式PID
class motor_PID:
    def __init__(self, kp_motor, ki_motor, kd_motor):
        self.kp_motor = kp_motor   # kp系数
        self.ki_motor = ki_motor   # ki系数
        self.kd_motor = kd_motor   # ki系数
        self.aim_speed = 0         # 期望速度
        self.speed = 0             # 实际速度
        self.speed_err = 0         # 误差
        self.last_speed = 0        # 上次误差
        self.last_speed2 = 0
        self.out = 0               # 输出值
    def motor_control(self, aim_speed, speed):
        self.speed_err = aim_speed - speed    # 计算误差
        
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
        if self.out > 4100:
            self.out = 4100
        elif self.out < -4100:
            self.out = -4100
            
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



# 全局变量赛道类型和阶段
road_type = {
    '左圆环1' : '左圆环1',
    '左圆环2' : '左圆环2',
    '右圆环1' : '右圆环1',
    '右圆环2' : '右圆环2',
    '斑马线' : '斑马线',
}
flag = None
zebra = 0
# 元素识别
def search_element():
    global flag, zebra
    '''
    # 斑马线
    for i in range(54, len(ccd_data2) - 54):
        if (abs(ccd_data2[i] - ccd_data2[i + 2]) >= 50):
            zebra += 1
    if zebra >= 8:
        flag = road_type['斑马线']
    else:
        zebra = 0
    '''

    
    # 左圆环
    if (30 <= left_point_2 <= 42 ) and (90 <= right_point_2 <= 108):      # 判断近端ccd左右无丢线
        if (left_point_1 < 7) and (88 <= right_point_1 <= 101):          # 判断远端ccd左丢线
            if (abs(right_point_1 - right_point_2 ) <= 10):              # 判断两个ccd不丢线一端是否变化
                flag = road_type['左圆环1']                               # 标志位1
    #判断两个ccd不丢线一端是否变化,且近端ccd左丢线
    if (abs(right_point_1 - right_point_2 ) <= 12) and (left_point_2 <= 10) and (87 <= right_point_2 <= 103):
        flag = road_type['左圆环2']                                       # 标志位2
     
    
    
    # 右圆环
    if (30 <= left_point_2 <= 44) and (90 <= right_point_2 <= 108):      # 判断近端ccd左右无丢线
        if (19 <= left_point_1 <= 36) and (116 <= right_point_1):        # 判断远端ccd右丢线
            if (abs(left_point_1 - left_point_2 ) <= 12):                # 判断两个ccd不丢线一端是否变化
                flag = road_type['右圆环1']                               # 标志位1
    #判断两个ccd不丢线一端是否变化,且近端ccd右丢线
    if (abs(left_point_1 - left_point_2) <= 12) and (115 <= right_point_2) and (31 <= left_point_2 <= 44):
        flag = road_type['右圆环2']                                       # 标志位2
    
    
    return flag
 


# UI总显示选择总程序
def menu():
    if (main_menu_item == 1):       # 菜单指针1
        main_menu()                 # 主菜单
    if (main_menu_item == 2):       # 菜单指针2
        Sec_Menu_01()               # 进入发车相关选择
    if (main_menu_item == 3):       # 菜单指针3
        Sec_Menu_02()               # 速度调控选择
    if (main_menu_item == 4):       # 菜单指针4
        Sec_Menu_03()               # 元素选择
    if (main_menu_item == 5):       # 菜单指针5
        Sec_Menu_04()               # 舵机PD系数
    if (main_menu_item == 6):       # 菜单指针6
        Sec_Menu_05()               # 电机PI系数
    if (main_menu_item == 7):       # 菜单指针7
        Sec_Menu_06()               # ccd图像显示
    if (main_menu_item == 8):       # 菜单指针8
        Sec_Menu_07()               # 常用参数
    if (main_menu_item == 9):       # 菜单指针9
        Sec_Menu_08()               # 屏幕一键关闭
    if (main_menu_item == 10):      # 菜单指针10
        Sec_Menu_09()               # 屏幕一键保存



# 主菜单显示
def main_menu():
    global sec_menu_item, main_menu_item,car_run
    lcd.str24(60, 0, "main_menu", 0x07E0)      # 主菜单标题
    lcd.str16(16, 30, "car_go", 0xFFFF)        # 进入发车相关选择
    lcd.str16(16, 46, "speed", 0xFFFF)         # 速度调控选择
    lcd.str16(16, 62, "element", 0xFFFF)       # 元素选择
    lcd.str16(16, 78, "turn_pd", 0xFFFF)       # 舵机PD系数
    lcd.str16(16, 94, "motor_pi", 0xFFFF)      # 电机PI系数
    lcd.str16(16, 110, "ccd_image", 0xFFFF)    # ccd图像显示
    lcd.str16(16, 126, "parameter", 0xFFFF)    # 常用参数
    lcd.str16(16, 142, "screen_off", 0xFFFF)   # 屏幕一键关闭
    lcd.str16(16, 158, "save_para", 0xFFFF)    # 屏幕一键保存
    
    if (data_change_flag == 0):
        lcd.str12(0, sec_menu_item * 16, ">", 0xF800)   # 光标 “>”
        if key_data[0]:
            lcd.clear(0x0000)     # 清屏
            key.clear(1)
        if key_data[1]:
            lcd.clear(0x0000)     # 清屏
            key.clear(2)
        if key_data[2]:
            lcd.clear(0x0000)     # 清屏
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)     # 清屏
            key.clear(4)
    elif (data_change_flag == 1):
        lcd.str12(0, sec_menu_item * 16, "*", 0xF800)   # 光标 “*”
        lcd.clear(0x0000)     # 清屏
        
    # 向上
    if key_data[0]:
        sec_menu_item -= 1    # 指针减少
        key.clear(1)
        
    # 向下
    if key_data[1]:
        sec_menu_item += 1    # 指针增加
        key.clear(2)
      
    # 返回一级菜单
    if(sec_menu_item == 2 and Go.value() == 0):
        car_run += 1
    if(sec_menu_item == 3 and Go.value() == 0):
        main_menu_item = 3
        sec_menu_item = 2
        lcd.clear(0x0000)     # 清屏
    if(sec_menu_item == 4 and Go.value() == 0):
        main_menu_item = 4
        sec_menu_item = 2
        lcd.clear(0x0000)     # 清屏
    if(sec_menu_item == 5 and Go.value() == 0):
        main_menu_item = 5
        sec_menu_item = 2
        lcd.clear(0x0000)     # 清屏
    if(sec_menu_item == 6 and Go.value() == 0):
        main_menu_item = 6
        sec_menu_item = 2
        lcd.clear(0x0000)     # 清屏
    if(sec_menu_item == 7 and Go.value() == 0):
        main_menu_item = 7
        sec_menu_item = 2
        lcd.clear(0x0000)     # 清屏
    if(sec_menu_item == 8 and Go.value() == 0):
        main_menu_item = 8
        sec_menu_item = 2
        lcd.clear(0x0000)     # 清屏
        
    gc.collect()


# 发车相关选择
def Sec_Menu_01():
    lcd.str24(60, 0, "car_go", 0x07E0)        # 二级菜单标题
    lcd.str16(16, 30, "return", 0xFFFF)        # 返回主菜单
    
    lcd.str16(16, 46, "GO", 0xFFFF)            # 发车

    if (data_change_flag == 0):
        lcd.str12(0, sec_menu_item * 16, ">", 0xF800)   # 光标 “>”
    elif (data_change_flag == 1):
        lcd.str12(0, sec_menu_item * 16, "*", 0xF800)   # 光标 “*”
        
    gc.collect()


# 速度调控选择
def Sec_Menu_02():
    global sec_menu_item, aim_speed_l, aim_speed_r, main_menu_item
    lcd.str24(60, 0, "speed", 0x07E0)         # 二级菜单标题
    lcd.str16(16, 96, "return", 0xFFFF)        # 返回主菜单
    
    lcd.str16(16, 30, "aim_speed_l = {}".format(aim_speed_l), 0xFFFF)    # 左轮目标速度
    lcd.str16(16, 62, "aim_speed_r = {}".format(aim_speed_r), 0xFFFF)    # 右轮目标速度
    
    if (data_change_flag == 0):
        lcd.str12(0, sec_menu_item * 16, ">", 0xF800)   # 光标 “>”
        if key_data[0]:
            lcd.clear(0x0000)     # 清屏
            key.clear(1)
        if key_data[1]:
            lcd.clear(0x0000)     # 清屏
            key.clear(2)
        if key_data[2]:
            lcd.clear(0x0000)     # 清屏
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)     # 清屏
            key.clear(4)
    elif (data_change_flag == 1):
        lcd.str12(0, sec_menu_item * 16, "*", 0xF800)   # 光标 “*”
        lcd.clear(0x0000)     # 清屏
    
    # 向上
    if key_data[0]:
        sec_menu_item -= 2
        key.clear(1)
        
    # 向下
    if key_data[1]:
        sec_menu_item += 2
        key.clear(2)
    
    # 限幅指针
    if (sec_menu_item < 2):
        sec_menu_item = 2
    if (sec_menu_item > 6):
        sec_menu_item = 6
    
    # 按键调参
    if (sec_menu_item == 4):
        if key_data[3]:
            aim_speed_l += 5
        if key_data[2]:
            aim_speed_l -= 5
    if (sec_menu_item == 6):
        if key_data[3]:
            aim_speed_r += 5
        if key_data[2]:
            aim_speed_r -= 5
            
    # 返回一级菜单        
    if(sec_menu_item == 6 and Go.value() == 0):
        main_menu_item = 1
        sec_menu_item = 2
        lcd.clear(0x0000)     # 清屏
        
    gc.collect()
    
    
# 元素选择
def Sec_Menu_03():
    lcd.str24(60, 0, "element", 0x07E0)      # 二级菜单标题
    lcd.str16(16, 30, "return", 0xFFFF)      # 返回主菜单

    if (flag is '左圆环1'):
        lcd.str16(16, 110, 'left_round1', 0xFFFF)    # 左圆环1
    elif (flag is '左圆环2'):
        lcd.str16(16, 110, 'left_round1', 0xFFFF)    # 左圆环2
    elif (flag is '右圆环1'):
        lcd.str16(16, 110, 'right_round1', 0xFFFF)   # 右圆环1
    elif (flag is '右圆环2'):
        lcd.str16(16, 126, 'right_round2', 0xFFFF)   # 右圆环2
    elif (flag is '斑马线'):
        lcd.str16(16, 142, 'zebra', 0xFFFF)         # 斑马线

    gc.collect()


# 舵机PD系数
def Sec_Menu_04():
    global sec_menu_item, main_menu_item
    lcd.str24(60, 0, "turn_pd", 0x07E0)      # 二级菜单标题
    lcd.str16(16, 222, "return", 0xFFFF)      # 返回主菜单

    lcd.str16(16, 62, "ccd1_kp_turn = {}".format(kp_turn[0]), 0xFFFF)    # 舵机kp_ccd1
    lcd.str16(16, 46, "ccd1 p + / -  0.01", 0x001F)
    lcd.str16(16, 110, "ccd1_kd_turn = {}".format(kd_turn[0]), 0xFFFF)    # 舵机kd_ccd1
    lcd.str16(16, 94, "ccd1 d + / -  0.01", 0x001F)
    lcd.str16(16, 158, "ccd2_kp_turn = {}".format(kp_turn[1]), 0xFFFF)    # 舵机kp_ccd2
    lcd.str16(16, 142, "ccd2 p + / - 0.01", 0x001F)
    lcd.str16(16, 206, "ccd2_kd_turn = {}".format(kd_turn[1]), 0xFFFF)    # 舵机kd_ccd2
    lcd.str16(16, 190, "ccd2 d + / - 0.01", 0x001F)
    
    if (data_change_flag == 0):
        lcd.str12(0, sec_menu_item * 16, ">", 0xF800)   # 光标 “>”
        if key_data[0]:
            lcd.clear(0x0000)     # 清屏
            key.clear(1)
        if key_data[1]:
            lcd.clear(0x0000)     # 清屏
            key.clear(2)
        if key_data[2]:
            lcd.clear(0x0000)     # 清屏
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)     # 清屏
            key.clear(4)
    elif (data_change_flag == 1):
        lcd.str12(0, sec_menu_item * 16, "*", 0xF800)   # 光标 “*”
        lcd.clear(0x0000)     # 清屏
    
    # 向上
    if key_data[0]:
        sec_menu_item -= 1
        key.clear(1)
        
    # 向下
    if key_data[1]:
        sec_menu_item += 1
        key.clear(2)
    
    # 指针限幅
    if (sec_menu_item < 2):
        sec_menu_item = 2
    if (sec_menu_item > 14):
        sec_menu_item = 14
    
    # 按键调参
    if (sec_menu_item == 3):
        if key_data[3]:
            kp_turn[0] += 0.01
        if key_data[2]:
            kp_turn[0] -= 0.01
    if (sec_menu_item == 4):
        if key_data[3]:
            kp_turn[0] += 0.1
        if key_data[2]:
            kp_turn[0] -= 0.1
    if (sec_menu_item == 6):
        if key_data[3]:
            kd_turn[0] += 0.01
        if key_data[2]:
            kd_turn[0] -= 0.01
    if (sec_menu_item == 7):
        if key_data[3]:
            kd_turn[0] += 0.1
        if key_data[2]:
            kd_turn[0] -= 0.1
    if (sec_menu_item == 9):
        if key_data[3]:
            kp_turn[1] += 0.01
        if key_data[2]:
            kp_turn[1] -= 0.01
    if (sec_menu_item == 10):
        if key_data[3]:
            kp_turn[1] += 0.1
        if key_data[2]:
            kp_turn[1] -= 0.1
    if (sec_menu_item == 12):
        if key_data[3]:
            kd_turn[1] += 0.01
        if key_data[2]:
            kd_turn[1] -= 0.01
    if (sec_menu_item == 13):
        if key_data[3]:
            kd_turn[1] += 0.1
        if key_data[2]:
            kd_turn[1] -= 0.1
            
    # 返回一级菜单
    if(sec_menu_item == 14 and Go.value() == 0):
        main_menu_item = 1
        sec_menu_item = 2
        lcd.clear(0x0000)     # 清屏
        
    gc.collect()


# 电机PI系数
def Sec_Menu_05():
    global sec_menu_item, kp_motor, ki_motor, kd_motor, main_menu_item
    lcd.str24(60, 0, "motor_pi", 0x07E0)      # 二级菜单标题
    lcd.str16(16, 158, "return", 0xFFFF)       # 返回主菜单

    lcd.str16(16, 62, "kp_motor = {}".format(kp_motor), 0xFFFF)    # 电机kp
    lcd.str16(16, 46, "p + / -  0.01", 0x001F)
    lcd.str16(16, 110, "ki_motor = {}".format(ki_motor), 0xFFFF)    # 电机ki
    lcd.str16(16, 94, "i + / -  0.01", 0x001F)
    lcd.str16(16, 142, "kd_motor = {}".format(kd_motor), 0xFFFF)    # 电机ki
    lcd.str16(16, 126, "i + / -  0.01", 0x001F)
    
    if (data_change_flag == 0):
        lcd.str12(0, sec_menu_item * 16, ">", 0xF800)   # 光标 “>”
        if key_data[0]:
            lcd.clear(0x0000)     # 清屏
            key.clear(1)
        if key_data[1]:
            lcd.clear(0x0000)     # 清屏
            key.clear(2)
        if key_data[2]:
            lcd.clear(0x0000)     # 清屏
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)     # 清屏
            key.clear(4)
    elif (data_change_flag == 1):
        lcd.str12(0, sec_menu_item * 16, "*", 0xF800)   # 光标 “*”
        lcd.clear(0x0000)     # 清屏
    
    # 向上
    if key_data[0]:
        sec_menu_item -= 1
        key.clear(1)
        
    # 向下
    if key_data[1]:
        sec_menu_item += 1
        key.clear(2)
    
    # 指针限幅
    if (sec_menu_item < 2):
        sec_menu_item = 2
    if (sec_menu_item > 10):
        sec_menu_item = 10
    
    # 按键调参
    if (sec_menu_item == 3):
        if key_data[3]:
            kp_motor += 0.01
        if key_data[2]:
            kp_motor -= 0.01
    if (sec_menu_item == 4):
        if key_data[3]:
            kp_motor += 0.1
        if key_data[2]:
            kp_motor -= 0.1
    if (sec_menu_item == 6):
        if key_data[3]:
            ki_motor += 0.01
        if key_data[2]:
            ki_motor -= 0.01
    if (sec_menu_item == 7):
        if key_data[3]:
            ki_motor += 0.1
        if key_data[2]:
            ki_motor -= 0.1
    
    # 返回一级菜单
    if(sec_menu_item == 10 and Go.value() == 0):
        main_menu_item = 1
        sec_menu_item = 2
        lcd.clear(0x0000)     # 清屏
        
    gc.collect()


# ccd图像显示
def Sec_Menu_06():
    lcd.str24(60, 0, "ccd_image", 0x07E0)     # 二级菜单标题
    lcd.str16(16, 30, "return", 0xFFFF)       # 返回主菜单
    
    lcd.wave(0,  64, 128, 64, ccd_data1)     # ccd1图像
    lcd.wave(0, 128, 128, 64, ccd_data2)     # ccd2图像
    lcd.line(64, 64, 64, 192, color = 0x001F, thick = 1)    # 实际中线
    
    if (data_change_flag == 0):
        lcd.str12(0, sec_menu_item * 16, ">", 0xF800)   # 光标 “>”
        if key_data[0]:
            lcd.clear(0x0000)     # 清屏
            key.clear(1)
        if key_data[1]:
            lcd.clear(0x0000)     # 清屏
            key.clear(2)
        if key_data[2]:
            lcd.clear(0x0000)     # 清屏
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)     # 清屏
            key.clear(4)
    elif (data_change_flag == 1):
        lcd.str12(0, sec_menu_item * 16, "*", 0xF800)   # 光标 “*”
        lcd.clear(0x0000)     # 清屏
        
    gc.collect() 
 
 
# 常用参数
def Sec_Menu_07():
    global sec_menu_item
    lcd.str24(60, 0, "parameter", 0x07E0)    # 二级菜单标题
    lcd.str16(16, 30, "return", 0xFFFF)       # 返回主菜单
    
    lcd.str16(16, 46, "tof = {:<4d}".format(tof_data), 0xFFFF)                            # TOF数据
    lcd.str16(16, 62, "out_l = {:<4d}".format(out_l), 0xFFFF)                             # 左环pid输出
    lcd.str16(16, 78, "out_r = {:<4d}".format(out_r), 0xFFFF)                             # 右环pid输出
    lcd.str16(16, 94, "encl = {:<4d}".format(encl_data), 0xFFFF)                              # 左编码器值
    lcd.str16(16, 110, "encr = {:<4d}".format(encr_data), 0xFFFF)                             # 右编码器值
    lcd.str16(16, 126, "Mid_point1 = {:<.2f}".format(Mid_point1), 0xFFFF)                 # ccd1中点
    lcd.str16(16, 142, "Mid_point2 = {:<.2f}".format(Mid_point2), 0xFFFF)                 # ccd2中点
    lcd.str16(16, 158, "error1 = {:<.2f}".format(error1), 0xFFFF)                         # ccd1误差
    lcd.str16(16, 174, "error2 = {:<.2f}".format(error2), 0xFFFF)                         # ccd2误差
    lcd.str16(16, 190, "left_point_1 = {:<3d}".format(left_point_1), 0xFFFF)              # 上摄像头左边点
    lcd.str16(16, 206, "right_point_1 = {:<3d}".format(right_point_1), 0xFFFF)            # 上摄像头右边点
    lcd.str16(16, 222, "left_point_2 = {:<3d}".format(left_point_2), 0xFFFF)              # 下摄像头左边点
    lcd.str16(16, 238, "right_point_2 = {:<3d}".format(right_point_2), 0xFFFF)            # 下摄像头右边点
    lcd.str16(16, 254, "width_1 = {:<3d}".format(right_point_1 - left_point_1), 0xFFFF)   # ccd1计算赛道宽度
    lcd.str16(16, 270, "width_2 = {:<3d}".format(right_point_2 - left_point_2), 0xFFFF)   # ccd2计算赛道宽度

    if (data_change_flag == 0):
        lcd.str12(0, sec_menu_item * 16, ">", 0xF800)   # 光标 “>”
        if key_data[0]:
            lcd.clear(0x0000)     # 清屏
            key.clear(1)
        if key_data[1]:
            lcd.clear(0x0000)     # 清屏
            key.clear(2)
        if key_data[2]:
            lcd.clear(0x0000)     # 清屏
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)     # 清屏
            key.clear(4)
    elif (data_change_flag == 1):
        lcd.str12(0, sec_menu_item * 16, "*", 0xF800)   # 光标 “*”
        lcd.clear(0x0000)     # 清屏
        
    gc.collect()
  
  
# 屏幕一键关闭
def Sec_Menu_08():
    lcd.clear(0x0000)      # 清屏
    return


# 屏幕一键保存
def Sec_Menu_09():
    write_flash()         # 写入缓冲区
    
    buzzer.value(1)       # 蜂鸣器开
    lcd.clear(0xF800)     # 清屏
    time.sleep_ms(100)    # 延时
    main_menu_item = 1    # 返回一级菜单
    buzzer.value(0)       # 蜂鸣器关
    

# 写入缓冲区
def write_flash():
    os.chdir("/flash")    # 切换到 /flash 目录
    try:
        # 通过 try 尝试打开文件 因为 r+ 读写模式不会新建文件
        user_file = io.open("user_data.txt", "r+")
    except:
        # 如果打开失败证明没有这个文件 所以使用 w+ 读写模式新建文件
        user_file = io.open("user_data.txt", "w+")

    # 将指针移动到文件头 0 偏移的位置
    user_file.seek(0, 0)
    # 使用 write 方法写入数据到缓冲区
    
    user_file.write("%.4f\n"%(kp_turn))
    user_file.write("%.4f\n"%(kd_turn))
    user_file.write("%.4f\n"%(kp_motor))
    user_file.write("%.4f\n"%(ki_motor))
    user_file.write("%d\n"%(aim_speed_l))
    user_file.write("%d\n"%(aim_speed_r))
    
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
    
# 位置式PID
class PID:
    def __init__(self, kp, ki, kd):  
        self.kp = kp     # kp系数
        self.ki = ki     # ki系数
        self.kd = kd     # kd系数
        self.err = 0               # 误差
        self.last_err = 0          # 上次误差
        self.integral = 0             # 积分限幅
        self.out = 0               # 输出值
        self.limit = 0
    def use_pid(self, set_value, actual_value, limit):
        # 误差计算
        self.err = set_value - actual_value
        # 积分
        self.integral += self.err
        # 积分限幅
        if (self.integral >= limit):
            self.integral = limit
        if (self.integral <= -limit):
            self.integral = -limit
        # 比例项
        P = self.err * self.kp
        # 积分项
        I = self.integral * self.ki
        # 微分项
        D = self.kd * (self.err - self.last_err)
        # 使用公式计算转角值
        self.out = P + I + D
        #更新err
        self.last_err = self.err
        
        # 限幅在13.0°-10.5°之间
        if self.out >= 10.5:
            self.out = 10.5
        elif self.out <= -13.0:
            self.out = -13.0
            
        return self.out  
    

#第十九届智能车竞赛NXP组
#作者：可达鸭
#2024年8月23日
#本代码免费开源给大家，严禁出售！

    
speed_pid_l = motor_PID(kp_motor = 10.0, ki_motor = 1, kd_motor = 0)    # 左电机PID初始化
speed_pid_r = motor_PID(kp_motor = 10.0, ki_motor = 1, kd_motor = 0)    # 右电机PID初始化
servo_pid = PID(kp = 0.4, ki = 0.002525*2, kd = 1)
s_p = turn_PID(kp_turn = 0.4, kd_turn = 1)
#servo_pid = turn_PID(kp_turn = 0.425, kd_turn = 0.5)                      # 舵机PID初始化0.002525*2
T = 500     # 总路程
while True:
    # 计算路程
    distance_T += (encl_data + encr_data) / 2 / 1024 * 30 / 50 * 0.05 * 3.1415926
     
    if(distance_T >= 3) :
        # 获取电机对应占空比 
        motor_l.duty(0)
        motor_r.duty(0)
                
    else:
        # 获取电机对应占空比 
        motor_l.duty(out_r)
        motor_r.duty(out_l)
                

    # 拨码开关关中断
    if end_switch.value() == 0:
        pit1.stop()    # pit1关闭
        pit2.stop()    # pit2关闭
        pit3.stop()    # pit3关闭
        break          # 跳出判断
    
    # 上位机显示
    wireless.send_oscilloscope(output_encl, output_encr, aim_speed_l, aim_speed_r)
        
    # 负压92.5%起转
    bldc1.highlevel_us(1925)
    bldc2.highlevel_us(1925)
    
    # 速度控制    
    if (error2 >= 10.0):
        aim_speed_l = T - (s_p.Direction_pid(error2) * 4)   # 差速：目标速度 +/- (舵机输出 * 差速系数)
        aim_speed_r = T + (s_p.Direction_pid(error2) * 4)   # 差速：目标速度 +/- (舵机输出 * 差速系数)
    if (-10.0 <= error2 <= 10.0):
        aim_speed_l = T     # 无差速状态
        aim_speed_r = T     # 无差速状态
    if (error2 <= -10.0):
        aim_speed_l = T - (s_p.Direction_pid(error2) * 4)   # 差速：目标速度 +/- (舵机输出 * 差速系数)
        aim_speed_r = T + (s_p.Direction_pid(error2) * 4)   # 差速：目标速度 +/- (舵机输出 * 差速系数)
        
    # 编码器卡尔曼滤波
    output_encl = kalman_filter(kfp_var_l, encl_data)
    output_encr = kalman_filter(kfp_var_r, encr_data)
            
    # 电机PID计算
    out_l = speed_pid_l.motor_control(aim_speed = aim_speed_l, speed = output_encl)
    out_r = speed_pid_r.motor_control(aim_speed = aim_speed_r, speed = output_encr)
            

    # 3ms中断标志位
    if(ticker_flag_3ms):
        menu()                           # 菜单显示
        lcd.wave(0,  256, 128, 64, ccd_data1, max = 255)
        lcd.wave(0, 64, 128, 64, ccd_data2, max = 255)
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
    
    

    
    # 10ms中断标志位
    if (ticker_flag):
        encl_data = encoder_l.get()     # 读取左编码器的数据
        encr_data = encoder_r.get()     # 读取右编码器的数据
        key_data = key.get()            # 读取按键的数据
        
        # 舵机PD控制
        #out = servo_pid.Direction_pid(error2) + angle
        out = servo_pid.use_pid(set_value = 64, actual_value = Mid_point2, limit = 2000) + angle
        # 获取舵机角度对应占空比
        duty = int(duty_angle(pwm_servo_hz, out))
        pwm_servo.duty_u16(duty)
        
        
        # 左圆环处理
        if (flag is '左圆环1') and n == 0:
            n = 1
        if (flag is '左圆环2') and n == 1:
            n = 2  # 补线
            N = 1
        if (n == 2):
            grzo_z1 += imu_data[5]  * 0.002 / 14.3
            distance_l_1 += (encl_data + encr_data) / 2 / 1024 * 30 / 50 * 0.05 * 3.1415926
            if( -0.8 < grzo_z1 < 0.8 ):        #防误判弯道
                if (distance_l_1 > 0.1):
                    grzo_z1 = 0
                    flag is not  '左圆环1'
                    buzzer.value(1)
                    n = 3
                    N = 2  #环补线
                    distance_l_1 = 0
            if(-0.8 > grzo_z1 ) or ( 0.8 < grzo_z1):
                n = 0
                N = 0
                grzo_z1= 0
                
        if (n == 3):
            flag is not  '左圆环1'
            flag is not  '左圆环2'
        if (N == 2):
            #环内陀螺仪积分
            grzo_z += imu_data[5]  * 0.002 / 14.3
            if (grzo_z >= 40):
                N = 3
                n = 4 #补线
        if (n == 4):
            grzo_z = 0
            # 计算补线距离仍是一个路口距离
            distance_l_2 += (encl_data + encr_data) / 2 / 1024 * 30 / 50 * 0.05 * 3.1415926
            if(distance_l_2 >= 0.15):
                n = 0
                N = 0
                distance_l_2 = 0
                grzo_z = 0
                grzo_z1 = 0
                distance_l_1 = 0
                
                
        # 右圆环处理
        if (flag is '右圆环1') and m == 0:
            m = 1
        if (flag is '右圆环2') and m == 1:
            m = 2  # 补线
            M = 1
        if (m == 2):
            grzo_z1 += imu_data[5]  * 0.002 / 14.3
            distance_l_1 += (encl_data + encr_data) / 2 / 1024 * 30 / 50 * 0.05 * 3.1415926
            if( -0.8 < grzo_z1 < 0.8 ):
                if (distance_l_1 > 0.1):
                    grzo_z1 = 0
                    flag is not  '右圆环1'
                    buzzer.value(1)
                    m = 3
                    M = 2  #环补线
                    distance_l_1 = 0
            if(-0.8 > grzo_z1 ) or ( 0.8 < grzo_z1):
                m = 0
                M = 0
                grzo_z1= 0
                
        if (m == 3):
            flag is not  '右圆环1'
            flag is not  '右圆环2'
        if (M == 2):
            #环内陀螺仪积分
            grzo_z += imu_data[5]  * 0.002 / 14.3
            if (grzo_z <= -40):
                M = 3
                m = 4 #补线
        if (m == 4):
            grzo_z = 0
            # 计算补线距离仍是一个路口距离
            distance_l_2 += (encl_data + encr_data) / 2 / 1024 * 30 / 50 * 0.05 * 3.1415926
            if(distance_l_2 >= 0.15):
                m = 0
                M = 0
                distance_l_2 = 0
                grzo_z = 0
                grzo_z1 = 0
                distance_l_1 = 0
                
            
        # 关中断标志位
        gc.collect()
        ticker_flag = False

    gc.collect()


#第十九届智能车竞赛NXP组
#作者：可达鸭
#2024年8月23日
#本代码免费开源给大家，严禁出售！