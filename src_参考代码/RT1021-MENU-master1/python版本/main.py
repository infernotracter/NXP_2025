from machine import *
from display import *
from smartcar import *
from seekfree import *
import math
import os
import io
import gc
import time
from menu import *
#####################################################################################################
###################################按键代码区域块2####################################################
####################################################################################################
image_show_flag = 0
value_show_flag = 0
def locate_data_read():##从本地txt中按顺序读出data##由于两个文件的变量可能不互通，所以在主循环这里再读一次txt文件的数据
    global speed_kp
    global speed_ki
    global DirInner_KP
    global DirInner_KD
    global set_speed
    global menu_speed
    global set_angle
    global EXP_TIME
    global image_show_flag
    global value_show_flag
    global DirOutter_KP
    global DirOutter_KP_EX
    global DirOutter_KD
    global CarInfo_CamMaxACC
    global allow_value_show
    global allow_image_show
    global send_ccdimg_flag
    global send_data_flag
    global recv_data_flag
    global runSpeedMode
    global StraightLSpeed
    global StraightSSpeed
    global StraightBreakSpeed
    global CurveSpeed
    global AnnulusSpeed
    global RampSpeed
    global ObstacleSpeed
    global runtime
    global delaytime
    global annual_delay_time
    global annual_out_offset
    global high_level_us
    global open_bldc_flag
    global allow_value_show
    global allow_image_show
    read_data = 512* [0]
    user_file = 'menu.txt'
    # 切换到 /flash 目录
    os.chdir("/flash")
    try:
    # 通过 try 尝试打开文件 因为 r+ 读写模式不会新建文件
        user_file = io.open("menu.txt", "r+")
    except:
    # 如果打开失败证明没有这个文件 所以使用 w+ 读写模式新建文件
        user_file = io.open("menu.txt", "r+")
    user_file.seek(0,0)
    # 读取三行数据 到临时变量 分别强制转换回各自类型
    for index in range(512):
        read_data[index] = int(user_file.readline())
    # 将数据重新输出 这就演示了如何保存数据和读取数据  
    speed_kp= read_data[0]
    speed_ki= read_data[1]
    DirInner_KP = read_data[4]
    DirInner_KD = read_data[5]
    set_speed = read_data[6]
    menu_speed = set_speed
    set_angle = 0
    EXP_TIME = read_data[7]
    DirOutter_KP = read_data[10]
    DirOutter_KP_EX = read_data[11]
    DirOutter_KD = read_data[12]
    CarInfo_CamMaxACC = read_data[13]
    send_ccdimg_flag = read_data[14]
    send_data_flag = read_data[15]
    recv_data_flag = read_data[16]
    runSpeedMode = read_data[17]
    StraightLSpeed = read_data[18]
    StraightSSpeed = read_data[19]
    StraightBreakSpeed = read_data[20]
    CurveSpeed =  read_data[21]
    AnnulusSpeed =  read_data[22]
    RampSpeed =  read_data[23]
    ObstacleSpeed =  read_data[24]
    ParkingSpeed =  read_data[25]
    runtime =  read_data[26]
    delaytime =  read_data[27]
    runtime = runtime + delaytime
    annual_delay_time = read_data[28]
    annual_out_offset = float(read_data[29]/100.00)
    high_level_us = read_data[30]
    open_bldc_flag = read_data[31]
    allow_value_show = read_data[32]
    if(allow_value_show):
        value_show_flag = read_data[9]
    else:
        value_show_flag = 0
    allow_image_show = read_data[33]
    if(allow_image_show):
        image_show_flag = read_data[8]
    else:
        image_show_flag = 0
    user_file.close()


def locate_menu():
    return adjust_menu()
  
#ADC采集
adc_in1 = ADC('B14')
adc_in2 = ADC('B15')
adc_in3 = ADC('B26')
adc_in4 = ADC('B27')
# # # # # # # # # # # # # # # 六轴初始化 # # # # # # # # # # # # # # #
imu = IMU660RA()
# # # # # # # # # # # # # # # 红外传感器初始化 # # # # # # # # # # # # # # #
Red_Distance_Adc = ADC('B15')
# # # # # # # # # # # # # # # 电调接口初始化 # # # # # # # # # # # # # # #
# bldc1 = BLDC_CONTROLLER(BLDC_CONTROLLER.PWM_B27, freq=100, highlevel_us = 1000)
# bldc_controller = Pin('C6', Pin.OUT, pull=Pin.PULL_UP_47K, value = 0)
# # # # # # # # # # # # # # # 无线串口 # # # # # # # # # # # # # # #
wireless = WIRELESS_UART(460800)#不用到先注释
global serialbuffer
serialbuffer = [0] * 271
# # # # # # # # # # # # # # # 无线串口 # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # 屏幕初始化# # # # # # # # # # # # # # # 
# 定义控制引脚
rst = Pin('B9' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc  = Pin('B8' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C4' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
drv = LCD_Drv(SPI_INDEX=1, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
lcd = LCD(drv)
lcd.color(0xFFFF, 0x0000)
lcd.mode(2)
lcd.clear(0x0000)
# # # # # # # # # # # # # # # 屏幕初始化# # # # # # # # # # # # # # # 
# # # # # # # # # # # # # # # 电机初始化# # # # # # # # # # # # # # # 
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty = 0, invert = False)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty = 0, invert = True)
motor_ldir = 0
motor_rdir = 0
OutSpeed = 0
encl_data = 0
encr_data = 0
all_road_len = 0
out = 0
Speed_OutPut = 0
# # # # # # # # # # # # # # # 编码器# # # # # # # # # # # # # # # 
encoder_r = encoder("D1", "D0",False)
encoder_l = encoder("D3", "D2",True)
encl_data = 0
encr_data = 0
allencl_data = 0#5603  1m/脉冲
allencr_data = 0#5594  wocao ge 1m/脉冲
speed_TrueL = 1
speed_TrueR = 1
# # # # # # # # # # # # # # # 编码器# # # # # # # # # # # # # # #

# # # # # # # # # # # # # # # CCD
# 调用 TSL1401 模块获取 CCD 实例# 参数是采集周期 调用多少次 capture 更新一次数据
ccd = TSL1401(EXP_TIME)

##从本地txt中按顺序读出data##由于两个文件的变量可能不互通，所以在主循环这里再读一次txt文件的数据
##############调参运行##########################################
paper_index = 0   ##变1量数量
value_index = 0   ##变量大小
pit3 = ticker(2)
pit3.capture_list()
pit3.callback(key_pit)
pit3.start(200)
locate_menu()
pit3.stop()
time.sleep_ms(5)
locate_data_read()
##############调参运行##########################################
#中断代码区域块1
system_time = 0
#获取系统时间
def system_time_get():
    global system_time
    system_time = system_time + 0.5 - 0.5
    if(system_time > 18446744073709551615):
        system_time = 0
    return system_time

# # # # # # # # # # # # # # # 定时中断
# 定义一个回调函数
#回调函数1
def time_pit_handler1(time):
    global ticker_flag1
    global ticker_count1
    global ccd_data1
    global ccd_data2
    global DirOutter_OutPut
    global ccd1_center
    global ccd2_center
    global ccd1_error
    global ccd2_
    global Red_Distance
    Red_Distance = get_the_distance()
    if ticker_flag1 == False:
        ccd_data1 = ccd.get(0)
        ccd_data2 = ccd.get(1)
        ticker_flag1 = True

ccd1_ave_threshold = 1000# 全局变量初始化
ccd2_ave_threshold = 1000
max_value1 = 0
max_value2 = 0
min_value1 = 0
min_value2 = 0
threshold1 = 0
threshold2 = 0
ccd_data1 = [0] * 128
ccd_data2 = [0] * 128
ccd_deal1 = [0] * 128
ccd_deal2 = [0] * 128
ccd1_mid = 63
ccd2_mid = 63
ccd_error = 0
ccd1_error = 0
ccd2_error = 0
ccd1_error_last = 0
ccd2_error_last = 0
ccd1_center = int(ccd1_mid)
ccd2_center = int(ccd2_mid)
ccd1_center_last = int(ccd1_mid)
ccd2_center_last = int(ccd2_mid)
ccd1_road_len = 50
ccd2_road_len = 60
ac_ccd1_road_len = int(ccd1_road_len)
ac_ccd2_road_len = int(ccd2_road_len)
ccd2_left_index = 0
ccd2_right_index = 0
ccd1_left_index = 0
ccd1_right_index = 0
ccd2_left_index_last = 0
ccd2_right_index_last = 0
ccd1_left_index_last = 0
ccd1_right_index_last = 0
ccd1_IllegalEdge = 6
ccd2_IllegalEdge = 6
ccd1_left_lose_flag = 0
ccd1_right_lose_flag = 0
ccd2_left_lose_flag = 0
ccd2_right_lose_flag = 0
annual_flag = 0# 圆环
annual_Detect_flag = 0
annual_state = 0
annual_dir = 0
annual_delay = 0
annual_yaw_angle = 0
annual_all_error1 = 0
annual_all_error2 = 0
annual_ave_error1 = 0
annual_ave_error2 = 0
annual_all_tips = 0
allow_annual_time = 0
ramp_state = 0# 坡道
ramp_flag = 0
allow_ramp_time = 0
parking_flag = 0# 停车
parking_state = 0
parking_tip1 = 0
parking_tip2 = 0
allow_parking_time = 0
ccd1_parking_difference = [0] * 31
ccd2_parking_difference = [0] * 31
def get_the_dealCCd(ccd_data1, ccd_data2):#ccd二值化
    global max_value1
    global max_value2
    global min_value1
    global min_value2
    global threshold1
    global threshold2
    global ccd1_ave_threshold
    global ccd2_ave_threshold
    global ccd1_parking_difference
    global ccd2_parking_difference
    ccd1_ave_threshold = 0
    ccd2_ave_threshold = 0
    for value in ccd_data1:# 找到ccd_deal1的最大值和最小值
        ccd1_ave_threshold = ccd1_ave_threshold + value
    ccd1_ave_threshold = ccd1_ave_threshold / 128
    for value in ccd_data2:# 找到ccd_deal2的最大值和最小值
        ccd2_ave_threshold = ccd2_ave_threshold + value
    ccd2_ave_threshold = ccd2_ave_threshold / 128
    ccd1_parking_difference = [ccd_data1[i] - ccd_data1[i-2] for i in range(36, 98, 2)]
    ccd2_parking_difference = [ccd_data2[i] - ccd_data2[i-2] for i in range(36, 98, 2)]
    return ccd_deal1, ccd_deal2

def find_the_ccd1Center(ccd_deal1,ccd1_center_last,ccd1_left_index,ccd1_right_index):#找到ccd1中心
    global ccd1_center
    global ac_ccd1_road_len
    global ccd1_road_len
    global ccd1_left_index_last
    global ccd1_right_index_last
    global ccd1_mid
    global ccd1_IllegalEdge
    global ccd1_left_lose_flag
    global ccd1_right_lose_flag
    global ccd1_ave_threshold
    global ccd_data1
    ccd1_center_last = ccd1_center
    ccd1_left_index_last = ccd1_left_index
    ccd1_right_index_last = ccd1_right_index
    ccd1_left_index = ccd1_center_last
    ccd1_right_index = ccd1_center_last
    while ccd1_left_index >= 0:
        if(ccd1_left_index >= 4):
            if(ccd_data1[ccd1_left_index] - ccd_data1[ccd1_left_index - 4] > ccd1_ave_threshold*0.055):
                break
            ccd_deal1[ccd1_left_index] = 64
        else:
            if(ccd_data1[ccd1_left_index] < ccd1_ave_threshold):
                break
        ccd1_left_index -= 1
    while ccd1_right_index <= 127:
        if(ccd1_right_index <= 123):
            if(ccd_data1[ccd1_right_index] - ccd_data1[ccd1_right_index + 4] > ccd1_ave_threshold*0.055):
                break
            ccd_deal1[ccd1_right_index] = 64
        else:
            if(ccd_data1[ccd1_right_index] < ccd1_ave_threshold):
                break
        ccd1_right_index += 1
    ac_ccd1_road_len = ccd1_right_index - ccd1_left_index
    #######################################丢边判断
    if(ac_ccd1_road_len - ccd1_road_len > 25):      
        if (ccd1_left_index <= 4*ccd1_IllegalEdge and ccd1_right_index >= 127 -  4*ccd1_IllegalEdge ):
            ccd1_center = ccd1_mid
    if(ac_ccd1_road_len - ccd1_road_len > 20):      
        if (ccd1_left_index <= 5*ccd1_IllegalEdge and ccd1_right_index >= 127 -  5*ccd1_IllegalEdge ):
            ccd1_center = ccd1_mid
    elif ac_ccd1_road_len - ccd1_road_len > 10:
        if (ccd1_left_index <= 5*ccd1_IllegalEdge):
            ccd1_center = ccd1_right_index - ccd1_road_len//2
        elif (ccd1_right_index >= 127 - 5*ccd1_IllegalEdge):
            ccd1_center = ccd1_left_index + ccd1_road_len //2
    else:
        ccd1_center = (ccd1_left_index + ccd1_right_index) // 2
    return ccd1_center,ccd1_center_last,ac_ccd1_road_len,ccd1_left_index,ccd1_right_index,ccd1_left_index_last,ccd1_right_index_last,ac_ccd1_road_len

def find_the_ccd2Center(ccd_deal2,ccd2_center_last,ccd2_left_index,ccd2_right_index):#找到ccd2中心
    global ccd2_road_len
    global ac_ccd2_road_len
    global ccd2_center
    global ccd2_left_index_last
    global ccd2_right_index_last
    global ccd2_mid
    global ccd2_IllegalEdge
    global ccd2_left_lose_flag
    global ccd2_right_lose_flag
    global ccd2_ave_threshold
    global ccd_data2
    ccd2_left_lose_flag = 0
    ccd2_right_lose_flag = 0
    ccd2_center_last = ccd2_center
    ccd2_left_index_last = ccd2_left_index
    ccd2_right_index_last = ccd2_right_index
    ccd2_left_index = ccd2_center_last
    ccd2_right_index = ccd2_center_last
    while ccd2_left_index >= 0:
        if(ccd2_left_index >= 4):
            if(ccd_data2[ccd2_left_index] - ccd_data2[ccd2_left_index - 4] > ccd2_ave_threshold*0.055):
                break
            ccd_deal2[ccd2_left_index] = 64
        else:
            if(ccd_data2[ccd2_left_index] < ccd2_ave_threshold):
                break
        ccd2_left_index -= 1
    while ccd2_right_index <= 127:
        if(ccd2_right_index <= 123):
            if(ccd_data2[ccd2_right_index] - ccd_data2[ccd2_right_index + 4] > ccd2_ave_threshold*0.055):
                break
            ccd_deal2[ccd2_right_index] = 64
        else:
            if(ccd_data2[ccd2_right_index] < ccd2_ave_threshold):
                break
        ccd2_right_index += 1
    ac_ccd2_road_len = ccd2_right_index - ccd2_left_index
    if(ac_ccd2_road_len - ccd2_road_len > 25):#丢边判断
        if (ccd2_left_index <= 4*ccd2_IllegalEdge and ccd2_right_index >= 127 -  4*ccd2_IllegalEdge ):
            ccd2_center = ccd2_mid
            ccd2_left_lose_flag = 1
            ccd2_right_lose_flag = 1
    if(ac_ccd2_road_len - ccd2_road_len > 20):      
        if (ccd2_left_index <= 5*ccd2_IllegalEdge and ccd2_right_index >= 127 -  5*ccd2_IllegalEdge ):
            ccd2_center = ccd2_mid
    elif ac_ccd2_road_len - ccd2_road_len> 10:
        if (ccd2_left_index<= 5*ccd2_IllegalEdge):
            ccd2_center = ccd2_right_index - ccd2_road_len // 2
            ccd2_left_lose_flag = 1
        elif (ccd2_right_index >= 127 - 5*ccd2_IllegalEdge):
            ccd2_center = ccd2_left_index + ccd2_road_len // 2
            ccd2_right_lose_flag = 1
    else: 
        ccd2_center = (ccd2_left_index + ccd2_right_index) // 2
    return ccd2_center,ccd2_center_last,ac_ccd2_road_len,ccd2_left_index,ccd2_right_index,ccd2_left_index_last,ccd2_right_index_last,ac_ccd2_road_len


####元素
def go_annual(ac_ccd1_road_len,ac_ccd2_road_len,ccd1_left_index,ccd1_right_index,ccd2_left_index,ccd2_right_index,annual_state,annual_Detect_flag,annual_yaw_angle,annual_dir):
    global annual_flag
    global ccd1_road_len
    global ccd2_road_len
    global annual_delay
    global ccd1_mid
    global ccd2_mid
    global annual_all_error1
    global annual_ave_error1
    global annual_all_error2
    global annual_ave_error2
    global annual_all_tips
    global ccd1_IllegalEdge
    global ccd2_IllegalEdge
    global ccd1_center
    global ccd2_center
    global ccd1_right_index_last
    global speed_type
    global Stop
    global annual_delay_time
    annual_Detect_flag,annual_dir = leftanulusdetect(ac_ccd1_road_len,ac_ccd2_road_len,ccd1_left_index,ccd1_right_index,ccd2_left_index,ccd2_right_index,ccd1_left_index_last,annual_state,annual_Detect_flag)
    annual_Detect_flag,annual_dir = rightanulusdetect(ac_ccd1_road_len,ac_ccd2_road_len,ccd1_left_index,ccd1_right_index,ccd2_left_index,ccd2_right_index,ccd1_right_index_last,annual_state,annual_Detect_flag)
    if( annual_state == 0 and annual_dir >= 1 ):
        annual_state = 1
    if(annual_state == 1 and abs(annual_yaw_angle) <= 200):
        return annual_flag,annual_state,annual_dir,annual_Detect_flag,annual_yaw_angle
    if(annual_state == 1):
        if(abs(annual_yaw_angle) >= 300):
            if(ccd1_right_index >= 127 - 2*ccd1_IllegalEdge and annual_dir == 1 ):
                annual_Detect_flag = 0
                annual_state = 2
            if(ccd1_left_index <= 2*ccd1_IllegalEdge and annual_dir == 2 ):
                annual_Detect_flag = 0
                annual_state = 2
    if(annual_state == 2 ):#按平均误差走
        if(ac_ccd1_road_len <=  75 ):#直到到达圆中
            annual_state = 3
            annual_all_error1 = 0
            annual_all_error2 = 0
            annual_all_tips = 0
            annual_Detect_flag = 0
    if(annual_state == 3):
        annual_delay += 1
        if(annual_delay >= annual_delay_time):
            annual_dir = 0
            annual_state = 0
            annual_yaw_angle = 0
            annual_delay = 0
    annual_flag = annual_state
    return annual_flag,annual_state,annual_dir,annual_Detect_flag,annual_yaw_angle
#左圆环口识别
annual_debug = 0
def leftanulusdetect(ac_ccd1_road_len,ac_ccd2_road_len,ccd1_left_index,ccd1_right_index,ccd2_left_index,ccd2_right_index,ccd1_left_index_last,annual_state,annual_Detect_flag):
    global annual_flag
    global annual_dir
    global ccd2_road_len
    global ccd1_road_len
    global ccd1_mid
    global ccd2_mid
    global annual_all_error1
    global annual_ave_error1
    global annual_all_error2
    global annual_ave_error2
    global annual_all_tips
    global ccd1_IllegalEdge
    global ccd2_IllegalEdge
    global annual_debug
    global ccd2_center
    global ccd1_center
    global allow_annual_time
    ccd1_half_road_len = ccd1_road_len//2
    ccd2_half_road_len = ccd2_road_len//2
    if(annual_Detect_flag == 0):
        if(annual_state > 0 or
           ac_ccd1_road_len < ccd1_road_len + 30
           or abs(ccd2_center-ccd2_mid) >  10  or abs(ccd1_center-ccd1_mid ) >  10 ):
#             lcd.str16(0,16*0,"ccd1_left_index <  5 {:>.5f}.".format(abs(ccd1_left_index-ccd1_mid + ccd1_road_len//2)),0xFFFF)
#             lcd.str16(0,16*1,"ccd2_left_index <  5 {:>.5f}.".format(abs(ccd2_left_index-ccd2_mid + ccd2_road_len//2)),0xFFFF)
#             lcd.str16(0,16*2,"ccd1_right_index >  5 {:>.5f}.".format(abs(ccd1_right_index-ccd1_mid - ccd1_road_len//2)),0xFFFF)
#             lcd.str16(0,16*3,"ccd2_right_index >  5 {:>.5f}.".format(abs(ccd2_right_index-ccd2_mid - ccd2_road_len//2)),0xFFFF)
            annual_Detect_flag = 0
            allow_annual_time = 0
            return annual_Detect_flag,annual_dir
    if(ac_ccd1_road_len > ccd1_road_len + 30 and annual_state == 0 and  annual_Detect_flag == 0):
        if(ccd1_left_index <= 2*ccd1_IllegalEdge ):
            annual_Detect_flag = 1
        if(ccd1_right_index >=  127 - 4*ccd1_IllegalEdge):
            annual_Detect_flag = 0
            annual_dir = 0
            allow_annual_time = 0
    if(annual_Detect_flag > 4 or annual_state != 0 or annual_flag != 0):
        return annual_Detect_flag,annual_dir
    if(annual_Detect_flag == 1 and annual_flag == 0):
        allow_annual_time += 1
        if( ccd1_left_index - ccd1_left_index_last >= 1 and  annual_state == 0 and ccd1_left_index > 2*ccd1_IllegalEdge):
            annual_Detect_flag = 2
            allow_annual_time = 0
        if(ccd1_right_index >=  127 - 4*ccd1_IllegalEdge):
            annual_Detect_flag = 0
            annual_dir = 0
            allow_annual_time = 0
        if(allow_annual_time > 25):
            allow_annual_time = 0
            annual_Detect_flag = 0
    if(annual_Detect_flag == 2 and annual_flag == 0):
        allow_annual_time += 1
        if( ccd1_left_index - ccd1_left_index_last <= -1 and annual_state == 0 ):
            if(abs(ac_ccd2_road_len - ccd2_road_len ) < 15 ):
                annual_Detect_flag = 3
                allow_annual_time = 0
        if(allow_annual_time > 25):
            allow_annual_time = 0
            annual_Detect_flag = 0
    if(annual_Detect_flag == 3):
        allow_annual_time += 1
        if( ccd1_left_index <= 2*ccd1_IllegalEdge ):
            annual_dir = 1
        if(allow_annual_time > 25 ):
            annual_Detect_flag = 0
            allow_annual_time = 0
    return annual_Detect_flag,annual_dir
#右圆环口识别
def rightanulusdetect(ac_ccd1_road_len,ac_ccd2_road_len,ccd1_left_index,ccd1_right_index,ccd2_left_index,ccd2_right_index,ccd1_right_index_last,annual_state,annual_Detect_flag):
    global annual_flag
    global annual_dir
    global ccd2_road_len
    global ccd1_road_len
    global ccd1_mid
    global ccd2_mid
    global annual_all_error1
    global annual_ave_error1
    global annual_all_error2
    global annual_ave_error2
    global annual_all_tips
    global ccd1_IllegalEdge
    global ccd2_IllegalEdge
    global ccd2_center
    global ccd1_center
    global allow_annual_time
    ccd1_half_road_len = ccd1_road_len//2
    ccd2_half_road_len = ccd2_road_len//2
    if(annual_Detect_flag == 0):
        if(annual_state > 0 or
       ac_ccd1_road_len < ccd1_road_len + 30 
           or abs(ccd2_center-ccd2_mid) >  10  or abs(ccd1_center-ccd1_mid ) >  10 ):
            annual_Detect_flag = 0
            allow_annual_time = 0
            return annual_Detect_flag,annual_dir
    if( ac_ccd1_road_len> ccd1_road_len + 30  and annual_state == 0  and  annual_Detect_flag == 0 ):
        if(ccd1_right_index >= 127 - 2*ccd1_IllegalEdge ):
            annual_Detect_flag = 5
            annual_dir = 0
            allow_annual_time = 0
        if(ccd1_left_index <=  4*ccd1_IllegalEdge):
            annual_Detect_flag = 0
            annual_dir = 0
            allow_annual_time = 0
    if( annual_Detect_flag < 5 or annual_state != 0 or annual_flag != 0 ):
        return annual_Detect_flag,annual_dir
    if( annual_Detect_flag == 5 and annual_flag == 0 ):
        allow_annual_time += 1
        if(ccd1_right_index - ccd1_right_index_last <= -1 and annual_state == 0  and ccd1_right_index < 127 - 2*ccd1_IllegalEdge):
            annual_Detect_flag = 6
            allow_annual_time = 0
            annual_dir = 0
        if(ccd1_left_index <= 4*ccd1_IllegalEdge):
            annual_Detect_flag = 0
            allow_annual_time = 0
            annual_dir = 0 
        if(allow_annual_time > 25):
            allow_annual_time = 0
            annual_Detect_flag = 0
            annual_dir = 0
    if( annual_Detect_flag == 6 and annual_flag == 0 ):
        allow_annual_time += 1
        if(ccd1_right_index - ccd1_right_index_last >= 1  and annual_state == 0 ):
            if( abs(ac_ccd2_road_len - ccd2_road_len ) < 15):
                annual_Detect_flag = 7
                allow_annual_time = 0
                annual_dir = 0
        if(allow_annual_time > 25 ):
            annual_Detect_flag = 0
            allow_annual_time = 0
    if( annual_Detect_flag == 7 ):
        allow_annual_time += 1
        if(allow_annual_time < 25 and ccd1_right_index >= 127 - 2*ccd1_IllegalEdge ):
            annual_dir = 2
        if(allow_annual_time > 25 ):
            annual_Detect_flag = 0
            allow_annual_time = 0
    return annual_Detect_flag,annual_dir


def go_parking(parking_flag, parking_state,parking_tip1,parking_tip2,ccd1_parking_difference,ccd2_parking_difference,ccd1_left_index,ccd1_right_index,ccd2_left_index,ccd2_right_index,allow_parking_time):
    global all_road_len
    global ccd2_road_len
    global ccd1_road_len
    global ac_ccd2_road_len
    global ac_ccd1_road_len
    global ccd1_center
    global ccd2_center
    global ccd1_mid
    global ccd2_mid
    parking_tip1 = 0
    parking_tip2 = 0
    if(all_road_len < 3):
        return parking_flag, parking_state,parking_tip1,parking_tip2,allow_parking_time
    for i in range(1, 31):
        if(parking_state == 0 and abs(ac_ccd2_road_len - ccd2_road_len) <5 and  abs(ac_ccd1_road_len - ccd1_road_len) > 5 ):
            parking_tip1 += int(ccd1_parking_difference[i] * ccd1_parking_difference[i-1] < -1)
        if(parking_state == 1 and abs(ac_ccd1_road_len - ccd1_road_len) <5 and  abs(ac_ccd2_road_len - ccd2_road_len) > 5 ):
            parking_tip2 += int(ccd2_parking_difference[i] * ccd2_parking_difference[i-1] < -1)    
    if(parking_state == 0):
        if(parking_tip1 >= 8):
            parking_state = 1
    if(parking_state == 1):
        allow_parking_time += 1
        if(parking_tip2 >= 5):
            parking_state = 2
        if(allow_parking_time > 25):
            parking_state = 0
            allow_parking_time = 0
    if(parking_state == 2):
        allow_parking_time += 1
        if(allow_parking_time > 20):
            parking_state = 3       
    parking_flag = parking_state
    return parking_flag, parking_state,parking_tip1,parking_tip2,allow_parking_time




#方向控制
def direction_control(ccd1_left_index,ccd1_right_index,ccd2_left_index,ccd2_right_index,annual_Detect_flag,annual_flag,parking_flag,ramp_flag,annual_dir,annual_all_error1,annual_all_error2,annual_all_tips):
    global ccd1_road_len
    global ccd2_road_len
    global ccd1_center
    global ccd2_center
    global ccd1_mid
    global ccd2_mid
    global ccd1_error
    global ccd2_error
    global ccd1_error_last
    global ccd2_error_last
    global ccd1_center_last
    global ccd2_center_last
    global ac_ccd1_road_len
    global ac_ccd2_road_len
    global ccd1_IllegalEdge
    global ccd2_IllegalEdge
    global all_road_len
    global annual_ave_error1
    global annual_ave_error2
    global annual_out_offset
    global parking_tip1
    global parking_tip2
    ccd1_center_last = ccd1_center
    ccd2_center_last = ccd2_center
    ccd1_error_last = ccd1_error
    ccd2_error_last = ccd2_error         
    if(annual_Detect_flag or annual_flag ):
        if(annual_Detect_flag == 3 and annual_dir == 1):##右
            ccd1_center = ccd1_left_index + int(ccd1_road_len/annual_out_offset)
            ccd2_center = ccd2_left_index + int(ccd1_road_len/annual_out_offset)
        elif(annual_Detect_flag == 7 and annual_dir == 2):##右
            ccd1_center = ccd1_right_index - int(ccd1_road_len/annual_out_offset)
            ccd2_center = ccd2_right_index - int(ccd2_road_len/annual_out_offset)
        if(annual_flag == 3 and annual_dir == 1 ):##左
            ccd1_center = ccd1_right_index - int(ccd1_road_len/annual_out_offset)
            ccd2_center = ccd2_right_index - int(ccd2_road_len/annual_out_offset)
        elif(annual_flag == 3 and annual_dir == 2 ):##右
            ccd1_center = ccd1_left_index  + int(ccd1_road_len/annual_out_offset)
            ccd2_center = ccd2_left_index  + int(ccd2_road_len/annual_out_offset)
    if(parking_flag == 1):
        ccd1_center = ccd1_mid
    if(parking_tip2 == 2):
        ccd2_center = ccd2_mid
    if(all_road_len < 5):
        if(ac_ccd1_road_len < ccd1_road_len):
            ccd1_center = ccd1_mid
        if(ac_ccd2_road_len < ccd2_road_len):
            ccd2_center = ccd2_mid
    if(ccd1_center >= 127):
        ccd1_center = 127
    elif(ccd1_center <= 0):
        ccd1_center = 0
    if(ccd2_center >= 127):
        ccd2_center = 127
    elif(ccd2_center <= 0):
        ccd2_center = 0
    ccd1_error = ccd1_center - ccd1_mid
    ccd2_error = ccd2_center - ccd2_mid
    if(annual_flag == 1):
        annual_all_error1 = annual_all_error1 + ccd1_error
        annual_all_error2 = annual_all_error2 + ccd2_error
        annual_all_tips =  annual_all_tips + 1
        annual_ave_error1 = int(annual_all_error1/annual_all_tips)
        annual_ave_error2 = int(annual_all_error2/annual_all_tips)
    if(annual_flag == 2 and annual_state == 2):
        if(annual_dir == 1):#左出圆环
            if(abs(ccd1_left_index + ccd1_road_len//annual_out_offset - ccd1_mid) > abs(annual_ave_error1)):
                ccd1_error = annual_ave_error1
            else:
                ccd1_error = int((ccd1_left_index + ccd1_road_len//annual_out_offset - ccd1_mid))
            if(abs(ccd2_left_index + ccd2_road_len//annual_out_offset - ccd2_mid) > abs(annual_ave_error2)):
                ccd2_error = annual_ave_error2
            else:
                ccd2_error = int((ccd2_left_index + ccd2_road_len//annual_out_offset - ccd2_mid))
        if(annual_dir == 2):#右出圆环
            if(abs(ccd1_right_index - ccd1_road_len//annual_out_offset- ccd1_mid) > abs(annual_ave_error1)):
                ccd1_error = annual_ave_error1
            else:
                ccd1_error = int((ccd1_right_index - ccd1_road_len//annual_out_offset - ccd1_mid))
            if(abs(ccd2_right_index - ccd2_road_len//annual_out_offset - ccd2_mid) > abs(annual_ave_error2)):
                ccd2_error = annual_ave_error2
            else:
                ccd2_error = int((ccd2_right_index - ccd2_road_len//annual_out_offset - ccd2_mid))
    return ccd1_error,ccd2_error,annual_all_error1,annual_all_error2,annual_all_tips,annual_all_error1,annual_all_error2,annual_all_tips



###################################图像代码区域块2####################################################
def value_img_show():
    global ccd_data1
    global ccd_data2
    global ac_ccd1_road_len
    global ac_ccd2_road_len
    global annual_yaw_angle
    global CarAngle_Turn_Angle
    global encl_data
    global encr_data
    global all_road_len
    global allencl_data
    global allencr_data
    global speed_TrueL
    global speed_TrueR
    global ccd1_ave_threshold
    global ccd2_ave_threshold
    global annual_Detect_flag
    global annual_flag
    global CarAngle_Yawrate
    global annual_debug
    global image_show_flag
    global value_show_flag
    global allow_value_show
    global allow_image_show
    global CarAngle_Pit_Angle
    global Red_Distance
    if 0 != value_show_flag and 0 == image_show_flag:
        lcd.str16(0,16*0,"ccd1_road_len = {:>.5f}.".format(ccd1_road_len),0xFFFF)
        lcd.str16(0,16*1,"ccd2_road_len = {:>.5f}.".format(ccd2_road_len),0xFFFF)
        lcd.str16(0,16*2,"encl_data = {:>.5f}.".format(encl_data),0xFFFF)
        lcd.str16(0,16*3,"encr_data = {:>.5f}.".format(encr_data),0xFFFF)
        lcd.str16(0,16*4,"allencl_data = {:>.5f}.".format(allencl_data),0xFFFF)
        lcd.str16(0,16*5,"allencr_data = {:>.5f}.".format(allencr_data),0xFFFF)
        lcd.str16(0,16*6,"speed_TrueL = {:>.5f}.".format(speed_TrueL),0xFFFF)
        lcd.str16(0,16*7,"speed_TrueR = {:>.5f}.".format(speed_TrueR),0xFFFF)
        lcd.str16(0,130+16*0,"ccd1_center = {:>.5f}.".format(ccd1_center),0xFFFF)
        lcd.str16(0,130+16*1,"ccd2_center = {:>.5f}.".format(ccd2_center),0xFFFF)
        lcd.str16(0,130+16*2,"ccd1_error = {:>.5f}.".format(ccd1_error),0xFFFF)
        lcd.str16(0,130+16*3,"ccd2_error = {:>.5f}.".format(ccd2_error),0xFFFF)
        lcd.str16(0,130+16*4,"Red_Distance = {:>.5f}.".format(Red_Distance),0xFFFF)
        lcd.str16(0,130+16*5,"Inner_Error = {:>.5f}.".format(DirInner_Error),0xFFFF)
        lcd.str16(0,130+16*6,"Turn_Angle = {:>.5f}.".format(CarAngle_Turn_Angle),0xFFFF)
        lcd.str16(0,130+16*7,"Inner_OutPut = {:>.5f}.".format(DirInner_OutPut),0xFFFF)
        lcd.str16(0,130+16*8,"Pit_Angle = {:>.5f}.".format(CarAngle_Pit_Angle),0xFFFF)
        lcd.str16(0,130+16*9,"ccd1_ave_threshold = {:>.5f}.".format(ccd1_ave_threshold),0xFFFF)
        lcd.str16(0,130+16*10,"ccd2_ave_threshold = {:>.5f}.".format(ccd2_ave_threshold),0xFFFF)
    elif 0 != image_show_flag and 0 == value_show_flag:        
        lcd.wave(0,  0, 128, 64, ccd_data1)
        lcd.wave(0, 64, 128, 64, ccd_data2)
        lcd.wave(0, 128, 128, 64,ccd_deal1)
        lcd.wave(0, 192, 128, 64,ccd_deal2)
    elif 0 != image_show_flag and 0 != value_show_flag:        
        lcd.wave(0,  0, 128, 64, ccd_data1)
        lcd.wave(0, 64, 128, 64, ccd_data2)
        lcd.str16(0,130+16*0,"ccd1_center = {:>.5f}.".format(ccd1_center),0xFFFF)
        lcd.str16(0,130+16*1,"ccd2_center = {:>.5f}.".format(ccd2_center),0xFFFF)
        lcd.str16(0,130+16*2,"annual_Detect_flag = {:>.5f}.".format(annual_Detect_flag),0xFFFF)
        lcd.str16(0,130+16*3,"parking_tip1 = {:>.5f}.".format(parking_tip1),0xFFFF)
        lcd.str16(0,130+16*4,"parking_tip2 = {:>.5f}.".format(parking_tip2),0xFFFF)
        lcd.str16(0,130+16*5,"ccd1_left_index = {:>.5f}.".format(ccd1_left_index),0xFFFF)
        lcd.str16(0,130+16*6,"ccd1_right_index = {:>.5f}.".format(ccd1_right_index),0xFFFF)
        lcd.str16(0,130+16*7,"ccd2_left_index = {:>.5f}.".format(ccd2_left_index),0xFFFF)
        lcd.str16(0,130+16*8,"ccd2_right_index = {:>.5f}.".format(ccd2_right_index),0xFFFF)
        lcd.str16(0,130+16*9,"ac_ccd1_load_len = {:>.5f}.".format(ac_ccd1_road_len),0xFFFF)
        lcd.str16(0,130+16*10,"ac_ccd2_road_len = {:>.5f}.".format(ac_ccd2_road_len),0xFFFF)
        
        
ticker_flag1 = False
ac_imu_data = [0, 0, 0, 0, 0, 0]
CarAngle_Turn_Angle = 0
CarAngle_Pit_Angle = 0
Speed_ErrorFifo = [0, 0, 0]
Speed_ErrorDtFifo = [0, 0, 0]
DirOutter_Error = 0
DirOutter_ErrorDt = 0
DirOutter_ErrorLast = 0
DirOutter_Curvature_Correct = 0
CarDirACC = 0
gyo_yaw = 0
gyo_pit = 0
DirOutter_Curvature_Final = 0
DirInner_ErrorFifo = [ 0 , 0 , 0 , 0 , 0 , 0 ]
DirInner_ErrorDtTemp = [ 0, 0, 0, 0 ]
DirInner_FeedbackTemp = [ 0, 0, 0, 0 ]
DirInner_OutTemp = [ 0, 0, 0, 0 ]
DirInner_Error = 0
DirInner_OutPut = 0
DirOutter_OutPut = 0
data_wave = [ 111,222,333,444,555,666,777,888 ]
data_recv = [ 88,88,88,88,88,88,88,88 ]
while 1:#调参界面结束，初始化上面函数变量结束，到真正下位机运行循环界面
    if (ticker_flag1 == True):# # # # # # # # # # # # # V
        ccd_deal1, ccd_deal2 = get_the_dealCCd(ccd_data1,ccd_data2)
        ccd1_center,ccd1_center_last,ac_ccd1_road_len,ccd1_left_index,ccd1_right_index,ccd1_left_index_last,ccd1_right_index_last,ac_ccd1_road_len = find_the_ccd1Center(ccd_deal1,ccd1_center_last,ccd1_left_index,ccd1_right_index)
        ccd2_center,ccd2_center_last,ac_ccd2_road_len,ccd2_left_index,ccd2_right_index,ccd2_left_index_last,ccd2_right_index_last,ac_ccd2_road_len = find_the_ccd2Center(ccd_deal2,ccd2_center_last,ccd2_left_index,ccd2_right_index)
        if(ramp_flag == 0 and parking_flag == 0):
            annual_flag,annual_state,annual_dir,annual_Detect_flag,annual_yaw_angle = go_annual(ac_ccd1_road_len,ac_ccd2_road_len,ccd1_left_index,ccd1_right_index,ccd2_left_index,ccd2_right_index,annual_state,annual_Detect_flag,annual_yaw_angle,annual_dir)
        if(annual_flag == 0 and ramp_flag == 0):
            parking_flag,parking_state,parking_tip1,parking_tip2,allow_parking_time = go_parking(parking_flag, parking_state,parking_tip1,parking_tip2,ccd1_parking_difference,ccd2_parking_difference,ccd1_left_index,ccd1_right_index,ccd2_left_index,ccd2_right_index,allow_parking_time)
        ccd1_error,ccd2_error,annual_all_error1,annual_all_error2,annual_all_tips,annual_all_error1,annual_all_error2,annual_all_tips = direction_control(ccd1_left_index,ccd1_right_index,ccd2_left_index,ccd2_right_index,annual_Detect_flag,annual_flag,parking_flag,ramp_flag,annual_dir,annual_all_error1,annual_all_error2,annual_all_tips)
        ccd_error = ccd1_error*0.65+ccd2_error*0.35##ccd1和ccd2融合循迹，可以加一个低通滤波平滑这个不是连续变化的误差。
        speed_type = getspeed_type(speed_type)
        DirOutter_OutPut = direction_outter(ccd_error)
        if(1 == send_ccdimg_flag):
            wireless.send_ccd_image(WIRELESS_UART.ALL_CCD_BUFFER_INDEX)
        if(1 == send_data_flag):
            wireless.send_oscilloscope(DirOutter_OutPut,data_wave[1],data_wave[2],data_wave[3],data_wave[4],data_wave[5],data_wave[6],data_wave[7])
        if(1 == recv_data_flag):
            data_flag = wireless.data_analysis()
            for i in range(0,8):
                if (data_flag[i]):
                    data_recv[i] = wireless.get_data(i)
            if (data_recv[0] == 100.0 ):
                set_speed = 50
                data_recv[0] = 0
            if (data_recv[1] == 101.0 ):
                set_speed = -50
                data_recv[1] = 0
            if (data_recv[2] == 51.0  ):
                set_angle = set_angle + 60
                data_recv[2] = 0
            if (data_recv[3] == 50.0  ):
                set_angle = set_angle - 60
                data_recv[3] = 0
            if (data_recv[4] == 1.0   ):
                set_speed =50
                data_recv[4] = 0
            if (data_recv[5] == 2.0   ):
                set_speed =0
                data_recv[5] = 0
            if (data_recv[6] == 111.0 ):
                set_speed =0
                data_recv[6] = 0
            if (data_recv[7] == 111.0 ):
                set_speed =0
                data_recv[7] = 0 
        value_img_show()
        ticker_flag1 = False
    # # # # # # # # # # # # # V
    gc.collect()
#     if end_switch.value() == 0:
#     if(ccd2_ave_threshold < 180 and ccd1_ave_threshold < 180 ):
#         pit1.stop()
#         pit2.stop()
#         pit3.stop()
#         lcd.clear()
#         break




















