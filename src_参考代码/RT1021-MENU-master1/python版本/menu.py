from machine import *
from display import *
from smartcar import *
from seekfree import *
import math
import os
import io
import gc
import time
runtime = 0
delaytime = 0
paper_index = 0
last_paper_index = 0
value_index = 0
send_ccdimg_flag = 0 
send_data_flag = 0
recv_data_flag = 0
allow_value_show = 0
allow_image_show = 0
DirOutter_KP = 0
DirOutter_KP_EX = 0
DirOutter_KD = 0
CarInfo_CamMaxACC = 0
speed_kp = 0
speed_ki= 0
DirInner_KP= 0
DirInner_KD = 0
set_speed = 0
menu_speed = 0
runSpeedMode = 0
CONSTANT_SPEED = 0
VARIABLE_SPEED = 1
set_angle = 0
EXP_TIME = 0
image_show_flag = 0
value_show_flag = 0
AimSpeed = 0
StraightLSpeed = 0##加速
StraightSSpeed = 0
StraightBreakSpeed = 0
CurveSpeed = 0##二次公式弯道速度
AnnulusSpeed = 0
RampSpeed = 0##过坡道速度
ObstacleSpeed = 0##横断速度
ParkingSpeed = 0##停车速度
speed_type = 2
start_flag = False
high_level_us = 1000
open_bldc_flag = 0
Font_size_H = 16
Font_size_W = 8
go_flag = 0
annual_delay_time = 0
annual_out_offset = 0
#####################################################################################################
###################################按键代码区域块1####################################################
####################################################################################################

###################################按键菜单###################################
######################################################################
######################################################################
# # # # # # # # # # # # # # # 屏幕# # # # # # # # # # # # # # # 
rst = Pin('B9' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc  = Pin('B8' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C4' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
drv = LCD_Drv(SPI_INDEX=1, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
lcd = LCD(drv)
lcd.color(0xFFFF, 0x0000)
lcd.mode(2)
lcd.clear(0x0000)
# # # # # # # # # # # # # # # 屏幕# # # # # # # # # # # # # # #


##采用gpio设置引脚高低电平方式，请自行根据自己单片机采用的IO口修改。
end_switch = Pin('C18', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
key_up = Pin('D23', Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
key_down = Pin('D22', Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
key_left = Pin('D20', Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
key_right = Pin('D21', Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
# key = KEY_HANDLER(50)
def key_pit(time):
    global paper_index
    global value_index
    global allow_value_show
    global allow_image_show
    global go_flag
    if key_up.value() == 0:
        print(1)
        paper_index = paper_index - 1
    if key_down.value() == 0:
        paper_index = paper_index + 1
        print(2)
    if key_left.value() == 0:
        value_index = value_index + 1
        print(3)
    if key_right.value() == 0:
        value_index = value_index - 1
        print(4)
#     if key_allright.value() == 0:
#         go_flag = 1
#         lcd.clear(0x0000)
#         value_index = 0   
#         paper_index = 0
#         allow_value_show = 0
#         allow_image_show = 0
#         data_write()
#         print(5)
    
pit3 = ticker(2)
pit3.capture_list()
pit3.callback(key_pit)
pit3.start(200)

###################################按键菜单###################################
######################################################################
######################################################################


        


def data_read():##从txt中按顺序读出data
    global speed_kp
    global speed_ki
    global DirInner_KP
    global DirInner_KD
    global set_speed
    global EXP_TIME
    global DirOutter_KP
    global DirOutter_KP_EX
    global CarInfo_CamMaxACC
    global DirOutter_KD
    global image_show_flag
    global value_show_flag
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
    global ParkingSpeed
    global go_flag
    global Font_size_H
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
    DirInner_KP= read_data[4]
    DirInner_KD= read_data[5]
    set_speed = read_data[6]
    EXP_TIME = read_data[7]
    image_show_flag = read_data[8]
    value_show_flag = read_data[9]
    DirOutter_KP= read_data[10]
    DirOutter_KP_EX = read_data[11]
    DirOutter_KD = read_data[12]
    CarInfo_CamMaxACC = read_data[13]
    send_ccdimg_flag = read_data[14]
    send_data_flag = read_data[15]
    recv_data_flag = read_data[16]
    runSpeedMode =  read_data[17]
    StraightLSpeed =  read_data[18]
    StraightSSpeed =  read_data[19]
    StraightBreakSpeed =  read_data[20]
    CurveSpeed =  read_data[21]
    AnnulusSpeed =  read_data[22]
    RampSpeed =  read_data[23]
    ObstacleSpeed =  read_data[24]
    ParkingSpeed =  read_data[25]
    runtime =  read_data[26] 
    delaytime =  read_data[27]
    annual_delay_time =  read_data[28]
    annual_out_offset = read_data[29]
    high_level_us = read_data[30]
    open_bldc_flag = read_data[31]
    allow_value_show = read_data[32]
    allow_image_show = read_data[33]
    user_file.close()
    

def data_write():##把data写入txt
    global speed_kp
    global speed_ki
    global DirInner_KP
    global DirInner_KD
    global set_speed
    global EXP_TIME
    global DirOutter_KP
    global DirOutter_KP_EX
    global CarInfo_CamMaxACC
    global DirOutter_KD
    global image_show_flag
    global value_show_flag
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
    global ParkingSpeed
    global runtime
    global delaytime
    global annual_delay_time
    global annual_out_offset
    global high_level_us
    global open_bldc_flag
    global allow_value_show
    global allow_image_show
    write_data = 512* [0]
    write_data[0] = speed_kp
    write_data[1] = speed_ki
    write_data[2] = 0
    write_data[3] = 0
    write_data[4] = DirInner_KP
    write_data[5] = DirInner_KD
    write_data[6] = set_speed
    write_data[7] = EXP_TIME
    write_data[8] = image_show_flag
    write_data[9] = value_show_flag
    write_data[10] = DirOutter_KP
    write_data[11] = DirOutter_KP_EX
    write_data[12] = DirOutter_KD
    write_data[13] = CarInfo_CamMaxACC
    write_data[14] = send_ccdimg_flag
    write_data[15] = send_data_flag
    write_data[16] = recv_data_flag
    write_data[17] = runSpeedMode
    write_data[18] = StraightLSpeed
    write_data[19] = StraightSSpeed
    write_data[20] = StraightBreakSpeed
    write_data[21] = CurveSpeed
    write_data[22] = AnnulusSpeed 
    write_data[23] = RampSpeed
    write_data[24] = ObstacleSpeed 
    write_data[25] = ParkingSpeed 
    write_data[26] = runtime 
    write_data[27] = delaytime 
    write_data[28] = annual_delay_time
    write_data[29] = annual_out_offset
    write_data[30] = high_level_us
    write_data[31] = open_bldc_flag
    write_data[32] = allow_value_show
    write_data[33] = allow_image_show
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
         user_file.write("%s\n"%(str(write_data[index])))
    # 将缓冲区数据写入到文件 清空缓冲区 相当于保存指令
    user_file.flush()
    user_file.close()
    
    
def cursor_selected(max_idnex):
    global paper_index
    global last_paper_index
    if paper_index >= max_idnex:
        paper_index = 0
    elif paper_index <= -1:
        paper_index = max_idnex - 1
    for index in range(0, 20):
        if(index != paper_index+1):
            lcd.str16(0, index*Font_size_H,"  ", 0xFFFF)
    
    lcd.str16(0, (paper_index+1)*Font_size_H,"->", 0xF800)
        
    last_paper_index = paper_index
        
####总调参界面########################################################################
####总调参界面########################################################################################
####总调参界面############################################################################
def adjust_menu():
    global paper_index
    global value_index
    global key_ticker_flag
    global allow_value_show
    global allow_image_show
    global send_ccdimg_flag
    global send_data_flag
    global recv_data_flag
    global go_flag
    global Font_size_H
    data_read()
    lcd.clear(0xFFFF)
    end_paper_index = 3
    while True:
        if paper_index == 0:
            if value_index >= 1:
                value_index = 0
                control_menu()
        elif paper_index == 1:
            if value_index >= 1:
                value_index = 0
                image_menu()
        elif paper_index == 2:
            if value_index >= 1:
                value_index = 0
                function_menu()
        elif paper_index == end_paper_index:
            if value_index >= 1:
                lcd.clear(0x0000)
                value_index = 0   
                paper_index = 0
                allow_value_show = 1
                allow_image_show = 1
                data_write()
                break
        elif paper_index == end_paper_index+1: 
            if value_index >= 1:
                lcd.clear(0x0000)
                value_index = 0   
                paper_index = 0
                allow_value_show = 0
                allow_image_show = 0
                data_write()
                break
        if(go_flag == 1):
            go_flag = 0
            break
        lcd.str16(20, 0*Font_size_H, "     -=  Menu  =-     ", 0xF800)
        lcd.str16(20, 1*Font_size_H, "Control", 0x0000)
        lcd.str16(20, 2*Font_size_H, "image", 0x0000)
        lcd.str16(20, 3*Font_size_H, "Function", 0x0000)
        lcd.str16(20, (end_paper_index+1)*Font_size_H, "observer", 0xF800)
        lcd.str16(20, (end_paper_index+2)*Font_size_H, "go", 0xF800)
#         lcd.str16(20, 8*Font_size_H, "paper_index = {:>2d}".format(paper_index), 0x0000)
        
        cursor_selected(end_paper_index+2)
        if end_switch.value() == 0:
            pit3.stop()
            lcd.clear()
            break

####总调参界面########################################################################
####总调参界面########################################################################################
####总调参界面############################################################################


def control_menu():
    global paper_index
    global value_index
    global Font_size_H
    global go_flag
    lcd.clear(0xFFFF)
    paper_index = 0
    value_index = 0
    end_paper_index = 2
    while True:
        if paper_index == 0:
            if value_index >= 1:
                value_index = 0
                pid_menu()
        elif paper_index == 1:
            if value_index >= 1:
                value_index = 0
                speed_menu()
        elif paper_index == end_paper_index: 
            if value_index >= 1:
                lcd.clear(0xFFFF)
                value_index = 0
                time.sleep_ms(50)
                paper_index = 0
                value_index = 0
                return
            
        if(go_flag == 1):
            go_flag = 0
            break
        
        lcd.str16(0,  0*Font_size_H, "     -=  Control  =-     ", 0xF800)
        lcd.str16(20, 1*Font_size_H, "Pid",0x0000)
        lcd.str16(20, 2*Font_size_H, "Speed",0x0000)
        lcd.str16(20, (end_paper_index+1)*Font_size_H, "exit", 0xF800)
        cursor_selected(end_paper_index+1)


def pid_menu():
    global paper_index
    global value_index
    lcd.clear(0xFFFF)
    global speed_kp
    global speed_ki
    global DirInner_KP
    global DirInner_KD
    global DirOutter_KP
    global DirOutter_KP_EX
    global CarInfo_CamMaxACC
    global DirOutter_KD
    global go_flag
    global Font_size_H
    paper_index = 0
    value_index = 0
    end_paper_index = 8
    while True:
        if paper_index == 0:
            if value_index >= 1:
                value_index = 0
                speed_kp += 1
            elif value_index <= -1:
                value_index = 0
                speed_kp -= 1
        elif paper_index == 1:
            if value_index >= 1:
                value_index = 0
                speed_ki += 1
            elif value_index <= -1:
                value_index = 0
                speed_ki -= 1
        elif paper_index == 2:
            if value_index >= 1:
                value_index = 0
                DirInner_KP += 1
            elif value_index <= -1:
                value_index = 0
                DirInner_KP -= 1
        elif paper_index == 3:
            if value_index >= 1:
                value_index = 0
                DirInner_KD += 1
            elif value_index <= -1:
                value_index = 0
                DirInner_KD -= 1
        elif paper_index == 4:
            if value_index >= 1:
                value_index = 0
                DirOutter_KP += 1
            elif value_index <= -1:
                value_index = 0
                DirOutter_KP -= 1
        elif paper_index == 5:
            if value_index >= 1:
                value_index = 0
                DirOutter_KP_EX += 1
            elif value_index <= -1:
                value_index = 0
                DirOutter_KP_EX -= 1
        elif paper_index == 6:
            if value_index >= 1:
                value_index = 0
                DirOutter_KD += 1
            elif value_index <= -1:
                value_index = 0
                DirOutter_KD -= 1
        elif paper_index == 7:
            if value_index >= 1:
                value_index = 0
                CarInfo_CamMaxACC += 10
            elif value_index <= -1:
                value_index = 0
                CarInfo_CamMaxACC -= 10
        elif paper_index == end_paper_index: 
            if value_index >= 1:
                lcd.clear(0xFFFF)
                value_index = 0
                time.sleep_ms(50)
                paper_index = 0
                value_index = 0
                return
        
        if(go_flag == 1):
            go_flag = 0
            break
        
        lcd.str16(0, 0*Font_size_H, "     -=  PID  =-     ", 0xF800)
        lcd.str16(20, 1*Font_size_H, "speed_kp = {:5d}".format(speed_kp), 0x0000)
        lcd.str16(20, 2*Font_size_H, "speed_ki = {:5d}".format(speed_ki), 0x0000)
        lcd.str16(20, 3*Font_size_H, "DirInner_KP = {:5d}".format(DirInner_KP), 0x0000)
        lcd.str16(20, 4*Font_size_H, "DirInner_KD = {:5d}".format(DirInner_KD), 0x0000)
        lcd.str16(20, 5*Font_size_H, "DirOutter_KP = {:5d}".format(DirOutter_KP), 0x0000)
        lcd.str16(20, 6*Font_size_H, "DirOutter_KP_EX = {:5d}".format(DirOutter_KP_EX), 0x0000)
        lcd.str16(20, 7*Font_size_H, "DirOutter_KD = {:5d}".format(DirOutter_KD), 0x0000)
        lcd.str16(20, 8*Font_size_H, "CarInfo_CamMaxACC = {:5d}".format(CarInfo_CamMaxACC), 0x0000)
        lcd.str16(20, (end_paper_index+1)*Font_size_H, "exit", 0xF800)
        #lcd.str16(20, 9*Font_size_H, "paper_index = {:>2d}".format(paper_index), 0x0000)
        cursor_selected(end_paper_index+1)
    
def speed_menu():
    global paper_index
    global value_index
    lcd.clear(0xFFFF)
    global set_speed
    global StraightLSpeed
    global StraightSSpeed
    global StraightBreakSpeed
    global CurveSpeed
    global AnnulusSpeed
    global RampSpeed
    global ObstacleSpeed
    global ParkingSpeed
    global go_flag
    global Font_size_H
    paper_index = 0
    value_index = 0
    end_paper_index = 9
    while True:
        if paper_index == 0:
            if value_index >= 1:
                value_index = 0
                set_speed += 10
            elif value_index <= -1:
                value_index = 0
                set_speed -= 10
        if paper_index == 1:
            if value_index >= 1:
                value_index = 0
                StraightLSpeed += 10
            elif value_index <= -1:
                value_index = 0
                StraightLSpeed -= 10
        elif paper_index == 2:
            if value_index >= 1:
                value_index = 0
                StraightSSpeed += 10
            elif value_index <= -1:
                value_index = 0
                StraightSSpeed -= 10
        elif paper_index == 3:
            if value_index >= 1:
                value_index = 0
                StraightBreakSpeed += 10
            elif value_index <= -1:
                value_index = 0
                StraightBreakSpeed -= 10
        elif paper_index == 4:
            if value_index >= 1:
                value_index = 0
                CurveSpeed += 10
            elif value_index <= -1:
                value_index = 0
                CurveSpeed -= 10
        elif paper_index == 5:
            if value_index >= 1:
                value_index = 0
                AnnulusSpeed += 10
            elif value_index <= -1:
                value_index = 0
                AnnulusSpeed -= 10
        elif paper_index == 6:
            if value_index >= 1:
                value_index = 0
                RampSpeed += 10
            elif value_index <= -1:
                value_index = 0
                RampSpeed -= 10
        elif paper_index == 7:
            if value_index >= 1:
                value_index = 0
                ObstacleSpeed += 10
            elif value_index <= -1:
                value_index = 0
                ObstacleSpeed -= 10
        elif paper_index == 8:
            if value_index >= 1:
                value_index = 0
                ParkingSpeed += 10
            elif value_index <= -1:
                value_index = 0
                ParkingSpeed -= 10
        elif paper_index == end_paper_index: 
            if value_index >= 1:
                lcd.clear(0xFFFF)
                value_index = 0
                time.sleep_ms(50)
                paper_index = 0
                value_index = 0
                return
        
        if(go_flag == 1):
            go_flag = 0
            break
        lcd.str16(0, 0*Font_size_H, "     -=  speed  =-     ", 0xF800)
        lcd.str16(20, 1*Font_size_H, "set_speed = {:5d}".format(set_speed), 0x0000)
        lcd.str16(20, 2*Font_size_H, "StraightLSpeed = {:5d}".format(StraightLSpeed), 0x0000)
        lcd.str16(20, 3*Font_size_H, "StraightSSpeed = {:5d}".format(StraightSSpeed), 0x0000)
        lcd.str16(20, 4*Font_size_H, "StraightBreakSpeed = {:5d}".format(StraightBreakSpeed), 0x0000)
        lcd.str16(20, 5*Font_size_H, "CurveSpeed = {:5d}".format(CurveSpeed), 0x0000)
        lcd.str16(20, 6*Font_size_H, "AnnulusSpeed = {:5d}".format(AnnulusSpeed), 0x0000)
        lcd.str16(20, 7*Font_size_H, "RampSpeed = {:5d}".format(RampSpeed), 0x0000)
        lcd.str16(20, 8*Font_size_H, "ObstacleSpeed = {:5d}".format(ObstacleSpeed), 0x0000)
        lcd.str16(20, 9*Font_size_H, "ParkingSpeed = {:5d}".format(ParkingSpeed), 0x0000)
        lcd.str16(20, (end_paper_index+1)*Font_size_H, "exit", 0xF800)
        #lcd.str16(20, 9*Font_size_H, "paper_index = {:>2d}".format(paper_index), 0x0000)
        cursor_selected(end_paper_index+1)
        
        
###################################################################################        
###################################################################################        
###################################################################################        
###################################################################################        
###################################################################################        
###################################################################################        
###################################################################################        
###################################################################################        
###################################################################################
def image_menu():
    global paper_index
    global value_index
    lcd.clear(0xFFFF)
    global EXP_TIME
    global go_flag
    global annual_delay_time
    global annual_out_offset
    paper_index = 0
    value_index = 0
    end_paper_index = 3
    while True:
        if paper_index == 0:
            if value_index >= 1:
                value_index = 0
                EXP_TIME+= 1
            elif value_index <= -1:
                value_index = 0
                EXP_TIME -= 1
            if EXP_TIME > 10:
                EXP_TIME = 10
            if EXP_TIME < 1 :
                EXP_TIME = 1
        if paper_index == 1:
            if value_index >= 1:
                value_index = 0
                annual_delay_time+= 10
            elif value_index <= -1:
                value_index = 0
                annual_delay_time-= 10
            if annual_delay_time < 10:
                annual_delay_time = 10
        if paper_index == 2:
            if value_index >= 1:
                value_index = 0
                annual_out_offset += 5
            elif value_index <= -1:
                value_index = 0
                annual_out_offset-= 5
            if annual_out_offset < 200:
                annual_out_offset = 200
        elif paper_index == end_paper_index: 
            if value_index >= 1:
                lcd.clear(0xFFFF)   
                value_index = 0
                time.sleep_ms(50)
                paper_index = 0
                value_index = 0
                return
        
        if(go_flag == 1):
            go_flag = 0
            break
        lcd.str16(0, 0*Font_size_H, "     -=  image  =-     ", 0xF800)
        lcd.str16(20, 1*Font_size_H, "EXP_TIME = {:5d}".format(EXP_TIME), 0x0000)
        lcd.str16(20, 2*Font_size_H, "annual_delay_time = {:5d}".format(annual_delay_time), 0x0000)
        lcd.str16(20, 3*Font_size_H, "annual_out_offset = {:5f}".format(float(annual_out_offset)/100), 0x0000)
        lcd.str16(20, (end_paper_index+1)*Font_size_H, "exit", 0xF800)
        #lcd.str16(20, 3*Font_size_H, "paper_index = {:>2d}".format(paper_index), 0x0000)
        cursor_selected(end_paper_index + 1)
        

def function_menu():
    global paper_index
    global value_index
    lcd.clear(0xFFFF)
    global image_show_flag
    global value_show_flag
    global send_ccdimg_flag
    global send_data_flag
    global recv_data_flag
    global go_flag
    global CONSTANT_SPEED
    global VARIABLE_SPEED
    global runSpeedMode
    global runtime
    global delaytime
    global high_level_us
    global open_bldc_flag
    paper_index = 0
    value_index = 0
    end_paper_index = 11
    while True:
        if paper_index == 0:
            if value_index >= 1:
                value_index = 0
                image_show_flag+= 1
            elif value_index <= -1:
                value_index = 0
                image_show_flag -= 1
            if image_show_flag > 1:
                image_show_flag = 0
            if image_show_flag < 0 :
                image_show_flag = 1
        elif paper_index == 1:
            if value_index >= 1:
                value_index = 0
                value_show_flag+= 1
            elif value_index <= -1:
                value_index = 0
                value_show_flag -= 1
            if value_show_flag > 1:
                value_show_flag = 0
            if value_show_flag < 0 :
                value_show_flag = 1
        elif paper_index == 2:
            if value_index >= 1:
                value_index = 0
                send_ccdimg_flag+= 1
            elif value_index <= -1:
                value_index = 0
                send_ccdimg_flag -= 1
            if send_ccdimg_flag > 1:
                send_ccdimg_flag = 0
            if send_ccdimg_flag < 0 :
                send_ccdimg_flag = 1
        elif paper_index == 3:
            if value_index >= 1:
                value_index = 0
                send_data_flag+= 1
            elif value_index <= -1:
                value_index = 0
                send_data_flag -= 1
            if send_data_flag > 1:
                send_data_flag = 0
            if send_data_flag < 0 :
                send_data_flag = 1
        elif paper_index == 4:
            if value_index >= 1:
                value_index = 0
                recv_data_flag+= 1
            elif value_index <= -1:
                value_index = 0
                recv_data_flag -= 1
            if recv_data_flag > 1:
                recv_data_flag = 0
            if recv_data_flag < 0 :
                recv_data_flag = 1
        elif paper_index == 5:
            if value_index >= 1:
                value_index = 0
                runSpeedMode += 1
            elif value_index <= -1:
                value_index = 0
                runSpeedMode -= 1
            if runSpeedMode > 1:
                runSpeedMode = CONSTANT_SPEED
            if runSpeedMode < 0 :
                runSpeedMode = VARIABLE_SPEED
        elif paper_index == 6:
            if value_index >= 1:
                value_index = 0
                runSpeedMode += 1
            elif value_index <= -1:
                value_index = 0
                runSpeedMode -= 1
            if runSpeedMode > 1:
                runSpeedMode = CONSTANT_SPEED
            if runSpeedMode < 0 :
                runSpeedMode = VARIABLE_SPEED
        elif paper_index == 7:
            if value_index >= 1:
                value_index = 0
                runtime += 1
            elif value_index <= -1:
                value_index = 0
                runtime -= 1
            if runtime < 0 :
                runtime = 0
        elif paper_index == 8:
            if value_index >= 1:
                value_index = 0
                delaytime += 1
            elif value_index <= -1:
                value_index = 0
                delaytime -= 1
            if delaytime < 0 :
                delaytime = 0
        elif paper_index == 9:
            if value_index >= 1:
                value_index = 0
                high_level_us += 100
            elif value_index <= -1:
                value_index = 0
                high_level_us -= 100
            if high_level_us < 1000 :
                high_level_us = 1000
            elif high_level_us > 2000 :
                high_level_us = 2000
        elif paper_index == 10:
            if value_index >= 1:
                value_index = 0
                open_bldc_flag += 1
            elif value_index <= -1:
                value_index = 0
                open_bldc_flag -= 1
            if open_bldc_flag < 0 :
                open_bldc_flag = 1
            elif open_bldc_flag > 1 :
                open_bldc_flag = 0
        elif paper_index == end_paper_index: 
            if value_index >= 1:
                lcd.clear(0xFFFF)   
                value_index = 0
                time.sleep_ms(50)
                paper_index = 0
                value_index = 0
                return

        if(go_flag == 1):
            go_flag = 0
            break
        lcd.str16(0, 0*Font_size_H, "     -=  image  =-     ", 0xF800)
        if image_show_flag == 1:
            lcd.str16(20, 1*Font_size_H, "image_show     *", 0xF800)
        elif image_show_flag == 0:
            lcd.str16(20, 1*Font_size_H, "image_show      ", 0x0000)
        if value_show_flag == 1:
            lcd.str16(20, 2*Font_size_H, "value_show    *", 0xF800)
        elif value_show_flag== 0:
            lcd.str16(20, 2*Font_size_H, "value_show     ", 0x0000)
        if send_ccdimg_flag == 1:
            lcd.str16(20, 3*Font_size_H, "send_ccdimg    *", 0xF800)
        elif send_ccdimg_flag == 0:
            lcd.str16(20, 3*Font_size_H, "send_ccdimg     ", 0x0000)
        if send_data_flag == 1:
            lcd.str16(20, 4*Font_size_H, "send_data      *", 0xF800)
        elif send_data_flag == 0:
            lcd.str16(20, 4*Font_size_H, "send_data       ", 0x0000)
        if recv_data_flag == 1:
            lcd.str16(20, 5*Font_size_H, "recv_data      *", 0xF800)
        elif recv_data_flag == 0:
            lcd.str16(20, 5*Font_size_H, "recv_data       ", 0x0000)
        if runSpeedMode == 0:
            lcd.str16(20, 6*Font_size_H, "constand      *", 0xF800)
            lcd.str16(20, 7*Font_size_H, "valuable       ", 0x0000)
        elif runSpeedMode == 1:
            lcd.str16(20, 6*Font_size_H, "constand       ", 0x0000)
            lcd.str16(20, 7*Font_size_H, "valuable      *", 0xF800)
        lcd.str16(20, 8*Font_size_H, "runtime = {:5d}".format(runtime), 0x0000)
        lcd.str16(20, 9*Font_size_H, "delaytime = {:5d}".format(delaytime), 0x0000)
        lcd.str16(20, 10*Font_size_H, "high_level_us = {:5d}".format(high_level_us), 0x0000)
        if open_bldc_flag == 1:
            lcd.str16(20, 11*Font_size_H, "open_bldc      *", 0xF800)
        elif open_bldc_flag == 0:
            lcd.str16(20, 11*Font_size_H, "open_bldc       ", 0x0000)
        
         
        lcd.str16(20, (end_paper_index+1)*Font_size_H, "exit", 0xF800)
        #lcd.str16(20, 3*Font_size_H, "paper_index = {:>2d}".format(paper_index), 0x0000)
        cursor_selected(end_paper_index + 1)
##从本地txt中按顺序读出data##由于两个文件的变量可能不互通，所以在主循环这里再读一次txt文件的数据
##从本地txt中按顺序读出data##由于两个文件的变量可能不互通，所以在主循环这里再读一次txt文件的数据
