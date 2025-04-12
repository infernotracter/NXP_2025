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


# 包含 gc time 类
import gc
import time

# 核心板上 C4 是 LED
led1 = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)

# 拨码开关2
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 拨码开关4
switch_3 = Pin('B14', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 拨码开关3
switch_4 = Pin('B15', Pin.IN, pull=Pin.PULL_UP_47K, value=True)

# 实例化 KEY_HANDLER 模块
key = KEY_HANDLER(10)

# 实例化 MOTOR_CONTROLLER 电机驱动模块 一共四个参数 两个必填两个可选 [mode,freq,duty,invert]
# mode - 工作模式 一共四种选项 [PWM_C24_DIR_C26,PWM_C25_DIR_C27,PWM_C24_PWM_C26,PWM_C25_PWM_C27]
#        实际对应 DRV8701 双驱双电机 以及 HIP4082 双驱双电机 请确保驱动正确且信号连接正确
# freq - PWM 频率
# duty - 可选参数 初始的占空比 默认为 0 范围 ±10000 正数正转 负数反转 正转反转方向取决于 invert
# invert - 可选参数 是否反向 默认为 0 可以通过这个参数调整电机方向极性
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty = 0, invert = True)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty = 0, invert = True)

# 本例程默认使用 DRV8701 双驱模块搭配双电机 ！！！
# 本例程默认使用 DRV8701 双驱模块搭配双电机 ！！！
# 本例程默认使用 DRV8701 双驱模块搭配双电机 ！！！

motor_dir = 1
motor_duty = 0

ticker_flag = False

def time_pit_1ms_handler(time):
    global ticker_flag
    ticker_flag = True

pit1 = ticker(1)
pit1.capture_list(imu, key)
pit1.callback(time_pit_1ms_handler)
pit1.start(10) 

# 屏幕实例化
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

motor_duty_max = 1000

def sec_menu_02(key_data):
    global motor_duty
    lcd.str24(60, 0, "speed", 0x07E0)
    lcd.str16(16, 62, "return", 0xFFFF)
    lcd.str16(16, 30, "motor_duty={}".format(motor_duty), 0xFFFF)
    if key_data == 1:
        motor_duty = motor_duty + 50
        if motor_duty >= motor_duty_max:
            motor_duty = motor_duty_max
    if key_data == 2:
        motor_duty = motor_duty - 50
        if motor_duty <= -motor_duty_max:
            motor_duty = -motor_duty_max

while True:
    time.sleep_ms(100)
    
    if motor_dir:
        motor_duty = motor_duty + 50
        if motor_duty >= motor_duty_max:
            motor_dir = 0
    else:
        motor_duty = motor_duty - 50
        if motor_duty <= -motor_duty_max:
            motor_dir = 1
    

    led1.value(motor_duty < 0)
    # duty 接口更新占空比 范围 ±10000
    motor_l.duty(motor_duty)
    motor_r.duty(motor_duty)
    
    print(f"{motor_duty}")

    gc.collect()

    if (ticker_flag) :
            key_data = key.get()
            ticker_flag = False