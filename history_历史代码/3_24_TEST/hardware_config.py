from machine import *
from display import *
from smartcar import *
from seekfree import *

# 硬件初始化配置
def init_hardware():
    # 无线模块
    wireless = WIRELESS_UART(460800)
    
    # 显示屏
    cs = Pin('C5', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
    rst = Pin('B9', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
    dc = Pin('B8', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
    blk = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
    drv = LCD_Drv(SPI_INDEX=1, BAUDRATE=60000000, 
                 DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
    lcd = LCD(drv)
    lcd.color(0xFFFF, 0x0000)
    lcd.mode(2)
    lcd.clear(0x0000)
    
    # 电机和编码器
    motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000)
    motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000)
    encoder_l = encoder("D0", "D1", True)
    encoder_r = encoder("D2", "D3")
    
    # 传感器和输入设备
    imu = IMU963RA()
    key = KEY_HANDLER(10)
    Go = Pin('C21', Pin.IN, pull=Pin.PULL_UP_47K, value=0)
    end_switch = Pin('C19', Pin.IN)
    ccd = TSL1401(3)
    
    return {
        'wireless': wireless,
        'lcd': lcd,
        'motor_l': motor_l,
        'motor_r': motor_r,
        'encoder_l': encoder_l,
        'encoder_r': encoder_r,
        'imu': imu,
        'key': key,
        'ccd': ccd
    }