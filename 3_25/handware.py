from machine import *
from display import *
from smartcar import *
from seekfree import *
import math
import gc
import time


# 实例化 WIRELESS_UART 模块 参数是波特率
# 无线串口模块需要自行先配对好设置好参数
wireless = WIRELESS_UART(460800)
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
# 实例化 MOTOR_CONTROLLER 电机驱动模块
motor_l = MOTOR_CONTROLLER(
    MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty=0, invert=True)
motor_r = MOTOR_CONTROLLER(
    MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty=0, invert=True)
# 实例化 encoder 模块
encoder_l = encoder("D0", "D1", True)
encoder_r = encoder("D2", "D3")
# 实例化 IMU963RA 模块
imu = IMU963RA()
# 核心板上的LED
led1 = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
# 实例化 KEY_HANDLER 模块
key = KEY_HANDLER(10)
# 拨码开关2
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 拨码开关4
switch_3 = Pin('B14', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 拨码开关3
switch_4 = Pin('B15', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 调用 TSL1401 模块获取 CCD 实例
ccd = TSL1401(3)

# return wireless, lcd, motor_l, motor_r, encoder_l, encoder_r, imu, led1, key, end_switch, switch_3, switch_4, ccd



class SYSTEM:
    def __init__(self):
        self.ACC_SPL = 4096.0
        self.GYRO_SPL = 16.4
        self.key_data = [0] * 4  # 按键数据
        self.ccd_data1 = [0] * 128  # ccd1原始数组
        self.ccd_data2 = [0] * 128  # ccd2原始数组
        self.encl_data = 0  # 左编码器数据
        self.encr_data = 0  # 右数据编码器
        self.aim_speed = 0  # 之后要可以使用KEY手动修改
        self.aim_speed_l = 0  # 左轮期望速度
        self.aim_speed_r = 0  # 右轮期望速度
        self.MedAngle = 64.0
        self.speed_d = 50  # 速度增量(调试用)
        self.last_imu_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0]
        self.imu_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0]
        self.tof_data = 0

        self.left_point_1 = 0
        self.left_point_2 = 0
        self.right_point_1 = 0
        self.right_point_2 = 0
        self.error1 = 0
        self.error2 = 0
        self.mid_point1 = 0
        self.mid_point2 = 0


        self.current_yaw = 0
        self.current_pitch = 0
        self.current_roll = 0

        self.gyrooffsetx = 0
        self.gyrooffsety = 0
        self.gyrooffsetz = 0
        self.accoffsetx = 0
        self.accoffsety = 0
        self.accoffsetz = 0
        self.OFFSETNUM = 100
        
    def update_key(self):
        self.imu_data = [float(x) for x in imu.get()]
    def update_ccd(self):
        pass
    def update_encoder(self):
        self.encl_data = encoder_l.read()
        self.encr_data = encoder_r.read()
    def update_imu(self):
        self.imu_data = [float(x) for x in imu.get()]
    def imu_filter(self):
        # 低通滤波处理（加速度计）
        self.update_imu()
        alpha = 0.5
        for i in range(3):
            # 先进行零偏校正和单位转换
            current_processed = (
                self.imu_data[i] - [self.accoffsetx, self.accoffsety, self.accoffsetz][i]) / self.ACC_SPL
            # 再应用滤波，使用上一次的滤波结果
            self.imu_data[i] = alpha * current_processed + \
                (1 - alpha) * self.last_imu_data[i]
            # 更新历史值为当前滤波结果
            self.last_imu_data[i] = self.imu_data[i]

        # 陀螺仪单位转换（减去偏移后除以灵敏度）
        for i in range(3, 6):
            self.imu_data[i] = math.radians(
                (self.imu_data[i] - [self.gyrooffsetx, self.gyrooffsety, self.gyrooffsetz][i - 3]) / self.GYRO_SPL)

    def imu_offset(self):
        for _ in range(self.OFFSETNUM):
            self.update_imu()
            self.accoffsetx += self.imu_data[0]
            self.accoffsety += self.imu_data[1]
            self.accoffsetz += self.imu_data[2]
            self.gyrooffsetx += self.imu_data[3]
            self.gyrooffsety += self.imu_data[4]
            self.gyrooffsetz += self.imu_data[5]
        self.accoffsetx /= self.OFFSETNUM
        self.accoffsety /= self.OFFSETNUM
        self.accoffsetz /= self.OFFSETNUM
        self.gyrooffsetx /= self.OFFSETNUM
        self.gyrooffsety /= self.OFFSETNUM
        self.gyrooffsetz /= self.OFFSETNUM

sys = SYSTEM()