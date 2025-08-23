from machine import *
from smartcar import *
from seekfree import *
from display import *
import utime
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
# 实例化 IMU963RA 模块
# 核心板上的LED
led1 = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
# 拨码开关2
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 拨码开关4
switch_3 = Pin('B14', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 拨码开关3
switch_4 = Pin('B15', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 调用 TSL1401 模块获取 CCD 实例
ccd = TSL1401(5)
# 实例化 KEY_HANDLER 模块
key = KEY_HANDLER(10)
imu = IMU963RA()

def scale_value(x, x_min, x_max):
    """
    将输入x从范围[x_min, x_max]线性映射到[0.4, 1]
    输入越大输出越小，输入越小输出越大
    """
    if x_min == x_max:
        return 0.8  # 处理所有输入相同的情况
    normalized = max(0, min(1, (x - x_min) / (x_max - x_min)))
    return 1.0 - 0.6 * normalized


class MOVEMENTTYPE:
    default = 0
    Mode_1 = 10
    Mode_2 = 20
    Mode_3 = 30
    Mode_4 = 40
    Mode_5 = 50
class MovementType:
    def __init__(self):
        self.mode=MOVEMENTTYPE.default
        self.aim_speed=0.0
        self.speed=0.0
    def update(self):
        # 防止速度调参时变化过快直接倒地的pid
        self.speed = speed_control.calculate(self.aim_speed, self.speed)
             
#         if self.mode == MOVEMENTTYPE.default:
#             self.aim_speed=-20
#              
#         elif self.mode == MOVEMENTTYPE.Mode_1:
#             self.aim_speed=-20
#              
#         elif self.mode == MOVEMENTTYPE.Mode_2:
#             self.aim_speed=-30
#  
#         elif self.mode == MOVEMENTTYPE.Mode_3:
#             self.aim_speed=-40
#  
#         elif self.mode == MOVEMENTTYPE.Mode_4:
#             self.aim_speed=-50
#  
#         elif self.mode == MOVEMENTTYPE.Mode_5:
#             self.aim_speed=-60
movementtype=MovementType()

key_data =[0]*4
encl_data = 0  # 左编码器数据
encr_data = 0  # 右数据编码器
#aim_speed = 0  # 之后要可以使用KEY手动修改
out_l = 0  # 左轮输出值
out_r = 0  # 右轮输出值
speed_d = 50  # 速度增量(调试用)

# 赛道元素状态枚举
# 赛道元素状态枚举
class RoadElement:
    stop = -1
    normal = 0
    l1 = 1
    l2 = 2
    r1 = 5
    r2 = 4
    l3_not = 14
    r3_not = 15
    l3 = 3
    r3 = 6
    lin = 7
    lout = 8
    loutcoming = 88
    loutout = 888
    rin = 9
    rout = 10
    routcoming = 89
    routout = 999
    zebrain = 11
    zebraout = 111
    ramp = 12
    barrier = 13
    crossroad_coming = 16

class Distance:
    """行驶距离"""
    def __init__(self):
        self.start_flag = False
        self.data = 0
        self.start()
    def start(self):
        self.start_flag = True
    
    def clear(self):
        self.data=0

    def update(self, tmpdata, k):
        if self.start_flag:
            self.data += tmpdata * k
        if self.data > 999999:
            self.data = 0.0
    def off(self):
        self.data = 0
        self.start_flag = False
element_distance = Distance()
alldistance = Distance()
speed_slow_distance = Distance()
openart_distance = Distance()

class Beeper:
    def __init__(self, beerpin = 'C9'):
        self.beep_pin = Pin(beerpin , Pin.OUT, pull = Pin.PULL_UP_47K, value = False)
        self.start_time = 0       # 鸣叫开始时间戳
        self.duration = 0         # 当前鸣叫持续时间
        self.is_active = False    # 鸣叫状态标志
        self.long_duration = 30  # 长鸣时长(ms)
        self.short_duration = 10 # 短鸣时长(ms)
        self._last_update = 0     # 上次更新时间戳

    def start(self, duration_type):
        """触发蜂鸣器鸣叫
        :param duration_type: 'long' 或 'short'
        """
        self.duration = self.long_duration if duration_type == 'long' else self.short_duration
        self.start_time = utime.ticks_ms()
        self.beep_pin.high()
        self.is_active = True

    def update(self):
        """需在循环中定期调用，建议调用间隔<=10ms"""
        if not self.is_active:
            return
            
        current = utime.ticks_ms()
        elapsed = utime.ticks_diff(current, self.start_time)
        
        # 持续时间达到后关闭
        if elapsed >= self.duration:
            self.beep_pin.low()
            self.is_active = False
            self.duration = 0

    def set_durations(self, long=None, short=None):
        """动态修改鸣叫时长"""
        if long is not None: self.long_duration = long
        if short is not None: self.short_duration = short
beep = Beeper()