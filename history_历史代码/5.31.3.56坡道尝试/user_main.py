# 基础库、NXP库、第三方库
import gc
import utime
import math
#from menutext import *
#from tof_hander import *
import utime
from machine import *
from smartcar import *
from seekfree import *
from display import *


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
tof = DL1B()
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

key_data =[0]*4
encl_data = 0  # 左编码器数据
encr_data = 0  # 右数据编码器
#aim_speed = 0  # 之后要可以使用KEY手动修改
out_l = 0  # 左轮输出值
out_r = 0  # 右轮输出值
speed_d = 50  # 速度增量(调试用)
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

def scale_value(x, x_min, x_max):
    """
    将输入x从范围[x_min, x_max]线性映射到[0.4, 1]
    输入越大输出越小，输入越小输出越大
    """
    if x_min == x_max:
        return 0.8  # 处理所有输入相同的情况
    normalized = (x - x_min) / (x_max - x_min)
    return 1.0 - 0.6 * normalized


def create_roll_checker():
    history = []
    def check(current_roll):
        # 将新数据添加到历史记录中
        history.append(current_roll)
        # 保持最多保留最近20个数据点
        if len(history) > 10:
            history[:] = history[-10:]
        # 如果数据不足20个，返回False
        if len(history) < 10:
            return False
        # 统计不满足条件的数据个数
        count = sum(1 for num in history if not (-60.0 < num < 0.0))
        return count >= 8
    return check
checker = create_roll_checker()


print("种族骑士王小桃来啦UwU")
# 单位换算用
ACC_SPL = 4096.0
GYRO_SPL = 16.4

pit_cont_pid = 0

# 定义一个回调函数
ticker_flag_pid = False
ticker_flag_4ms = False
ticker_flag_8ms = False
ticker_flag_menu = False


def time_pit_pid_handler(time):
    global ticker_flag_menu, pit_cont_pid
    pit_cont_pid += 5
    if (pit_cont_pid % 10 == 0):
        pit_cont_pid = 0
        ticker_flag_menu=True


# 实例化 PIT ticker 模块
pit0 = ticker(0)
pit0.capture_list(ccd, key)
pit0.callback(time_pit_pid_handler)
pit0.start(5)


def time_pit_imu_handler(time):
    global ticker_flag_pid
    ticker_flag_pid = True


pit1 = ticker(1)
pit1.capture_list(imu, encoder_l, encoder_r,tof)
pit1.callback(time_pit_imu_handler)
pit1.start(5)

pit_cont_dir = 0



def time_pit_turnpid_handler(time):
    global ticker_flag_4ms, ticker_flag_8ms, pit_cont_dir
    pit_cont_dir += 5
    if (pit_cont_dir % 20 == 0):
        ticker_flag_4ms = True
    if (pit_cont_dir >= 40):
        ticker_flag_8ms = True
        pit_cont_dir = 0


pit3 = ticker(3)
pit3.capture_list()
pit3.callback(time_pit_turnpid_handler)
pit3.start(5)

class TickerProfiler:
    def __init__(self, name, expected_interval_ms):
        self.name = name  # Ticker名称（如 "1ms"）
        self.expected_us = expected_interval_ms * 1000  # 预期间隔（微秒）
        self.last_ticks = 0  # 上一次触发时间戳
        self.first_trigger = True  # 首次触发标志

    def update(self):
        """更新并打印时间间隔（需在每次ticker触发时调用）"""
        current_ticks = utime.ticks_us()

        if not self.first_trigger:
            # 计算实际间隔（自动处理计数器溢出）
            actual_interval_us = utime.ticks_diff(current_ticks, self.last_ticks)

            # 打印带颜色标记的调试信息（可选）
            error = abs(actual_interval_us - self.expected_us)
            status = "OK" if error < self.expected_us * 0.1 else "WARN"
            color_code = "\033[32m" if status == "OK" else "\033[31m"
            print(f"{color_code}[{self.name} Ticker] 预期: {self.expected_us}us, 实际: {actual_interval_us}us\033[0m")

        # 更新状态
        self.last_ticks = current_ticks
        self.first_trigger = False
def create_roll_checker():
    history = []
    def check(current_roll):
        # 将新数据添加到历史记录中
        history.append(current_roll)
        # 保持最多保留最近20个数据点
        if len(history) > 10:
            history[:] = history[-10:]
        # 如果数据不足20个，返回False
        if len(history) < 10:
            return False
        # 统计不满足条件的数据个数
        count = sum(1 for num in history if not (-3600.0 < num < 100.0))
        return count >= 8
    return check
checker = create_roll_checker()
def check_tuple(data, count_up, count_down):
    """统计超过count_up和低于count_down的元素数量，根据阈值返回状态"""
    OVER_THRESHOLD = 110
    over_count = sum(num > count_up for num in data)
    under_count = sum(num < count_down for num in data)
    
    if over_count >= OVER_THRESHOLD:
        return 1
    if under_count >= OVER_THRESHOLD:
        return -1
    return 0

class CCDHandler:
    def __init__(self, channel):
        self.data = [0] * 128
        self.last_mid = 64
        self.mid = 64
        self.left = 0
        self.right = 127
        self.channel = channel
        self.error = 0
        self.follow = 0

    def update(self):
        """更新CCD传感器数据"""
        self.data = ccd.get(self.channel)
        return self.data

    def get_threshold(self):
        """计算动态阈值"""
        relevant_data = self.data[4:123]  # 包含索引4到122
        return (max(relevant_data) + min(relevant_data)) // 2

    def get_mid_point(self, value, reasonrange, follow=0, searchgap=0):
        """获取赛道中线坐标及边界"""
        if  follow != 0:
            self.follow = follow
        self.data = ccd.get(self.channel)  # 获取最新数据
        
        # 当上次中点无效时进行边界搜索
        if self.data[self.last_mid] < self.get_threshold():
            self._handle_invalid_midpoint(searchgap, value)
            self.mid = min(max(self.mid, 5), 122)
            return self.mid
        
        # 常规边界搜索
        self._search_boundaries(searchgap, value)
        
        # 计算中线并应用跟随偏移
        if self.follow != 0:
            self._apply_follow_offset()
            self.mid = (self.left + self.right) // 2
            return self.mid
        
        # 限制中线变化幅度
        self._limit_mid_change(reasonrange)
        
        # 确保中线在有效范围内
        self.mid = min(max(self.mid, 5), 122)

        self.mid = (self.left + self.right) // 2
        return self.mid
    
    def read_mid_point(self, value, reasonrange, follow=0, searchgap=0):
        """获取赛道中线坐标及边界, get改成read"""  
        if self.follow != 0:
            self.follow = follow
        self.data=ccd.read(self.channel)
        # 当上次中点无效时进行边界搜索
        if self.data[self.last_mid] < self.get_threshold():
            self._handle_invalid_midpoint(searchgap, value)
            self.mid = min(max(self.mid, 5), 122)
            return self.mid
        
        # 常规边界搜索
        self._search_boundaries(searchgap, value)
        
        # 计算中线并应用跟随偏移
        self.mid = (self.left + self.right) // 2
        self._apply_follow_offset()
        
        # 限制中线变化幅度
        self._limit_mid_change(reasonrange)
        
        # 确保中线在有效范围内
        self.mid = min(max(self.mid, 5), 122)
        return self.mid

    def _handle_invalid_midpoint(self, search_gap, edge_ratio):
        """处理无效中线时的边界搜索"""
        if self.last_mid > 64:
            self.right = self._search_edge(self.last_mid - search_gap, 0, -1, 4, edge_ratio, 0)
            self.left = self._search_edge(self.right - search_gap, 0, -1, 4, edge_ratio, 0)
        else:
            self.left = self._search_edge(self.last_mid + search_gap, 126, 1, -4, edge_ratio, 127)
            self.right = self._search_edge(self.left + search_gap, 126, 1, -4, edge_ratio, 127)

    def _search_boundaries(self, search_gap, edge_ratio):
        """常规边界搜索逻辑"""
        self.left = self._search_edge(
            self.last_mid - 4 - search_gap, 0, -1, 4, edge_ratio, 0,
        )
        self.right = self._search_edge(
            self.last_mid + 4 + search_gap, 126, 1, -4, edge_ratio, 127,
        )

    def _search_edge(self, start, end, step, offset, ratio_thresh, default):
        """通用边界搜索函数, offset正负需要判断"""
        for i in range(start, end, step):
            # 边界检查
            if not (0 <= i + offset <= 127):
                continue
                
            # 计算差比和
            diff = abs(self.data[i + offset] - self.data[i])
            sum_ = self.data[i + offset] + self.data[i] + 1
            if sum_ == 0:
                continue
                
            if (diff * 100 / sum_) > ratio_thresh:
                return i
        return default

    def _apply_follow_offset(self):    #大于零跟左线，小于零跟右线
        """应用跟随偏移调整边界"""
        if self.follow > 0:
            self.right = self.left + self.follow
        elif self.follow < 0:
            self.left = self.right + self.follow

    def _limit_mid_change(self, max_change):
        """限制中线位置突变"""
        if abs(self.mid - self.last_mid) > max_change:
            self.mid = self.last_mid
        self.last_mid = self.mid
        
ccd_near = CCDHandler(1)
ccd_far=CCDHandler(0)

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


class CCD_Controller:
    """CCD控制器, 远近端, follow跟随偏移"""
    def __init__(self):
        self.error = 0
        self.last_error = 0
        self.far = False  # 是否使用远端CCD
        self.fix_error_value = 0  # 是否固定error值 出圆环时需要
        self.follow = 0
        self.value = 31
        self.ccd_near_length = 60
        self.ccd_far_length = 60
    def get_error(self):
        """获取CCD误差"""
        ccd_far.get_mid_point(value=self.value, reasonrange=128, follow=0, searchgap=0)
        ccd_near.get_mid_point(value=self.value, reasonrange=128, follow=0, searchgap=0)
        if self.fix_error_value != 0:
            return self.fix_error_value   #直接返回
        
        if self.far:
            self.error = ccd_far.mid - 64
        else:
            self.error = ccd_near.mid - 64
        
        if self.follow > 0:
            self.error = (ccd_near.left*2 + self.follow)//2 -64
            return self.error
            
        elif self.follow < 0:
            self.error = (ccd_near.right*2 +self.follow)//2 -64
            return self.error
            
        return self.error
ccd_controller = CCD_Controller()

class ElementDetector:
    """赛道元素检测器"""
    def __init__(self):
        self.prev_state = RoadElement.normal
        self.state = RoadElement.normal
        self.ring_progress = 0  # 圆环进度
        self.zebra_count = 1    # 斑马线有几次是直接过
        self.imu_data = [0] * 9
        self.enc_data = 0
        self.follow = 0
        self.far_or_near = True
        self.tmperror=0   #误差缓存
        self.outflag=0

        # 常量定义（根据实际赛道调整）
        self.ccd_near_l = [9, 60]       # 左圆环阶段1近端CCD左右边点范围
        self.ccd_near_r = [70, 120]
        self.ccd_near_l_lost = 7
        self.ccd_near_r_lost = 120

        self.ccd_far_r = [80, 120]        # 远端CCD右边点范围
        self.ccd_far_l = [20, 50]         # 远端CCD左边点范围
        self.ccd_far_l_lost = 7                 # 远端CCD左丢线阈值
        self.ccd_far_r_lost = 120               # 远端CCD右丢线阈值

        self.POINT_diff_data = 20            # 特征点差异阈值

        self.DISTANCE_ring_2_data = 80
        self.GYRO_Z_ring2_data = 800

        # l3
        self.GYRO_Z_ring3_data = 1000
        self.DISTANCE_ring3_data = 120

        # lin
        self.GYRO_Z_ring_in_data = 1700
        self.DISTANCE_ring_in_data = 150

        self.DISTANCE_ring_out_data = 70
        self.DISTANCE_ring_out_out_data = 200
        self.ccd_near_length = 60
        self.ccd_far_length = 60
        self.DISTANCE_ring_outcoming_data = 160
        self.DISTANCE_ring3_not_data = 300
        self.DISTANCE_zebra_out_data = 40 # 斑马线
        self.ERROR_l_out_value = -13
        #crossroad
        self.DISTANCE_crossroad_data = 80  #十字路口

        #------------------------避障需要的数据----------------------
        self.mid=ccd_near.mid
        self.left=ccd_near.left
        self.right=ccd_near.right
        self.last_lenth=40   #待测，估计值
        self.lenth=self.right-self.left
        #-----------------------------------------------------------

    def debug(self):
        """纠正CCD和陀螺仪数据"""
        temp_ccd_near_data_l = 0
        temp_ccd_near_data_r = 0
        temp_ccd_far_data_l = 0
        temp_ccd_far_data_r = 0
        for _ in range(10):
            ccd_near.update()
            ccd_far.update()
            temp_ccd_near_data_l += ccd_near.left
            temp_ccd_near_data_r += ccd_near.right
            temp_ccd_far_data_l += ccd_far.left
            temp_ccd_far_data_r += ccd_far.right
        delta = 8
        self.ccd_near_l[0] = temp_ccd_near_data_l // 10 - delta
        self.ccd_near_l[1] = temp_ccd_near_data_l // 10 + delta
        self.ccd_near_r[0] = temp_ccd_near_data_r // 10 - delta
        self.ccd_near_r[1] = temp_ccd_near_data_r // 10 + delta
        self.ccd_near_length = abs(self.ccd_near_l[1] - self.ccd_near_l[0])
        self.ccd_far_l[0] = temp_ccd_far_data_l // 10 - delta
        self.ccd_far_l[1] = temp_ccd_far_data_l // 10 + delta
        self.ccd_far_r[0] = temp_ccd_far_data_r // 10 - delta
        self.ccd_far_r[1] = temp_ccd_far_data_r // 10 + delta
        self.ccd_far_length = abs(self.ccd_far_l[1] - self.ccd_far_l[0])
        self.POINT_diff_data = temp_ccd_near_data_l // 10 + delta

    def update(self):
        """主检测函数: , imu_data, enc_data """
        tempcheck = self.state

#         if self.find_barrier() :
#             self.state = RoadElement.barrier
#             if self.find_barrier() == 1:
#                 self.follow = -self.ccd_near_length
#             elif self.find_barrier() == -1:
#                 self.follow = self.ccd_near_length
        # 判断全黑全白
        # #if check_tuple(ccd_near.data, 90, 30)==-1:
        #     #self.state = RoadElement.stop # 跑出去了,别把车子撞坏了,歇歇吧
        # if self.state == RoadElement.stop:# 返回正常状态
        #     if self._check_normal() and movementtype.mode == MOVEMENTTYPE.default:
        #         self.state = RoadElement.normal

        if self._check_zebra():
            self.state = RoadElement.zebrain
            speed_controller.target_speed=0
        if self.state == RoadElement.zebrain:
            if self._check_zebra_out():
                self.zebra_count -= 1
                self.state = RoadElement.normal
                if self.zebra_count < 0:
                    self.state = RoadElement.stop # 完赛啦
        
        # if self._check_normal():
        #         self.state = RoadElement.normal

        if self.state == RoadElement.normal:
            if self._left_1( ):
                self.state = RoadElement.l1

        elif self.state == RoadElement.l1:
            if self._crossroad_coming():
                self.state = RoadElement.normal
            elif self._left_2( ):
                self.state = RoadElement.l2

        # 防误判圆环 important
 
        elif self.state == RoadElement.l2:
            if self._left_3():
                self.state = RoadElement.l3

        # 进圆环 


        elif self.state == RoadElement.l3:
            if movementtype.mode == MOVEMENTTYPE.Mode_1:
                if self._left_in_not():
                    self.state = RoadElement.normal
            elif movementtype.mode == MOVEMENTTYPE.Mode_2:
                if self._left_in():
                    self.state = RoadElement.lin

        # 出圆环

        elif self.state == RoadElement.lin:
            if self._left_outcoming():
                self.state = RoadElement.loutcoming

        elif self.state == RoadElement.loutcoming:
            if self._left_out():
                self.state = RoadElement.lout

        elif self.state == RoadElement.lout:
            if self._left_out_out():
                self.state = RoadElement.loutout
                
        # 出圆环
        if self.state == RoadElement.loutout:
            self.state = RoadElement.normal

        # 在update方法中添加右圆环状态转换：
        if self.state == RoadElement.normal:
            if self._right_1():
                self.state = RoadElement.r1

        elif self.state == RoadElement.r1:
            if self._crossroad_coming():
                self.state = RoadElement.normal
            elif self._right_2():
                self.state = RoadElement.r2

        elif self.state == RoadElement.r2:
            if self._right_3():
                self.state = RoadElement.r3

        elif self.state == RoadElement.r3:
            if movementtype.mode == MOVEMENTTYPE.Mode_1:
                if self._right_in_not():
                    self.state = RoadElement.normal
            elif movementtype.mode == MOVEMENTTYPE.Mode_2:
                if self._right_in():
                    self.state = RoadElement.rin

        elif self.state == RoadElement.rin:
            if self._right_outcoming():
                self.state = RoadElement.routcoming

        elif self.state == RoadElement.routcoming:
            if self._right_out():
                self.state = RoadElement.rout

        elif self.state == RoadElement.rout:
            if self._right_out_out():
                self.state = RoadElement.routout 

        # 十字
        elif self.state == RoadElement.crossroad_coming:
            if self._crossroad_out():
                self.state = RoadElement.normal

        self._element_operations()  # 执行元素状态相关操作
        # if tempcheck != self.state:
        #     beep.start('short')
        return self.state
    
    def _element_operations(self):
        # 如果状态没有变化,直接返回,降低时间复杂度
        if self.prev_state == self.state:
            return
        # 状态变化
        # 防止上次的循迹状态（error, follow）影响当前状态
        ccd_controller.fix_error_value = 0
        ccd_controller.follow = 0
        ccd_controller.far = False
        element_gyro.clear()
        element_distance.clear()
        element_gyro.start()
        element_distance.start()
        if self.state == RoadElement.stop:  # 停止状态
            speed_controller.target_speed = 10
        elif self.state == RoadElement.zebrain:
            ccd_controller.far = True
        elif self.state == RoadElement.normal: # 正常状态
            ccd_controller.fix_error_value = 0
            ccd_controller.follow = 0
            # element_gyro.off()
            # element_distance.off()
        # elif self.state == RoadElement.l1:    #跟右边线
        #     ccd_controller.follow=-self.ccd_near_length
        elif self.state == RoadElement.l1:
            ccd_controller.follow = -self.ccd_near_length
        elif self.state == RoadElement.l2:
            ccd_controller.follow = -self.ccd_near_length

        elif self.state == RoadElement.l3:
            ccd_controller.follow = self.ccd_near_length
        elif self.state == RoadElement.l3_not:
            ccd_controller.follow = -self.ccd_near_length

        elif self.state == RoadElement.lin:
            ccd_controller.follow = 0
            # self.tmperror=stage_error.get_tmp()

        elif self.state == RoadElement.loutcoming:
            ccd_controller.fix_error_value = self.ERROR_l_out_value

        elif self.state == RoadElement.lout:
            ccd_controller.follow = -self.ccd_near_length

        elif self.state == RoadElement.crossroad_coming:
            ccd_controller.fix_error_value = 0
        elif self.state == RoadElement.r1:
            ccd_controller.follow = self.ccd_near_length
            
        elif self.state == RoadElement.r2:
            ccd_controller.follow = self.ccd_near_length

        elif self.state == RoadElement.r3:
            ccd_controller.follow = -self.ccd_near_length

        elif self.state == RoadElement.rin:
            ccd_controller.follow = 0

        elif self.state == RoadElement.routcoming:
            ccd_controller.fix_error_value = -self.ERROR_l_out_value  # 注意符号取反

        elif self.state == RoadElement.rout:
            ccd_controller.follow = self.ccd_near_length

        elif self.state == RoadElement.routout:
            self.state = RoadElement.normal

        self.prev_state=self.state

    def _left_1(self):
        """左圆环检测逻辑"""
        # 近端CCD特征检查   近端左右边线都正常
        near_valid = (self.ccd_near_l[0] <=  ccd_near.left <= self.ccd_near_l[1] and 
                     self.ccd_near_r[0] <=  ccd_near.right <= self.ccd_near_r[1])
        
        # 远端CCD特征检查   远端左丢线，右边正常
        far_valid = (ccd_far.left < self.ccd_far_l_lost and 
                    self.ccd_far_r[0] <= ccd_far.right <= self.ccd_far_r[1])
        
        # 特征点一致性检查  检测远近端右边线是否为直线，防止误判左弯道
        point_diff = abs(ccd_near.right -  ccd_far.right)
        
#         print("{}   {}   {}".format(near_valid  ,far_valid ,point_diff))
        return near_valid and far_valid and point_diff < self.POINT_diff_data
        #return ccd_near.left < self.ccd_far_l_lost
    
            

    def _left_2(self):
        """左圆环状态2检测：近端左丢线+特征点稳定"""
        # 近端CCD左丢线检查（left_point_2 <=10）
        near_left_lost = ccd_near.left <= self.ccd_near_l_lost
        
        # 近端右边界有效性检查（87 <= right_point_2 <=103）
        near_right_valid = self.ccd_near_r[0] <=  ccd_near.right <= self.ccd_near_r[1]
        
        # 特征点稳定性检查（|right_point_1 - right_point_2| <=12）
        point_diff = abs(ccd_far.right -  ccd_near.right)
        
#       print("{}   {}   {}   ".format(near_left_lost , near_right_valid  ,point_diff))
#         if abs(element_distance.data) > self.DISTANCE_ring_2_data or abs(element_gyro.data) > self.GYRO_Z_ring2_data:
#             self.state = RoadElement.normal
        return near_left_lost and near_right_valid and point_diff < self.POINT_diff_data
        # return ccd_near.left < self.ccd_near_l_lost


  
    def _check_zebra(self):
        """斑马线检测逻辑"""
        crossings = 0         # 跳变次数统计
        threshold = 31
        min_crossings = 10    #最小的斑马线检测点次数，待测
        for i in range(30, 97):    #在靠近赛道中间的部分检测，待测
            diff = abs(ccd_near.data[i] - ccd_near.data[i + 3])*100
            sum = ccd_near.data[i] + ccd_near.data[i + 3] + 1
            ratio = abs(diff / sum)
            if ratio > threshold:  # 跳变阈值
                crossings += 1
        if crossings > min_crossings:
            return True
        return False
    
    def _check_zebra_out(self):
        if abs(element_distance.data) > self.DISTANCE_zebra_out_data:
            return True
        return False
    
    def _check_normal(self):
        if ccd_near.left > self.ccd_near_l_lost and ccd_near.right < self.ccd_near_r_lost:
            return True
        return False


    def _left_3(self):
        """left圆环状态3检测"""
        if abs(element_distance.data) > self.DISTANCE_ring3_data:  # 距离方向取反
            if -self.GYRO_Z_ring3_data < element_gyro.data < self.GYRO_Z_ring3_data:
                if self.ccd_near_l[0] < ccd_near.left < self.ccd_near_l[1]:
                    if abs(ccd_near.right - ccd_far.right) < self.POINT_diff_data:
                        return True
                    else:
                        self.state = RoadElement.normal
        if abs(element_gyro.data) > self.GYRO_Z_ring3_data:
            self.state = RoadElement.normal
        return False

    def _left_in_not(self):
        # 近端CCD左丢线检查（left_point_2 <= ...）
        near_left_lost =  ccd_near.left <= self.ccd_near_l_lost
        
        # 近端右边界有效性检查（右边界范围检查）
        near_right_valid = self.ccd_near_r[0] <=  ccd_near.right <= self.ccd_near_r[1]
        
        # 特征点稳定性检查（右远和右近的差值）
        point_diff = abs(ccd_far.right -  ccd_near.right)
        
        if near_left_lost and near_right_valid and (point_diff <= self.POINT_diff_data) and \
            element_distance.data > self.DISTANCE_ring3_not_data:
            return True

    def _left_in(self):
        # 陀螺仪极性取反（原右转检测正方向，左转检测负方向）
        if abs(element_gyro.data) > self.GYRO_Z_ring_in_data or abs(element_distance.data) > self.DISTANCE_ring_in_data:
            return True

    def _left_outcoming(self):
        # 超过一定距离并且全白
        if abs(element_distance.data) > self.DISTANCE_ring_outcoming_data:
            if (ccd_near.left<self.ccd_near_l_lost and ccd_near.right>self.ccd_near_r_lost)or (ccd_far.left <self.ccd_far_l_lost and ccd_far.right >self.ccd_far_r_lost):
                return True
        if abs(element_distance.data) > self.DISTANCE_ring3_data * 3.5:
            self.state = RoadElement.normal

            
    def _left_out_out(self):
        if abs(element_distance.data) > self.DISTANCE_ring_out_out_data:
            if ( ccd_near.right < self.ccd_near_r_lost):
                return True
        if abs(element_distance.data) > self.DISTANCE_ring3_data * 1.5:
            self.state = RoadElement.normal
            
    def _left_out(self):
        if abs(element_distance.data) > self.DISTANCE_ring_out_data:
            if ccd_near.right < self.ccd_near_r_lost:
                return True
    def _right_1(self):
        """右圆环检测逻辑（对称于_left_1）"""
        # 近端CCD特征检查（左右镜像）
        near_valid = (self.ccd_near_l[0] <= ccd_near.left <= self.ccd_near_l[1] and 
                    self.ccd_near_r[0] <= ccd_near.right <= self.ccd_near_r[1])
        
        # 远端CCD特征检查（左右镜像）
        far_valid = (ccd_far.right > self.ccd_far_r_lost and 
                    self.ccd_far_l[0] <= ccd_far.left <= self.ccd_far_l[1])
        
        # 特征点一致性检查（比较左边缘）
        point_diff = abs(ccd_near.left - ccd_far.left)
        
        return near_valid and far_valid and point_diff < self.POINT_diff_data

    def _right_2(self):
        """右圆环状态2检测（对称于_left_2）"""
        # 近端CCD右丢线检查
        near_right_lost = ccd_near.right >= self.ccd_near_r_lost
        
        # 近端左边界有效性检查
        near_left_valid = self.ccd_near_l[0] <= ccd_near.left <= self.ccd_near_l[1]
        
        # 特征点稳定性检查
        point_diff = abs(ccd_far.left - ccd_near.left)
        
        return near_right_lost and near_left_valid and point_diff < self.POINT_diff_data

    def _right_3(self):
        """右圆环状态3检测（对称于_left_3）"""
        if abs(element_distance.data) > self.DISTANCE_ring3_data:
            if -self.GYRO_Z_ring3_data < element_gyro.data < self.GYRO_Z_ring3_data:
                if self.ccd_near_r[0] < ccd_near.right < self.ccd_near_r[1]:
                    if abs(ccd_near.left - ccd_far.left) < self.POINT_diff_data:
                        return True
                    else:
                        self.state = RoadElement.normal
        if abs(element_gyro.data) > self.GYRO_Z_ring3_data:
            self.state = RoadElement.normal
        return False

    def _right_in_not(self):
        """右圆环防误判（对称于_left_in_not）"""
        near_right_lost = ccd_near.right >= self.ccd_near_r_lost
        near_left_valid = self.ccd_near_l[0] <= ccd_near.left <= self.ccd_near_l[1]
        point_diff = abs(ccd_far.left - ccd_near.left)
        
        if near_right_lost and near_left_valid and point_diff <= self.POINT_diff_data and \
           element_distance.data > self.DISTANCE_ring3_not_data:
            return True

    def _right_in(self):
        """进入右圆环（对称于_left_in）"""
        if abs(element_gyro.data) > self.GYRO_Z_ring_in_data or abs(element_distance.data) > self.DISTANCE_ring_in_data:
            return True

    def _right_outcoming(self):
        """准备出右圆环（对称于_left_outcoming）"""
        if abs(element_distance.data) > self.DISTANCE_ring_outcoming_data:
            if (ccd_near.left<self.ccd_near_l_lost and ccd_near.right>self.ccd_near_r_lost)or (ccd_far.left <self.ccd_far_l_lost and ccd_far.right >self.ccd_far_r_lost):
                return True
        if abs(element_distance.data) > self.DISTANCE_ring3_data * 3.5:
            self.state = RoadElement.normal

    def _right_out(self):
        """出右圆环（对称于_left_out）"""
        if abs(element_distance.data) > self.DISTANCE_ring_out_data:
            if ccd_near.left > self.ccd_near_l_lost:
                return True

    def _right_out_out(self):
        """完全出右圆环（对称于_left_out_out）"""
        if abs(element_distance.data) > self.DISTANCE_ring_out_out_data:
            if ccd_near.left > self.ccd_near_l_lost :
                return True
        if abs(element_distance.data) > self.DISTANCE_ring3_data * 1.5:
            self.state = RoadElement.normal
#     def find_barrier(self):
#         """障碍物检测"""
#         self.last_lenth=self.lenth
#         self.mid,self.left,self.right=ccd_near.get_mid_point(value =31, reasonrange = 128, follow = 0, searchgap = 0)
#         self.lenth=self.right-self.left
#         widthRate = abs(self.lenth - self.last_lenth) / self.last_lenth if self.last_lenth != 0 else 0
#         threshold = 0.3
# 
#         if widthRate > threshold:
#             if self.mid < 64:
#                 return 1   #障碍物在右边，小车贴左边线
#             else:
#                 return -1  #障碍物在左边，小车贴右边线
#         else:
#             return 0      #没检测到障碍物
            

    # 十字判断
    def _crossroad_coming(self):
        near_valid = (self.ccd_near_l[0] <=  ccd_near.left <= self.ccd_near_l[1] and 
                self.ccd_near_r[0] <=  ccd_near.right <= self.ccd_near_r[1])
        far_ccd_lost=(ccd_far.left < self.ccd_far_l_lost) and (ccd_far.right > self.ccd_far_r_lost)
        return near_valid and far_ccd_lost
    def _crossroad_out(self):
        near_ccd_normal= (ccd_near.left >self.ccd_near_l_lost and ccd_near.right < self.ccd_near_r_lost)
        if near_ccd_normal and element_distance.data > self.DISTANCE_crossroad_data:
            return True
        
    # def _update_state(self, element, imu_data):
    #     """状态机更新"""
    #     if element == RoadElement.circle_l1:
    #         if self.state != RoadElement.circle_l2:
    #             self.ring_progress = 0
    #             self.state = RoadElement.circle_l1
                
    #     elif element == RoadElement.circle_l2:
    #         self.ring_progress += abs(imu_data[5]) * 0.002  # 积分角速度计算进度
    #         if self.ring_progress >= 360:  # 完成一圈
    #             self.state = RoadElement.normal
                
    #     # 其他状态更新...
elementdetector = ElementDetector()
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
        if self.data > 9999999999:
            self.data = 0.0
    def off(self):
        self.data = 0
        self.start_flag = False
element_distance = Distance()
alldistance = Distance()
lampdistance= Distance()

class Speed_controller:
    def __init__(self):
        self.target_speed=0
        self.real_speed=0
        self.fast_speed=-60
        self.slow_speed=-33
    def update(self):
        if abs(alldistance.data) <2000:
            self.target_speed=self.slow_speed
        elif 6000>abs(alldistance.data) > 4200:
            self.target_speed=-180
        elif abs(alldistance.data)>=6000:
            self.target_speed=fast_speed
        elif abs(alldistance.data) > 8000:
            self.target_speed=self.slow_speed
        else:
            self.target_speed=self.fast_speed
        speed_kd=scale_value(abs(ccd_near.mid-ccd_far.mid),0,15)
        self.real_speed=int(self.target_speed *speed_kd)
        return self.real_speed

speed_controller=Speed_controller()


class Gyro_Z_Test:
    """陀螺仪Z轴积分"""
    def __init__(self):
        self.start_flag = False
        self.offset = [0.0] * 9
        self.data = 0.0
        self.start()
        # self._getoffset()
    # def _getoffset(self, num = 50):
    #     for _ in range(num):
    #         imu_data = imu.read()
    #         for i in range(6):
    #             self.offset[i] += imu_data[i]
    #     for i in range(6):
    #         self.offset[i] /= num
    def clear(self):
        self.data=0

    def start(self):
        self.start_flag = True
    def update(self, tmpdata, k):
        if self.start_flag:
            self.data += (tmpdata + 142) * k
        if self.data > 999999999999:
            self.data = 0.0
    def off(self):
        self.data = 0
        self.start_flag = False
element_gyro = Gyro_Z_Test()
debuggyroz = Gyro_Z_Test()

class Error_test:
    #求误差平均值
    def __init__(self):
        self.data=0.0
    def get_tmp(self):
        for _ in range(10):
            tmp_mid = ccd_near.read_mid_point(value =31, reasonrange = 30, follow = 0, searchgap = 0)
            tmp_error=tmp_mid - 64
            self.data +=tmp_error
        self.tmperror=self.data/10
    def reset(self):
        self.data=0.0
stage_error=Error_test()



print("王小桃快跑，邮箱来了")


# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

movementtype.aim_speed=0

# 在主程序初始化阶段创建实例
profiler_1ms = TickerProfiler("5ms", expected_interval_ms=5)
profiler_5ms = TickerProfiler("5ms", expected_interval_ms=5)
profiler_4ms = TickerProfiler("20ms", expected_interval_ms=20)
profiler_8ms = TickerProfiler("40ms", expected_interval_ms=40)

last_imu_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0]
data_wave = [0, 0, 0, 0, 0, 0, 0, 0]

key_data = key.get()
imu_data = imu.get()
imu_data_filtered = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0]
speed_err_k = 1

def clearall():
    key.clear(1)
    key.clear(2)
    key.clear(3)
    key.clear(4)
stop_flag = 1
gyro_bias_x = -726
gyro_bias_y = 16.15
gyro_bias_z = -140.56
# def get_offset():
#     global gyro_bias_y, gyro_bias_z , gyro_bias_x
#     for _ in range(100):
#         imu_data = imu.read()
#         gyro_bias_y += imu_data[3]
#         gyro_bias_x += imu_data[4]
#         gyro_bias_z += imu_data[5]
#     gyro_bias_y /= 100
#     gyro_bias_z /= 100
# get_offset()
class Tof_hander:
    def __init__(self):
        self.data= 0
        self.flag=0
        self.lampdis=300
    def update(self):
        self.data = tof.get()
        if self.data<800 and self.flag == 0:
            lampdistance.clear()
            speed_controller.fast_speed= -180
            self.flag=1
        if self.data>8000 and self.flag == 1:
            speed_controller.fast_speed= -20
            self.flag=2
        if abs(lampdistance.data) > self.lampdis and self.flag == 2:
            speed_controller.fast_speed = -80
tof_hander=Tof_hander()


def update_filter(acc_x, gyro_y):
    global current_angle
    alpha = 0.95
    pitch_gyro = current_angle + gyro_y * 0.005
    current_angle = alpha * acc_x + (1 - alpha) * pitch_gyro
    return current_angle


# ----------------- PID算法 -----------------
#位置式
def pid_controller(now, target, kp, ki, kd, sum_err, last_err):
    error = target - now
    sum_err += error
    derivative = error - last_err
    output = kp * error + ki * sum_err + kd * derivative
    return output, sum_err, error

#增量式
# def pid_increment(now, target, kp, ki, kd, last_err, prev_err):
#     error = target - now
#     increment = kp*(error - last_err) + ki*error + kd*(error - 2*last_err + prev_err)
#     last_err = error
#     prev_err = last_err
#     return increment, last_err, prev_err
def pid_increment(now, target, kp, ki, kd, last_err, prev_err):
    error = target - now
    increment = kp*(error - last_err) + ki*error + kd*(error - 2*last_err + prev_err)
    
    new_prev_err = last_err  # 保存旧的last_err
    new_last_err = error     # 更新为当前error
    
    return increment, new_last_err, new_prev_err
# ----------------- 全局变量 -----------------

pitch_vel = 0
vel_disturbance = 0
last_turn_out = 0
real_speed = 0
acc_x = 0
gyro_y = 0
pwm_l = 0
pwm_r = 0
acc_z = 0
last_error2 = 0


target_speed = -37
balance_angle = -2850
vel_kp = 3.71
vel_ki = 2.84
vel_kd = 0
angle_kp = 0.103
angle_ki = 0
angle_kd = 0
speed_kp = -2.03
speed_ki = 0
speed_kd = 0

now_speed = 0
counter_speed = 0
counter_angle = 0
speed_sum_error = 0
speed_last_error = 0
angle_sum_error = 0
angle_last_error = 0
vel_last_error = 0
vel_prev_error = 0
turn_sum_error = 0
turn_last_error = 0
turn_output = 0
pwm = 0
current_angle = 0

angle_disturbance = 0
pwm_l_value = 0
pwm_r_value = 0
real_speed = 0
# ----------------- 控制回调函数 -----------------
def vel_loop_callback(pit1):
    global pwm_l_value, pwm_r_value
    global pwm, current_angle, turn_output
    global vel_last_error, vel_prev_error
    global turn_sum_error, turn_last_error
    global vel_kp, vel_ki, vel_kd
    global angle_disturbance
    global speed_kp, speed_ki, speed_kd
    global speed_sum_error, speed_last_error
    global balance_angle, angle_kp, angle_ki, angle_kd
    global angle_sum_error, angle_last_error
    global yaw_vel, counter_speed, counter_angle
    global imu, gyro_bias_y, gyro_bias_z
    global motor_l, motor_r,encl_data,encr_data
    global imu_data, now_speed 
    global vel_disturbance  # Add this line

    imu_data = imu.get()
    acc_x = imu_data[1]
    acc_z = imu_data[2]
    gyro_y = imu_data[3]
    tempgyro_z = imu_data[5]

    current_angle = update_filter(acc_x, gyro_y)
    yaw_vel = (tempgyro_z - gyro_bias_z) * 0.07
    pitch_vel = (gyro_y - gyro_bias_y) * 0.07

    counter_speed += 1
    if counter_speed >= 5:
        counter_speed = 0
        left = -encoder_l.get()
        right = encoder_r.get()
        encl_data=left
        encr_data=right

#         max_speed = 2500
#         left = max(min(left, max_speed), -max_speed)
#         right = max(min(right, max_speed), -max_speed)
        
        #now_speed = now_speed*0.1 + ((left + right) / 2)*0.9
        now_speed=(left+right)/2.0
        angle_disturbance, speed_sum_error, speed_last_error = pid_controller(
            now_speed, speed_controller.real_speed, 
            speed_kp, speed_ki, speed_kd,
            speed_sum_error, speed_last_error
        )
#        angle_disturbance = max(min(angle_disturbance, 450), -450)

    # Ensure vel_disturbance is initialized before use
    if 'vel_disturbance' not in globals():
        vel_disturbance = 0

    target_angle = balance_angle - angle_disturbance

    # 外环控制（每5ms执行）
    counter_angle += 1
    if counter_angle >= 2:
        counter_angle = 0
        vel_disturbance, angle_sum_error, angle_last_error = pid_controller(
            current_angle, target_angle,
            angle_kp, angle_ki, angle_kd,
            angle_sum_error, angle_last_error
        )
        #vel_disturbance = max(min(vel_disturbance, 60), -60)

    target_vel = vel_disturbance
    # 内环控制
    delta_pwm, vel_last_error, vel_prev_error = pid_increment(
        pitch_vel, target_vel,
        vel_kp, vel_ki, vel_kd, 
        vel_last_error, vel_prev_error
    )

    pwm += delta_pwm
    pwm = max(min(pwm, 6000), -6000)
    
    turn_output = 0
    pwm_output = pwm  # 极性修改

    pwm_l_value = pwm_output + turn_output
    pwm_r_value = pwm_output - turn_output
    
    
def my_limit(value, min_val, max_val):
    return max(min_val, min(value, max_val))   
# 新增转向控制相关变量
target_turn_angle = 0.0       # 目标转向角度（由遥控器设置）
current_turn_angle = 0.0      # 当前转向角度（通过陀螺仪积分）
target_yaw_vel = 0.0          # 外环输出的目标角速度
turn_out_sum_error = 0.0      # 外环积分累积
turn_in_sum_error = 0.0       # 内环积分累积
counter_turn_out = 0
counter_turn_in = 0
turn_out_last_error = 0
turn_in_last_error = 0
turn_out_kp = -119.73
turn_out_ki = 0
turn_out_kd = 0
turn_in_kp = -2.8
turn_in_ki = 0
turn_in_kd = 0
turn_in_disturbance = 0.0
error = 0
# ----------------- 转向控制回调函数 -----------------
def turn_loop_callback(pit1):
    global current_turn_angle, target_yaw_vel, turn_output
    global turn_out_sum_error, turn_out_last_error, turn_in_sum_error, turn_in_last_error
    global counter_turn_out, yaw_vel
    global turn_out_kp, turn_out_ki, turn_out_kd
    global turn_in_kp, turn_in_ki, turn_in_kd
    global turn_in_disturbance
    global counter_turn_in
    
    # turn外环
    counter_turn_out += 5
    if counter_turn_out >= 40:
        counter_turn_out = 0
        turn_in_disturbance, turn_out_sum_error, turn_out_last_error = pid_controller(
            error, 0,
            turn_out_kp, turn_out_ki, turn_out_kd,
            turn_out_sum_error, turn_out_last_error
        )
        #turn_in_disturbance = max(min(turn_in_disturbance, 3999), -3999)

    
    # turn内环
    counter_turn_in += 5
    if counter_turn_in >= 20:
        counter_turn_in = 0
        imu_data = imu.get()
        turn_output, turn_in_sum_error, turn_in_last_error = pid_controller(
            turn_in_disturbance, imu_data[4] - gyro_bias_x,
            turn_in_kp, turn_in_ki, turn_in_kd,
            turn_in_sum_error, turn_in_last_error
        )
        #turn_output = max(min(turn_output, 3999), -3999)

    return turn_output
# def change_target_speed(distance):
#     global target_speed
#     if distance < 2000:
#         target_speed = -37
#     elif distance > 8000:
#         target_speed = -37
#     else:
#         target_speed = -50
def death_pwm(value):
    if value > 0:
        return value + 650
    else:
        return value - 650
stop_flag = 1
movementtype.mode=MOVEMENTTYPE.Mode_2
elementdetector.state = RoadElement.normal
alldistance.start()
print("""   ____   _           _   _           /\/|
  / ___| (_)   __ _  | | | |   ___   |/\/ 
 | |     | |  / _` | | | | |  / _ \       
 | |___  | | | (_| | | | | | | (_) |      
  \____| |_|  \__,_| |_| |_|  \___/       """)
while True:
    error=ccd_controller.get_error()+6
    elementdetector.update()
#     if elementdetector.state==RoadElement.stop:
#         stop_flag=0
    if end_switch.value() == 1:
        break  # 跳出判断
        
    if (ticker_flag_pid):
        # profiler_gyro.update()
        imu_data = imu.get()
        element_gyro.update(imu_data[5],0.01)
        element_distance.update(encl_data+encr_data,0.01)
        alldistance.update(encl_data+encr_data,0.01)
        #debug += (encoder_l.get() - encoder_r.get()) * 0.01
        vel_loop_callback(pit1)
        turn_loop_callback(pit1)
        speed_controller.update()
        #tof_hander.update()
        motor_l.duty(my_limit(death_pwm(pwm_l_value - turn_output),-6000,6000) * stop_flag)
        motor_r.duty(my_limit(death_pwm(pwm_r_value + turn_output),-6000,6000) * stop_flag)
        ticker_flag_pid = False
        
    if (ticker_flag_menu):
        #menu(key_data)
        key_data = key.get()
        if key_data[0] or key_data[1] or key_data[2] or key_data[3]:
            element_distance.clear()
            element_gyro.clear()
            stop_flag=1
            clearall()
        ticker_flag_menu=False

    if (ticker_flag_8ms):
        # profiler_8ms.update()
        if checker(current_angle):
            stop_flag = 0
        data_flag = wireless.data_analysis()
        for i in range(0, 8):
            # 判断哪个通道有数据更新
            if (data_flag[i]):
                # 数据更新到缓冲
                data_wave[i] = wireless.get_data(i)
                # 将更新的通道数据输出到 Thonny 的控制台
                print("Data[{:<6}] updata : {:<.3f}.\r\n".format(i, data_wave[i]))
                if i==0:
                    target_speed = data_wave[i]
                # 根据通道号单独更新对应参数

#                 if i == 0:
#                     vel_kp = data_wave[i]
#                 elif i == 1:
#                     vel_ki = data_wave[i]
#                 elif i == 2:
#                     vel_kd = data_wave[i]
#                 elif i == 3:
#                     angle_kp = data_wave[i]
#                 elif i == 4:
#                     angle_ki = data_wave[i]
#                 elif i == 5:
#                     angle_kd = data_wave[i]
#                 elif i == 6:
#                     speed_kp = data_wave[i]
#                 elif i == 7:
#                     balance_angle = data_wave[i]
#                 if i == 0:
#                     elementdetector.GYRO_Z_ring3_data = data_wave[i]
#                 elif i == 1:
#                     elementdetector.DISTANCE_ring3_data = data_wave[i]
#                 elif i == 2:
#                     target_speed = data_wave[i]
#                 elif i == 3:
#                     ccd_controller.value = data_wave[i]
#                 elif i == 4:
#                     turn_out_ki = data_wave[i]
#                 elif i == 5:
#                     turn_out_kd = data_wave[i]
#                 elif i == 6:
#                     elementdetector.state = data_wave[i]
#                 elif i == 7:
#                     balance_angle = data_wave[i]
                if i == 0:
                    turn_in_kp = data_wave[i]
                elif i == 1:
                    turn_out_kp = data_wave[i]
                elif i == 2:
                    speed_controller.fast_speed = data_wave[i]
                elif i == 3:
                    ccd_controller.value = data_wave[i]
                elif i == 4:
                    turn_out_ki = data_wave[i]
                elif i == 5:
                    turn_out_kd = data_wave[i]
                elif i == 6:
                    elementdetector.state = data_wave[i]
                elif i == 7:
                    balance_angle = data_wave[i]

        # 将数据发送到示波器
        wireless.send_ccd_image(WIRELESS_UART.ALL_CCD_BUFFER_INDEX)
        wireless.send_oscilloscope(
        #     #vel_kp, vel_ki, vel_kd, angle_kp, vel_disturbance, angle_disturbance, motor_l.duty(), current_angle
              alldistance.data,elementdetector.state,speed_controller.target_speed,tof_hander.flag,lampdistance.data
              ,speed_controller.real_speed
        #     #imu_data[3], imu_data[4], imu_data[5]
        #     #turn_in_disturbance,turn_output, error
        #     #gyro_bias_x , gyro_bias_y, gyro_bias_z
             )
        gc.collect()
        ticker_flag_8ms = False








