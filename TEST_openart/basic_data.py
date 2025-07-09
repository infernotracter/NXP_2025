from machine import *
from smartcar import *
from seekfree import *
from display import *
import math
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
#tof = DL1B()
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



class Tof_hander:
    def __init__(self):
        self.data = 0
        self.state = False  # 初始状态设为False
        self.data_history = []  # 用于存储最近100次数据
        
    def update(self):
        # 获取最新数据
        self.data = tof.get()
        
        # 将新数据添加到历史记录中
        self.data_history.append(self.data)
        
        # 保留最近100次数据
        if len(self.data_history) > 10:
            self.data_history = self.data_history[-10:]
        
        # 当有足够数据时检查条件
        if len(self.data_history) >= 10:
            # 计算小于800的数据数量
            count_below_800 = sum(1 for value in self.data_history if value < 800)
            # 如果至少90次小于800则更新状态
            if count_below_800 >= 5:
                self.state = True
            else:
                self.state = False
        # 当数据不足100时保持状态不变
tof_hander=Tof_hander()


class Beeper:
    def __init__(self, beerpin = 'C9'):
        self.beep_pin = Pin(beerpin , Pin.OUT, pull = Pin.PULL_UP_47K, value = False)
        self.start_time = 0       # 鸣叫开始时间戳
        self.duration = 0         # 当前鸣叫持续时间
        self.is_active = False    # 鸣叫状态标志
        self.long_duration = 60  # 长鸣时长(ms)
        self.short_duration = 30 # 短鸣时长(ms)
        self._last_update = 0     # 上次更新时间戳

    def start(self, duration_type):
        """触发蜂鸣器鸣叫:param duration_type: 'long' 或 'short'
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







'''
------------------------------------
下为openart的串口配置
------------------------------------
'''
# 串口配置 - 使用UART1 (LPUART2)
uart3 = UART(3, 115200)

# 颜色列表 - 必须与摄像头程序完全一致
COLOR_NAMES = [
    "red", "green", "yellow", "orange", 
    "purple", "pink", "cyan", "brown"
]
def parse_detection_packet(packet):
    """
    解析检测数据包
    格式: AA [颜色ID(1) x(1) y(1) w(1) h(1)]... 55
    """
    if len(packet) < 3 or packet[0] != 0xAA or packet[-1] != 0x55:
        print("无效包格式:", packet)
        return []
    
    objects = []
    # 计算实际数据长度 (去掉头尾)
    data_len = len(packet) - 2
    
    # 检查数据完整性 (每组检测数据5字节)
    if data_len % 5 != 0:
        print(f"数据不完整: len={len(packet)} data_len={data_len}")
        return []
    
    # 解析每个检测对象
    for i in range(0, data_len, 5):
        start_idx = 1 + i  # 跳过包头
        
        color_idx = packet[start_idx]
        x = packet[start_idx + 1]
        y = packet[start_idx + 2]
        w = packet[start_idx + 3]
        h = packet[start_idx + 4]
        
        # 验证颜色索引范围
        color_name = COLOR_NAMES[color_idx] if color_idx < len(COLOR_NAMES) else f"未知({color_idx})"
        
        objects.append({
            "color": color_name,
            "x": x,
            "y": y,
            "width": w,
            "height": h
        })
    
    return objects

def read_detection_data():
    # 检查接收缓冲区
    buf_len = uart3.any()
    if buf_len:
        buf = uart3.read(buf_len)
        print(f"原始数据 (len={buf_len}):", buf)
        
        # 尝试解析数据包
        detected_objects = parse_detection_packet(buf)
        
        if detected_objects:
            # print(f"解析到 {len(detected_objects)} 个物体:")
            for obj in detected_objects:
                if (obj['color'] == 'pink' or obj['color'] == 'yellow') and obj['width'] * obj['height'] > 100:
                    beep.start('long')
                    

                # print(f"  颜色: {obj['color']}, 位置: ({obj['x']},{obj['y']})", end="")
                # print(f", 大小: {obj['width']}x{obj['height']}")
                
            # 可改为只回送解析结果
            # response = f"收到{len(detected_objects)}个对象"
            # uart3.write(response.encode())






# 全局接收缓冲区
PACKET_BUFFER = bytearray()

def find_frame(data, start_idx=0):
    """在缓冲区内查找完整帧"""
    while start_idx < len(data):
        # 查找包头
        header_pos = data.find(b'\xAA', start_idx)
        if header_pos == -1:
            return -1, -1
            
        # 查找包尾
        footer_pos = data.find(b'\x55', header_pos + 1)
        if footer_pos == -1:
            return header_pos, -1  # 有头无尾
            
        return header_pos, footer_pos
        
    return -1, -1  # 没有完整帧

def process_incoming_data(new_data):
    """处理新接收的数据，提取完整帧"""
    global PACKET_BUFFER
    
    # 添加新数据到缓冲区
    PACKET_BUFFER.extend(new_data)
    packets = []
    
    start_idx = 0
    while True:
        # 查找帧位置
        header_pos, footer_pos = find_frame(PACKET_BUFFER, start_idx)
        
        if header_pos == -1:
            # 无包头，清空缓冲区
            PACKET_BUFFER = bytearray()
            break
            
        if footer_pos == -1:
            # 有包头但无包尾，保留未处理数据
            PACKET_BUFFER = PACKET_BUFFER[header_pos:]
            break
            
        # 提取完整帧
        frame = PACKET_BUFFER[header_pos:footer_pos+1]
        packets.append(frame)
        
        # 继续查找后续帧
        start_idx = footer_pos + 1
        
        # 如果已经处理完所有数据
        if start_idx >= len(PACKET_BUFFER):
            PACKET_BUFFER = bytearray()
            break
    
    return packets

def parse_detection_packet_new(packet):
    """
    解析检测数据包
    格式: AA [颜色ID(1) x(1) y(1) w(1) h(1)]... 55
    """
    # 验证最小帧大小
    if len(packet) < 7:  # AA + 最小对象(5字节) + 55
        print(f"帧太短: len={len(packet)}")
        return []
    
    # 验证帧头帧尾
    if packet[0] != 0xAA or packet[-1] != 0x55:
        print(f"无效帧头尾: {packet[0]:02X} / {packet[-1]:02X}")
        return []
    
    objects = []
    data_bytes = packet[1:-1]  # 移除头尾
    
    # 每5字节表示一个检测对象
    object_count = len(data_bytes) // 5
    if len(data_bytes) % 5 != 0:
        print(f"数据长度错误: len={len(data_bytes)}")
        return []
    
    # 解析每个检测对象
    for i in range(object_count):
        offset = i * 5
        color_id = data_bytes[offset]
        x = data_bytes[offset+1]
        y = data_bytes[offset+2]
        w = data_bytes[offset+3]
        h = data_bytes[offset+4]
        
        # 验证颜色索引范围
        if color_id < len(COLOR_NAMES):
            color_name = COLOR_NAMES[color_id]
        else:
            color_name = f"未知({color_id})"
        
        objects.append({
            "color": color_name,
            "x": x,
            "y": y,
            "width": w,
            "height": h
        })
    
    return objects

def read_detection_data_new():
    # 检查接收缓冲区
    buf_len = uart3.any()
    if buf_len:
        raw_data = uart3.read(buf_len)
        # print(f"原始数据: len={len(raw_data)}")
        
        # 处理新数据提取完整帧
        frames = process_incoming_data(raw_data)
        
        # 处理每个完整帧
        for frame in frames:
            # print(f"完整帧: {bytes(frame).hex()}")
            detected_objects = parse_detection_packet_new(frame)
            
            if detected_objects:
                # print(f"解析到 {len(detected_objects)} 个物体:")
                for obj in detected_objects:
                    if (obj['color'] != 'purple'):
                        beep.start('short')
                    print(f"  颜色: {obj['color']}, 位置: ({obj['x']},{obj['y']})", end="")
                    print(f", 大小: {obj['width']}x{obj['height']}")
                
                # 发送ACK响应
                # uart3.write(f"ACK:{len(detected_objects)}\n".encode())





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
speed_fast_distance = Distance()
openart_distance = Distance()


class Openart_Validator:
    def __init__(self, openart_distance):
        """
        初始化ID验证器
        :param distance_obj: Distance类的实例，用于获取当前行驶距离
        """
        self.distance_obj = openart_distance        # 距离传感器对象
        self.last_valid_id = None               # 上一次有效的ID
        self.consecutive_count = 0              # 连续输入相同ID的次数
        
        # 存储每个ID的锁定信息（锁定时距离、解锁所需距离）
        # 格式: {id: {'locked': bool, 'lock_start_distance': float, 'unlock_distance': float}}
        self.lock_info = {}
        
        # 设置ID锁定规则（示例使用ID1需100距离，ID2需200距离）
        self.lock_rules = {RoadElement.l3: 600, RoadElement.r3: 600}

    def check_id(self, input_id):
        """
        检查输入的ID是否有效
        :param input_id: 输入的ID
        :return: 返回处理结果字符串（无效时返回"invalid"，有效时返回"valid"，触发锁定返回"locked"）
        """
        current_distance = self.distance_obj.data
        
        # 检查ID是否被锁定
        if self._is_id_locked(input_id, current_distance):
            return "invalid"
        
        # 检查是否与上一次有效ID相同
        if input_id == self.last_valid_id:
            self.consecutive_count += 1
        else:
            self.consecutive_count = 1
            self.last_valid_id = input_id
        
        # 检查是否达到连续三次
        if self.consecutive_count == 3:
            self._lock_id(input_id, current_distance)
            return "locked"
        
        return "valid"
    
    def _is_id_locked(self, id, current_distance):
        """检查ID是否处于锁定状态"""
        if id in self.lock_info and self.lock_info[id]['locked']:
            # 检查是否达到解锁距离
            locked_distance = current_distance - self.lock_info[id]['lock_start_distance']
            if locked_distance >= self.lock_info[id]['unlock_distance']:
                # 解锁ID
                self.lock_info[id]['locked'] = False
            else:
                return True  # ID仍处于锁定状态
        return False
    
    def _lock_id(self, id, current_distance):
        """锁定指定的ID"""
        if id not in self.lock_rules:
            return  # 无锁定规则则不处理
        
        unlock_distance = self.lock_rules[id]
        
        # 创建或更新锁定记录
        if id not in self.lock_info:
            self.lock_info[id] = {
                'locked': True,
                'lock_start_distance': current_distance,
                'unlock_distance': unlock_distance
            }
        else:
            self.lock_info[id].update({
                'locked': True,
                'lock_start_distance': current_distance
            })
        
        # 重置连续计数和上一次有效ID
        self.consecutive_count = 0
        self.last_valid_id = None

openart_l3 = Openart_Validator(openart_distance)