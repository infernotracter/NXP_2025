from basic_data import *
'''
------------------------------------
下为openart的串口配置
------------------------------------
'''
# 颜色列表 - 必须与摄像头程序完全一致
COLOR_NAMES = [
    "red", "green", "yellow", "orange", 
    "purple", "pink", "cyan", "brown"
]
# 串口配置 - 使用UART1 (LPUART2)
uart3 = UART(3, 115200)
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
                    return obj['color']
                    print(f"  颜色: {obj['color']}, 位置: ({obj['x']},{obj['y']})", end="")
                    print(f", 大小: {obj['width']}x{obj['height']}")
                
                # 发送ACK响应
                # uart3.write(f"ACK:{len(detected_objects)}\n".encode())

class Openart_Validator:
    def __init__(self, target_distance):
        self.distance = target_distance
        self.last_id = None
        self.state = 'waiting'
        self.count = 0

    def check_id(self, input_id):
        if input_id == 'green' or input_id == 'yellow' or input_id == 'red':
            beep.start('short')
            self.count += 1
            if self.count == 1:
                self.state = 'valid'
                openart_distance.data = 0
            elif self.count > 1:
                if abs(openart_distance.data) > self.distance:
                    openart_distance.data = 0
                    self.count = -1
                    self.state = 'waiting'

openart_l3 = Openart_Validator(400)