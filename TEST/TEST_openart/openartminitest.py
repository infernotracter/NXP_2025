# UART测试程序 - 带协议解析功能
from machine import *
import gc
import time

# 核心板上 C4 是 LED
led = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
# 学习板上 C19 对应二号拨码开关
switch2 = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
state2 = switch2.value()

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

uart3.write("检测程序已启动.\r\n")
print("等待数据...")
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
                    if (obj['color'] == 'pink' or obj['color'] == 'yellow'
                        or obj['color'] == 'brown' or obj['color'] == 'purple'
                        ):
                        pass
                    print(f"  颜色: {obj['color']}, 位置: ({obj['x']},{obj['y']})", end="")
                    print(f", 大小: {obj['width']}x{obj['height']}")
                
while True:
    time.sleep_ms(100)  # 缩短等待时间以更快响应
    led.toggle()  # 闪烁LED表示程序运行中

    read_detection_data_new()
    if switch2.value() != state2:
        print("测试程序停止.")
        uart3.write("程序停止\r\n")
        break

    gc.collect()  # 内存回收
