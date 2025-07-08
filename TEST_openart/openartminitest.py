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

while True:
    time.sleep_ms(100)  # 缩短等待时间以更快响应
    led.toggle()  # 闪烁LED表示程序运行中

    # 检查接收缓冲区
    buf_len = uart3.any()
    if buf_len:
        buf = uart3.read(buf_len)
        print(f"原始数据 (len={buf_len}):", buf)
        
        # 尝试解析数据包
        detected_objects = parse_detection_packet(buf)
        
        if detected_objects:
            print(f"解析到 {len(detected_objects)} 个物体:")
            for obj in detected_objects:
                print(f"  颜色: {obj['color']}, 位置: ({obj['x']},{obj['y']})", end="")
                print(f", 大小: {obj['width']}x{obj['height']}")
                
            # 可改为只回送解析结果
            response = f"收到{len(detected_objects)}个对象"
            uart3.write(response.encode())
    
    # 退出检查
    if switch2.value() != state2:
        print("测试程序停止.")
        uart3.write("程序停止\r\n")
        break

    gc.collect()  # 内存回收
