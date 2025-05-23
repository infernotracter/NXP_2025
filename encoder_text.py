from machine import *
from smartcar import ticker, encoder
import gc
import math

# 硬件初始化
led = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
switch2 = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
state2 = switch2.value()

# 编码器初始化
encoder_l = encoder("D0", "D1", True)
encoder_r = encoder("D2", "D3")

# ========== 滤波算法实现 ==========
class EncoderFilter:
    def __init__(self, filter_type='lowpass', window_size=5, alpha=0.3):
        """
        编码器滤波器初始化
        :param filter_type: 滤波类型 ('moving_average' 或 'lowpass')
        :param window_size: 移动平均窗口大小(仅对移动平均有效)
        :param alpha: 低通滤波系数(0-1, 越小滤波越强)
        """
        self.filter_type = filter_type
        self.alpha = alpha
        self.window_size = window_size
        self.buffer = []
        self.last_value = 0
        
    def update(self, new_value):
        """更新滤波值"""
        if self.filter_type == 'moving_average':
            return self._moving_average(new_value)
        else:
            return self._lowpass_filter(new_value)
    
    def _moving_average(self, new_value):
        """滑动平均滤波"""
        self.buffer.append(new_value)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        return sum(self.buffer) / len(self.buffer)
    
    def _lowpass_filter(self, new_value):
        """一阶低通滤波"""
        filtered = self.alpha * new_value + (1 - self.alpha) * self.last_value
        self.last_value = filtered
        return filtered

# 创建滤波器实例 (可修改参数)
filter_l = EncoderFilter(filter_type='lowpass', alpha=0.3)  # 右轮使用低通滤波
filter_r = EncoderFilter(filter_type='moving_average', window_size=5)  # 左轮使用移动平均

# ========== 主程序 ==========
ticker_flag = False
ticker_count = 0

def time_pit_handler(time):
    global ticker_flag, ticker_count
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else 1

pit1 = ticker(1)
pit1.capture_list(encoder_l, encoder_r)
pit1.callback(time_pit_handler)
pit1.start(10)  # 10ms采样周期

# 原始值和滤波值记录
raw_data = [0, 0]
filtered_data = [0, 0]

while True:
    if ticker_flag and ticker_count % 20 == 0:  # 每200ms处理一次
        led.toggle()
        
        # 获取原始数据
        raw_data[0] = encoder_l.get()
        raw_data[1] = encoder_r.get()
        
        # 应用滤波
        filtered_data[0] = filter_l.update(raw_data[0])
        filtered_data[1] = filter_r.update(raw_data[1])
        
        # 打印结果 (原始值 vs 滤波值)
        print("L: raw={:>6d} filtered={:>6.1f} | R: raw={:>6d} filtered={:>6.1f}".format(
            raw_data[0], filtered_data[0], 
            raw_data[1], filtered_data[1]))
        
        ticker_flag = False
    
    if switch2.value() != state2:
        pit1.stop()
        print("Test program stopped by user.")
        break
    
    gc.collect()