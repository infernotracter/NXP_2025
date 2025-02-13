
# 本示例程序演示如何使用 machine 库的 ADC 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的四路电磁运放接口

# 示例程序运行效果为每 500ms(0.5s) 改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 并通过 RT1021-MicroPython 核心板的 Type-C 的 CDC 虚拟串口输出一次转换数据结果

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc
import time

# 核心板上 C4 是 LED
# 调用 machine 库的 Pin 类实例化一个引脚对象
# 配置参数为 引脚名称 引脚方向 模式配置 默认电平
# 详细内容参考 RT1021-MicroPython固件接口说明.pdf
led1 = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)

# 运放对应的四个引脚都支持 ADC1 当引脚支持两个模块通道时优先分配到 ADC1 功能
# 调用 machine 库的 Pin 类实例化一个引脚对象
# 配置参数为 引脚名称 需要注意的是它优先匹配 RT1021 的 ADC1 通道
# 详细内容参考 RT1021-MicroPython固件接口说明.pdf
adc_in1 = ADC('B14')
adc_in2 = ADC('B15')
adc_in3 = ADC('B26')
adc_in4 = ADC('B27')

while True:
    # 延时 500ms
    time.sleep_ms(500)
    # 翻转 C4 LED 电平
    led1.toggle()
    
    # 读取通过 read_u16 接口读取 无参数 数据返回范围是 0-65535
    print("adc={:>6d},{:>6d},{:>6d},{:>6d}.\r\n".format(
        adc_in1.read_u16(),
        adc_in2.read_u16(),
        adc_in3.read_u16(),
        adc_in4.read_u16()))
    
    # 回收内存
    gc.collect()
