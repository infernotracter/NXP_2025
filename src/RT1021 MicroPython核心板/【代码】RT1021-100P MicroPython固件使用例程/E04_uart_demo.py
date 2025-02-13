
# 本示例程序演示如何使用 machine 库的 UART 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的无线串口模块接口

# 示例程序运行效果为每 500ms(0.5s) 改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 并通过无线串口模块接口接收并回传数据

# 从 machine 库包含所有内容
from machine import *

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

# 构造接口 标准 MicroPython 的 machine.UART 模块 参数说明
# id = 0 TX = B6  RX = B7
# id = 1 TX = C22 RX = C23
# id = 2 TX = C6  RX = C7
# id = 3 TX = B10 RX = B11
# id = 4 TX = D22 RX = D23
# id = 5 TX = B26 RX = B27
# id = 6 TX = D17 RX = D18
uart2 = UART(2)

# 串口参数设置 参数说明
#   baudrate串口速率    | 默认 9600
#   bits    数据位数    | 默认 8 bits 数据位
#   parity  校验位数    | 默认 无校验
#   stop    停止位数    | 默认 1 bit 停止位
uart2.init(460800)

uart2.write("Test.\r\n")
buf_len = 0

while True:
    # 每 500ms 读取一次 将数据再原样发回
    time.sleep_ms(500)
    buf_len = uart2.any()
    buf = uart2.read(buf_len)
    print("buf_len ={:>6d}".format(buf_len))
    uart2.write(buf)
    
    # 翻转 C4 LED 电平
    led1.toggle()
    # 回收内存
    gc.collect()
