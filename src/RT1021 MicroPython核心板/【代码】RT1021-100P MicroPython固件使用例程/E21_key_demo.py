
# 本示例程序演示如何使用 seekfree 库的 KEY_HANDLER 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的按键测试

# 示例程序运行效果为每 1000ms(1s) C4 LED 改变亮灭状态
# 当按键短按或者长按时通过 Type-C 的 CDC 虚拟串口输出信息
# 可以通过 C19 的电平状态来控制是否退出测试程序

# KEY_HANDLER 的扫描周期计算方式
# KEY_HANDLER 通过 KEY_HANDLER(x) 初始化构建对象时 传入的 x 代表需要进行几次触发才会更新一次数据
# Ticker 通过 start(y) 启动时 y 代表 Ticker 的周期
# 此时每 y 毫秒会触发一次 KEY_HANDLER 的更新
# 当触发次数大于等于 x 时 KEY_HANDLER 才会更新一次数据
# 因此 KEY_HANDLER 的扫描周期时间等于 y * x 本例程中就是 10ms * 10 = 100ms

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 从 seekfree 库包含 KEY_HANDLER
from seekfree import KEY_HANDLER

# 包含 gc 类
import gc

# 核心板上 C4 是 LED
led1 = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
# 开发板上的 C19 是拨码开关
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
end_state = end_switch.value()

# 实例化 KEY_HANDLER 模块 参数是按键扫描周期
# 扫描周期根据实际 ticker 周期或者延时周期来确定
# 请务必确保扫描周期是正确的 否则按键触发可能会有问题
key = KEY_HANDLER(10)

ticker_flag = False
ticker_count = 0
runtime_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)
# 关联采集接口 最少一个 最多八个
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
pit1.capture_list(key)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(10)

# 需要注意的是 ticker 是底层驱动的 这导致 Thonny 的 Stop 命令在这个固件版本中无法停止它
# 因此一旦运行了使用了 ticker 模块的程序 要么通过复位核心板重新连接 Thonny
# 或者像本示例一样 使用一个 IO 控制停止 Ticker 后再使用 Stop/Restart backend 按钮
# V1.1.2 以上版本则可以直接通过 Stop/Restart backend 按钮停止 Ticker

while True:
    if (ticker_flag):
        # 通过 capture 接口更新数据 但在这个例程中被 ticker 模块接管了
        # key.capture()
        # 通过 get 接口读取数据
        key_data = key.get()
        # 按键数据为三个状态 0-无动作 1-短按 2-长按
        if key_data[0]:
            print("key1 = {:>6d}.".format(key_data[0]))
            key.clear(1)
        if key_data[1]:
            print("key2 = {:>6d}.".format(key_data[1]))
            key.clear(2)
        if key_data[2]:
            print("key3 = {:>6d}.".format(key_data[2]))
            key.clear(3)
        if key_data[3]:
            print("key4 = {:>6d}.".format(key_data[3]))
            key.clear(4)
        if (ticker_count % 100 == 0):
            led1.toggle()
        ticker_flag = False
    if end_switch.value() != end_state:
        pit1.stop()
        print("Ticker stop.")
        break
    gc.collect()
