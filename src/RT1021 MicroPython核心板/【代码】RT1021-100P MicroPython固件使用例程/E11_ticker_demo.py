
# 本示例程序演示如何使用 smartcar 库的 ticker 类接口
# 使用 RT1021-MicroPython 核心板即可测试

# 示例程序运行效果为每 100ms(0.1s) 通过 Type-C 的 CDC 虚拟串口输出一次信息
# 可以通过 C19 的电平状态来控制是否退出测试程序

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 包含 gc 类
import gc

# 开发板上的 C19 是拨码开关
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
end_state = end_switch.value()

ticker_flag = False
ticker_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    ticker_flag = True  # 否则它会新建一个局部变量

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(100)

# 需要注意的是 ticker 是底层驱动的 这导致 Thonny 的 Stop 命令在这个固件版本中无法停止它
# 因此一旦运行了使用了 ticker 模块的程序 要么通过复位核心板重新连接 Thonny
# 或者像本示例一样 使用一个 IO 控制停止 Ticker 后再使用 Stop/Restart backend 按钮
# V1.1.2 以上版本则可以直接通过 Stop/Restart backend 按钮停止 Ticker

while True:
    if (ticker_flag):
        ticker_flag = False
        ticker_count = ticker_count + 1
        print("Ticker trigger {:>6d}.".format(ticker_count))
    if end_switch.value() != end_state:
        pit1.stop()
        print("Ticker stop.")
        break
    gc.collect()
