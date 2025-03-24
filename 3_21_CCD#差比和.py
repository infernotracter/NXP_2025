
# 本示例程序演示如何使用 seekfree 库的 TSL1401 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板与 TSL1401 IPS200 模块测试

# 示例程序运行效果是实时在 IPS200 屏幕上显示 CCD 的采集图像
# 当 C19 引脚电平出现变化时退出测试程序

# CCD 的曝光计算方式
# CCD 通过 TSL1401(x) 初始化构建对象时 传入的 x 代表需要进行几次触发才会更新一次数据
# Ticker 通过 start(y) 启动时 y 代表 Ticker 的周期
# 此时每 y 毫秒会触发一次 CCD 的更新
# 当触发次数大于等于 x 时 CCD 才会更新一次数据
# 因此 CCD 的曝光时间等于 y * x 本例程中就是 10ms * 10 = 100ms

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 包含 display 库
from display import *

# 从 seekfree 库包含 TSL1401
from seekfree import TSL1401

# 包含 gc 类
import gc

# 学习板上 C19 对应二号拨码开关

# 调用 machine 库的 Pin 类实例化一个引脚对象
# 配置参数为 引脚名称 引脚方向 模式配置 默认电平
# 详细内容参考 固件接口说明
switch2 = Pin('C19', Pin.IN , pull = Pin.PULL_UP_47K, value = True)

state2  = switch2.value()

# 定义片选引脚
cs = Pin('C5' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 拉高拉低一次 CS 片选确保屏幕通信时序正常
cs.high()
cs.low()
# 定义控制引脚
rst = Pin('B9' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc  = Pin('B8' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C4' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 新建 LCD 驱动实例 这里的索引范围与 SPI 示例一致 当前仅支持 IPS200
drv = LCD_Drv(SPI_INDEX=1, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
# 新建 LCD 实例
lcd = LCD(drv)
# color 接口设置屏幕显示颜色 [前景色,背景色]
lcd.color(0xFFFF, 0x0000)
# mode 接口设置屏幕显示模式 [0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转]
lcd.mode(2)
# 清屏
lcd.clear(0x0000)

# 调用 TSL1401 模块获取 CCD 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 默认参数为 1 调整这个参数相当于调整曝光时间倍数
ccd = TSL1401(10)

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

# 通过 capture 接口更新数据 但在这个例程中被 ticker 模块接管了
# ccd.capture()
# 关联采集接口 最少一个 最多八个
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
pit1.capture_list(ccd)

# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(10)

# 需要注意的是 ticker 是底层驱动的 这导致 Thonny 的 Stop 命令在这个固件版本中无法停止它
# 因此一旦运行了使用了 ticker 模块的程序 要么通过复位核心板重新连接 Thonny
# 或者像本示例一样 使用一个 IO 控制停止 Ticker 后再使用 Stop/Restart backend 按钮
# V1.1.2 以上版本则可以直接通过 Stop/Restart backend 按钮停止 Ticker


# 二值化处理
def ccd_image_value(ccd_data, value):
    ccd_value = [0] * len(ccd_data)   # 定义一个空二值化列表
    for i in range(len(ccd_data)):    # 遍历0-127点
        temp_num = ccd_data[i]        # 暂存列表
        if temp_num > value:          # 比较阈值大小
            ccd_value[i] = 1          # 大于阈值为1
        else:
            ccd_value[i] = 0          # 小于为0
    return ccd_value

# 获得动态阈值
def ccd_get_threshold(ccd_data):
    value_max = ccd_data[4]          # 从第5个元素开始考虑最大值
    value_min = ccd_data[4]          # 从第5个元素开始考虑最小值

    # 遍历5-122
    for i in range(5, 123):
        value_max = max(value_max, ccd_data[i])   # 限幅在最大传入数据和第5个元素值上
        value_min = min(value_min, ccd_data[i])   # 限幅在最小传入数据和第5个元素值上

    threshold = (value_max + value_min) / 2      # 计算阈值
    # threshold = min(max(75, threshold), 255)     # 阈值限幅在75-256之间
    return threshold

# # ccd滤波
# def ccd_filter(ccd_n):
#     if ccd_n == 1:  # 对CCD1采集的图像进行滤波
#         for i in range(1, 126):      # 防止越界
#             if image_value1[i] == 1 and image_value1[i - 1] == 0 and image_value1[i + 1] == 0:
#                 image_value1[i] = 0  # 如果当前为1，前后都为0，则将当前设置为0
#             elif image_value1[i] == 0 and image_value1[i - 1] == 1 and image_value1[i + 1] == 1:
#                 image_value1[i] = 1  # 如果当前为0，前后都为1，则将当前设置为1

#     elif ccd_n == 2:  # 对CCD2采集的图像进行滤波
#         for i in range(1, 126):      # 防止越界
#             if image_value2[i] == 1 and image_value2[i - 1] == 0 and image_value2[i + 1] == 0:
#                 image_value2[i] = 0  # 如果当前为1，前后都为0，则将当前设置为0
#             elif image_value2[i] == 0 and image_value2[i - 1] == 1 and image_value2[i + 1] == 1:
#                 image_value2[i] = 1  # 如果当前为0，前后都为1，则将当前设置为1



ccd_data2_filtered = [0] * 128
while True:
    if (ticker_flag):
        # 通过 get 接口读取数据 参数 [0,1] 对应学习板上 CCD1/2 接口
        # ccd_data1 = ccd.get(0)
        ccd_data2 = ccd.get(1)

        # 通过 wave 接口显示数据波形 (x,y,width,high,data,data_max)
        # x - 起始显示 X 坐标
        # y - 起始显示 Y 坐标
        # width - 数据显示宽度 等同于数据个数
        # high - 数据显示高度
        # data - 数据对象 这里基本仅适配 TSL1401 的 get 接口返回的数据对象
        # max - 数据最大值 TSL1401 的数据范围默认 0-255 这个参数可以不填默认 255
        # lcd.wave(0,  0, 128, 64, ccd_data1, max = 255)
        threshold = ccd_get_threshold(ccd_data2)
        ccd_data2_filtered = ccd_image_value(ccd_data2, threshold)
        # ccd_filter(2)

        lcd.wave(0,  0, 128, 64, ccd_data2_filtered, max = 255)
        lcd.wave(0, 80, 128, 64, ccd_data2, max = 255)
        
        ticker_flag = False
        runtime_count = runtime_count + 1
        print("runtime_count = {:>6d}.".format(runtime_count))
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()