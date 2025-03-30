# 基础库、NXP库、第三方库
from machine import *
from display import *
from smartcar import *
from seekfree import *
from math import *
import gc
import time
import utime
import math
from imu_handler import *
from menutext import *
# 单位换算用
ACC_SPL = 4096.0
GYRO_SPL = 16.4
# 实例化 WIRELESS_UART 模块 参数是波特率
# 无线串口模块需要自行先配对好设置好参数
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
imu = IMU963RA()
# 核心板上的LED
led1 = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
# 拨码开关2
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 拨码开关4
switch_3 = Pin('B14', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 拨码开关3
switch_4 = Pin('B15', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
# 调用 TSL1401 模块获取 CCD 实例
ccd = TSL1401(3)
# 实例化 KEY_HANDLER 模块
key = KEY_HANDLER(10)

pit_cont_pid = 0

# 定义一个回调函数
ticker_flag_gyro = False
ticker_flag_1ms = False
ticker_flag_4ms = False
ticker_flag_5ms = False
ticker_flag_8ms = False
ticker_flag_angle = False
ticker_flag_speed = False

def time_pit_pid_handler(time):
    global ticker_flag_gyro, ticker_flag_angle, ticker_flag_speed, pit_cont_pid
    if (pit_cont_pid % 1 == 0):
        ticker_flag_gyro = True
    if (pit_cont_pid % 5 == 0):
        ticker_flag_angle = True
    if (pit_cont_pid >= 10):
        ticker_flag_speed = True
        pit_cont_pid = 0
pit0 = ticker(0)
pit0.capture_list(ccd, key, encoder_l, encoder_r)
pit0.callback(time_pit_pid_handler)
pit0.start(1)

stop_time = 0


def time_pit_1ms_handler(time):
    global ticker_flag_1ms, stop_time
    ticker_flag_1ms = True
pit1 = ticker(1)
pit1.capture_list(imu)
pit1.callback(time_pit_1ms_handler)
pit1.start(1)  # 之前为3，现在改为1


def time_pit_5ms_handler(time):
    global ticker_flag_5ms
    ticker_flag_5ms = True
# # 实例化 PIT ticker 模块
pit2 = ticker(2)
pit2.capture_list(encoder_l, encoder_r)
pit2.callback(time_pit_5ms_handler)
pit2.start(5)

pit_cont_dir = 0
def time_pit_turnpid_handler(time):
    global ticker_flag_4ms, ticker_flag_8ms, pit_cont_dir
    pit_cont_dir += 1
    if (pit_cont_dir % 4 == 0):
        ticker_flag_4ms = True
    if (pit_cont_dir >= 8):
        ticker_flag_8ms = True
        pit_cont_dir = 0


pit3 = ticker(3)
pit3.capture_list()
pit3.callback(time_pit_turnpid_handler)
pit3.start(1)

# 初始化变量
ccd_data1 = [0] * 128  # ccd1原始数组
ccd_data2 = [0] * 128  # ccd2原始数组
encl_data = 0  # 左编码器数据
encr_data = 0  # 右数据编码器
aim_speed = 0  # 之后要可以使用KEY手动修改
aim_speed_l = 0  # 左轮期望速度
aim_speed_r = 0  # 右轮期望速度
out_l = 0  # 左轮输出值
out_r = 0  # 右轮输出值
MedAngle = 32.5
speed_d = 50  # 速度增量(调试用)
data_wave = [0, 0, 0, 0, 0, 0, 0, 0]

def my_limit(value, min_val, max_val):
    return max(min_val, min(value, max_val))


# 特殊输出调整函数
def gyro_adjustment(output):
    return output + 700 if output >= 0 else output - 700

# PID实例化
speed_pid = PID(kp=0.0, ki=0.0, integral_limits=(-2000, 2000))
# output_limits=(-500, 500))


angle_pid = PID(kp=0.0, kd=0.0)
# , integral_limits=(-2000, 2000))

gyro_pid = PID(kp=1750.0, kd=0.0,
               #  integral_limits=(-2000, 2000),
               # output_limits=(-500, 500),
               output_adjustment=gyro_adjustment)

dir_in = PID(kp=0.0, ki=0.0)
#  integral_limits=(-2000, 2000))

dir_out = PID(kp=0.0, kd=0.0)

# 串级PID相关变量
speed_pid_out = 0
angle_pid_out = 0
gyro_pid_out = 0
dir_in_out = 0
dir_out_out = 0

# 在主程序初始化阶段创建实例
profiler_1ms = TickerProfiler("1ms", expected_interval_ms=1)
profiler_5ms = TickerProfiler("5ms", expected_interval_ms=5)
profiler_gyro = TickerProfiler("Gyro", expected_interval_ms=1)  # 示例值
profiler_angle = TickerProfiler("Angle", expected_interval_ms=5)
profiler_speed = TickerProfiler("Speed", expected_interval_ms=10)
profiler_4ms = TickerProfiler("4ms", expected_interval_ms=4)
profiler_8ms = TickerProfiler("8ms", expected_interval_ms=8)

imu = IMUHandler()
stop_flag = 1
key_data = key.get()
while True:
    if (imu.roll >= 75) or (imu.roll <= 20):
        stop_flag = 0

    motor_l.duty(my_limit(gyro_pid_out - dir_in_out, -3000, 3000))
    motor_r.duty(my_limit(gyro_pid_out + dir_in_out, -3000, 3000))

    print(f"{motor_l.duty()}, {motor_r.duty()}, {imu.pitch}, {imu.roll}, {imu.yaw}")

    # 拨码开关关中断
    if end_switch.value() == 1:
        pit1.stop()  # pit1关闭
        pit2.stop()  # pit2关闭
        pit3.stop()  # pit3关闭
        break  # 跳出判断

    # 1ms中断标志位
    if (ticker_flag_1ms):
        profiler_1ms.update()
        imu.update()  # 更新IMU数据
        ticker_flag_1ms = False

    if (ticker_flag_5ms):
        profiler_5ms.update()
        encl_data = encoder_l.get()  # 读取左编码器的数据
        encr_data = encoder_r.get()  # 读取右编码器的数据
        # 原函数此时为圆环处理
        imu.quaternion_update()
        ticker_flag_5ms = False

    if (ticker_flag_gyro):  # kp=100.1  ki=2.0000001
        profiler_gyro.update()
        gyro_pid_out = gyro_pid.calculate(angle_pid_out, imu.data[3])
        ticker_flag_gyro = False

    if (ticker_flag_angle):
        profiler_angle.update()
        angle_pid_out = angle_pid.calculate(
            speed_pid_out + MedAngle, imu.roll)
        menu(key_data)
        key_data = key.get()
        if (key_data[0] or key_data[1] or key_data[2] or key_data[3]):
            stop_flag = 1
        ticker_flag_angle = False

    if (ticker_flag_speed):
        profiler_speed.update()
        speed_pid_out = speed_pid.calculate(
            aim_speed, (encl_data + encr_data) / 2)
        ticker_flag_speed = False

    if (ticker_flag_4ms):
        profiler_4ms.update()
        # dir_in_out = dir_in.calculate(dir_out_out, imu[4])
        ticker_flag_4ms = False

    if (ticker_flag_8ms):
        profiler_8ms.update()
        # 定期进行数据解析
        data_flag = wireless.data_analysis()
        for i in range(0, 8):
            # 判断哪个通道有数据更新
            if (data_flag[i]):
                # 数据更新到缓冲
                data_wave[i] = wireless.get_data(i)
                # 将更新的通道数据输出到 Thonny 的控制台
                print("Data[{:<6}] updata : {:<.3f}.\r\n".format(
                    i, data_wave[i]))
                gyro_pid.kp = data_wave[0]
                gyro_pid.kd = data_wave[1]
                angle_pid.kp = data_wave[2]
                angle_pid.kd = data_wave[3]
                speed_pid.kp = data_wave[4]
                speed_pid.ki = data_wave[5]
        # 将数据发送到示波器
        wireless.send_oscilloscope(
            gyro_pid.kp, gyro_pid.kd, angle_pid.kp, angle_pid.kd,
            speed_pid.kp, imu.yaw, imu.roll, motor_l.duty())

        # dir_out_out = dir_out.calculate(0, (error1 + error2) * error_k)
        ticker_flag_8ms = False


