from smartcar import ticker
from handware import *
# 零号ticker记数，用于三个pid
pit_cont_gyro = 0  # 仿独立计时器用于角速度环
pit_cont_angle = 0  # 仿独立计时器用于角度环
pit_cont_speed = 0  # 仿独立计时器用于速度环

# 定义一个回调函数
ticker_flag_2ms = False
ticker_flag_1ms = False
ticker_flag_4ms = False
ticker_flag_5ms = False
ticker_flag_8ms = False
ticker_flag_10ms = False
ticker_flag_50ms = False


def time_pit_pid_handler(time):
    global ticker_flag_2ms, ticker_flag_10ms, ticker_flag_50ms, pit_cont_gyro, pit_cont_angle, pit_cont_speed
    pit_cont_gyro += 1
    pit_cont_angle += 1
    pit_cont_speed += 1
    if (pit_cont_gyro == 5):
        ticker_flag_2ms = True
        pit_cont_gyro = 0  # 重置计时器
    if (pit_cont_angle == 25):
        ticker_flag_10ms = True
        pit_cont_angle = 0
    if (pit_cont_speed == 125):
        ticker_flag_50ms = True
        pit_cont_speed = 0


# # 实例化 PIT ticker 模块
pit0 = ticker(0)
pit0.capture_list(ccd, imu, key, encoder_l, encoder_r)
pit0.callback(time_pit_pid_handler)
pit0.start(1)


def time_pit_1ms_handler(time):
    global ticker_flag_1ms
    ticker_flag_1ms = True


pit1 = ticker(1)
pit1.capture_list(imu, key)
pit1.callback(time_pit_1ms_handler)
pit1.start(1)  # 之前为3，现在改为1


def time_pit_5ms_handler(time):
    global ticker_flag_5ms
    ticker_flag_5ms = True


# # 实例化 PIT ticker 模块
pit2 = ticker(2)
pit2.capture_list(ccd, key, encoder_l, encoder_r)
pit2.callback(time_pit_5ms_handler)
pit2.start(5)
pit_dir_in = 0
pit_dir_out = 0


def time_pit_turnpid_handler(time):
    global ticker_flag_4ms, ticker_flag_8ms, pit_dir_in, pit_dir_out
    pit_dir_in += 1
    pit_dir_out += 1
    if (pit_dir_in == 4):
        ticker_flag_4ms = True
        pit_dir_in = 0
    if (pit_dir_out == 8):
        ticker_flag_8ms = True
        pit_dir_out = 0


pit3 = ticker(3)
pit3.capture_list(ccd, imu)
pit3.callback(time_pit_turnpid_handler)
pit3.start(1)