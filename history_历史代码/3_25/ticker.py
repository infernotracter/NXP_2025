from smartcar import ticker
from handware import *
class ticker_flags:
    def __init__(self):
        self.flag_1ms = False
        self.flag_5ms = False
        self.flag_4ms = False
        self.flag_5ms_data = False
        self.flag_8ms = False
        self.flag_25ms = False
        self.flag_125ms = False
flags=ticker_flags()
# 零号ticker记数，用于三个pid
pit_cont_gyro = 0  # 仿独立计时器用于角速度环
pit_cont_angle = 0  # 仿独立计时器用于角度环
pit_cont_speed = 0  # 仿独立计时器用于速度环

# 定义一个回调函数

def time_pit_pid_handler(time):
    global pit_cont_gyro, pit_cont_angle, pit_cont_speed
    pit_cont_gyro += 1
    pit_cont_angle += 1
    pit_cont_speed += 1
    if (pit_cont_gyro == 5):
        flags.flag_5ms = True
        pit_cont_gyro = 0  # 重置计时器
    if (pit_cont_angle == 25):
        flags.flag_25ms = True
        pit_cont_angle = 0
    if (pit_cont_speed == 125):
        flags.flag_125ms = True
        pit_cont_speed = 0


# # 实例化 PIT  模块
pit0 = ticker(0)
pit0.capture_list(ccd, imu, key, encoder_l, encoder_r)
pit0.callback(time_pit_pid_handler)
pit0.start(1)


def time_pit_1ms_handler(time):
    flags.flag_1ms = True


pit1 = ticker(1)
pit1.capture_list(imu, key)
pit1.callback(time_pit_1ms_handler)
pit1.start(1)  # 之前为3，现在改为1


def time_pit_5ms_handler(time):
    flags.flag_5ms_data = True


# # 实例化 PIT  模块
pit2 = ticker(2)
pit2.capture_list(ccd, key, encoder_l, encoder_r)
pit2.callback(time_pit_5ms_handler)
pit2.start(5)
pit_dir_in = 0
pit_dir_out = 0


def time_pit_turnpid_handler(time):
    global pit_dir_in, pit_dir_out
    pit_dir_in += 1
    pit_dir_out += 1
    if (pit_dir_in == 4):
        flags.flag_4ms = True
        pit_dir_in = 0
    if (pit_dir_out == 8):
        flags.flag_8ms = True
        pit_dir_out = 0


pit3 = ticker(3)
pit3.capture_list(ccd, imu)
pit3.callback(time_pit_turnpid_handler)
pit3.start(1)

<<<<<<< HEAD

=======
>>>>>>> 4cff6c63778886ea78cd6ec1d5ec14f0f45670dd
def check_flag(flag):
    if flag==1:
        res=flags.flag_1ms
        flags.flag_1ms=False
        return res
    if flag==2:
        res = flags.flag_5ms
        flags.flag_5ms = False
        return res
    if flag==4:
        res = flags.flag_4ms
        flags.flag_4ms = False
        return res
    if flag==5:
        res = flags.flag_5ms_data
        flags.flag_5ms_data = False
        return res
    if flag==8:
        res = flags.flag_8ms
        flags.flag_8ms = False
        return res
    if flag==10:
        res = flags.flag_25ms
        flags.flag_25ms = False
        return res
    if flag==50:
        res = flags.flag_125ms
        flags.flag_125ms = False
        return res

