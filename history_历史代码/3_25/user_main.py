import gc
from handware import *
from imu_processor import *
from pid_controller import *
#from ticker import *
from menu_test import *


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


print("started")
sys.imu_offset()
while True:
    motor_l.duty(pid.out_l)
    motor_r.duty(pid.out_r)

    print(f"{motor_l.duty()}, {motor_r.duty()}, {sys.current_pitch}, {sys.current_roll}, {sys.current_yaw}")

    # 拨码开关关中断
    if end_switch.value() == 1:
        break  # 跳出判断
    
    lcd.str24(60, 0, "TEST", 0x07E0)
    menu.handle_input(sys.key_data)
    menu.render()

    # 1ms中断标志位
    if (ticker_flag_1ms):
        print("1ms")
        sys.imu_filter()
        sys.current_pitch, sys.current_roll, sys.current_yaw = quaternion_update(
            sys.imu_data[0], sys.imu_data[1],
            sys.imu_data[2], sys.imu_data[3], 
            sys.imu_data[4], sys.imu_data[5])
        ticker_flag_1ms = False

    if (ticker_flag_5ms):
        sys.update_encoder()
        # 原函数此时为圆环处理
        ticker_flag_5ms = False

    # if (check_flag(2)):  # kp=100.1  ki=2.0000001
    #     pid.gyro_pid_out = gyro_pid.calculate(pid.angle_pid_out, sys.imu_data[3])

    # if (check_flag(10)):
    #     pid.angle_pid_out = angle_pid.calculate(
    #         pid.speed_pid_out + sys.MedAngle, sys.current_roll)
    #     # menu(key_data)
    #     # key_data = key.get()

    # if (check_flag(50)):
    #     pid.speed_pid_out = speed_pid.calculate(
    #         sys.aim_speed, (sys.encl_data + sys.encr_data) / 2)

    # if (ticker.ticker_flag_4ms):
    #     # sys.dir_in_out = dir_in.calculate(dir_out_out, imu[4])
    #     ticker.ticker_flag_4ms = False

    # if (ticker.ticker_flag_8ms):
    #     # sys.dir_out_out = dir_out.calculate(0, (error1 + error2) * error_k)
    #     ticker.ticker_flag_8ms = False
    
    gc.collect() #主循环结束后进行垃圾回收
