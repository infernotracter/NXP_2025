import gc
from handware import *
from imu_processor import *
from pid_controller import *
from ticker import *
from menu import *

sys.imu_offset()
while True:
    motor_l.duty(pid.out_l)
    motor_r.duty(pid.out_r)

    print(f"{motor_l.duty()}, {motor_r.duty()}, {sys.current_pitch}, {sys.current_roll}, {sys.current_yaw}")

    # 拨码开关关中断
    if end_switch.value() == 1:
        pit1.stop()  # pit1关闭
        pit2.stop()  # pit2关闭
        pit3.stop()  # pit3关闭
        break  # 跳出判断

    # 1ms中断标志位
    if (ticker_flag_1ms):
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

    if (ticker_flag_2ms):  # kp=100.1  ki=2.0000001
        pid.gyro_pid_out = gyro_pid.calculate(pid.angle_pid_out, sys.imu_data[3])
        ticker_flag_2ms = False

    if (ticker_flag_10ms):
        pid.angle_pid_out = angle_pid.calculate(
            pid.speed_pid_out + sys.MedAngle, sys.current_roll)
        menu(key_data)
        key_data = key.get()
        ticker_flag_10ms = False

    if (ticker_flag_50ms):
        pid.speed_pid_out = speed_pid.calculate(
            sys.aim_speed, (sys.encl_data + sys.encr_data) / 2)
        ticker_flag_50ms = False

    if (ticker_flag_4ms):
        # sys.dir_in_out = dir_in.calculate(dir_out_out, imu[4])
        ticker_flag_4ms = False

    if (ticker_flag_8ms):
        # sys.dir_out_out = dir_out.calculate(0, (error1 + error2) * error_k)
        ticker_flag_8ms = False
    
    gc.collect() #主循环结束后进行垃圾回收
