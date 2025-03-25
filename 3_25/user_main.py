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
        break  # 跳出判断

    # 1ms中断标志位
    if (check_flag(1)):
        sys.imu_filter()
        sys.current_pitch, sys.current_roll, sys.current_yaw = quaternion_update(
            sys.imu_data[0], sys.imu_data[1],
            sys.imu_data[2], sys.imu_data[3], 
            sys.imu_data[4], sys.imu_data[5])

    if (check_flag(5)):
        sys.update_encoder()
        # 原函数此时为圆环处理

    if (check_flag(2)):  # kp=100.1  ki=2.0000001
        pid.gyro_pid_out = gyro_pid.calculate(pid.angle_pid_out, sys.imu_data[3])

    if (check_flag(10)):
        pid.angle_pid_out = angle_pid.calculate(
            pid.speed_pid_out + sys.MedAngle, sys.current_roll)
        menu(key_data)
        key_data = key.get()

    if (check_flag(50)):
        pid.speed_pid_out = speed_pid.calculate(
            sys.aim_speed, (sys.encl_data + sys.encr_data) / 2)

    # if (ticker.ticker_flag_4ms):
    #     # sys.dir_in_out = dir_in.calculate(dir_out_out, imu[4])
    #     ticker.ticker_flag_4ms = False

    # if (ticker.ticker_flag_8ms):
    #     # sys.dir_out_out = dir_out.calculate(0, (error1 + error2) * error_k)
    #     ticker.ticker_flag_8ms = False
    
    gc.collect() #主循环结束后进行垃圾回收
