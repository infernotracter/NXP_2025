# 基础库、NXP库、第三方库
import gc
import utime
from basic_data import *
from ccd_handler import *
#from menutext import *
#from tof_hander import *
from uart import *



def create_roll_checker():
    history = []
    def check(current_roll):
        # 将新数据添加到历史记录中
        history.append(current_roll)
        # 保持最多保留最近20个数据点
        if len(history) > 10:
            history[:] = history[-10:]
        # 如果数据不足20个，返回False
        if len(history) < 10:
            return False
        # 统计不满足条件的数据个数
        count = sum(1 for num in history if not (-60.0 < num < 0.0))
        return count >= 8
    return check
checker = create_roll_checker()


print("种族骑士王小桃来啦UwU")
# 单位换算用
ACC_SPL = 4096.0
GYRO_SPL = 16.4

pit_cont_pid = 0

# 定义一个回调函数
ticker_flag_pid = False
ticker_flag_4ms = False
ticker_flag_8ms = False
ticker_flag_menu = False


def time_pit_pid_handler(time):
    global ticker_flag_menu, pit_cont_pid
    pit_cont_pid += 5
    if (pit_cont_pid % 10 == 0):
        pit_cont_pid = 0
        ticker_flag_menu=True


# 实例化 PIT ticker 模块
pit0 = ticker(0)
pit0.capture_list(ccd, key)
pit0.callback(time_pit_pid_handler)
pit0.start(5)


def time_pit_imu_handler(time):
    global ticker_flag_pid
    ticker_flag_pid = True


pit1 = ticker(1)
pit1.capture_list(imu, encoder_l, encoder_r)
pit1.callback(time_pit_imu_handler)
pit1.start(5)

pit_cont_dir = 0



def time_pit_turnpid_handler(time):
    global ticker_flag_4ms, ticker_flag_8ms, pit_cont_dir
    pit_cont_dir += 5
    if (pit_cont_dir % 20 == 0):
        ticker_flag_4ms = True
    if (pit_cont_dir >= 40):
        ticker_flag_8ms = True
        pit_cont_dir = 0


pit3 = ticker(3)
pit3.capture_list()
pit3.callback(time_pit_turnpid_handler)
pit3.start(5)

class TickerProfiler:
    def __init__(self, name, expected_interval_ms):
        self.name = name  # Ticker名称（如 "1ms"）
        self.expected_us = expected_interval_ms * 1000  # 预期间隔（微秒）
        self.last_ticks = 0  # 上一次触发时间戳
        self.first_trigger = True  # 首次触发标志

    def update(self):
        """更新并打印时间间隔（需在每次ticker触发时调用）"""
        current_ticks = utime.ticks_us()

        if not self.first_trigger:
            # 计算实际间隔（自动处理计数器溢出）
            actual_interval_us = utime.ticks_diff(current_ticks, self.last_ticks)

            # 打印带颜色标记的调试信息（可选）
            error = abs(actual_interval_us - self.expected_us)
            status = "OK" if error < self.expected_us * 0.1 else "WARN"
            color_code = "\033[32m" if status == "OK" else "\033[31m"
            print(f"{color_code}[{self.name} Ticker] 预期: {self.expected_us}us, 实际: {actual_interval_us}us\033[0m")

        # 更新状态
        self.last_ticks = current_ticks
        self.first_trigger = False
def create_roll_checker():
    history = []
    def check(current_roll):
        # 将新数据添加到历史记录中
        history.append(current_roll)
        # 保持最多保留最近20个数据点
        if len(history) > 10:
            history[:] = history[-10:]
        # 如果数据不足20个，返回False
        if len(history) < 10:
            return False
        # 统计不满足条件的数据个数
        count = sum(1 for num in history if not (-3600.0 < num < 100.0))
        return count >= 8
    return check
checker = create_roll_checker()

# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

movementtype.aim_speed=0

# 在主程序初始化阶段创建实例
profiler_1ms = TickerProfiler("5ms", expected_interval_ms=5)
profiler_5ms = TickerProfiler("5ms", expected_interval_ms=5)
profiler_4ms = TickerProfiler("20ms", expected_interval_ms=20)
profiler_8ms = TickerProfiler("40ms", expected_interval_ms=40)

last_imu_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0]
data_wave = [0, 0, 0, 0, 0, 0, 0, 0]

key_data = key.get()
imu_data = imu.get()
imu_data_filtered = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 0.0]
speed_err_k = 1

def clearall():
    key.clear(1)
    key.clear(2)
    key.clear(3)
    key.clear(4)
stop_flag = 1
gyro_bias_x = -726
gyro_bias_y = 16.15
gyro_bias_z = -140.56
# def get_offset():
#     global gyro_bias_y, gyro_bias_z , gyro_bias_x
#     for _ in range(100):
#         imu_data = imu.read()
#         gyro_bias_y += imu_data[3]
#         gyro_bias_x += imu_data[4]
#         gyro_bias_z += imu_data[5]
#     gyro_bias_y /= 100
#     gyro_bias_z /= 100
# get_offset()

def update_filter(acc_x, gyro_y):
    global current_angle
    alpha = 0.95
    pitch_gyro = current_angle + gyro_y * 0.005
    current_angle = alpha * acc_x + (1 - alpha) * pitch_gyro
    return current_angle


# ----------------- PID算法 -----------------
#位置式
def pid_controller(now, target, kp, ki, kd, sum_err, last_err):
    error = target - now
    sum_err += error
    derivative = error - last_err
    output = kp * error + ki * sum_err + kd * derivative
    return output, sum_err, error

#增量式
# def pid_increment(now, target, kp, ki, kd, last_err, prev_err):
#     error = target - now
#     increment = kp*(error - last_err) + ki*error + kd*(error - 2*last_err + prev_err)
#     last_err = error
#     prev_err = last_err
#     return increment, last_err, prev_err
def pid_increment(now, target, kp, ki, kd, last_err, prev_err):
    error = target - now
    increment = kp*(error - last_err) + ki*error + kd*(error - 2*last_err + prev_err)
    
    new_prev_err = last_err  # 保存旧的last_err
    new_last_err = error     # 更新为当前error
    
    return increment, new_last_err, new_prev_err
# ----------------- 全局变量 -----------------

pitch_vel = 0
vel_disturbance = 0
last_turn_out = 0
real_speed = 0
acc_x = 0
gyro_y = 0
pwm_l = 0
pwm_r = 0
acc_z = 0
last_error2 = 0


balance_angle = -2849.446  #-2794.03               #-2711.82                   #-2724.24
vel_kp = 1.2    #2.01                        #2.271                         #3.71
vel_ki = 3.18        #3.154                     #2.674                         #2.84
vel_kd = 0
angle_kp = 0.036      # 0.059                   #0.062                        #0.103
angle_ki = 0
angle_kd = 0.036      #0.047
speed_kp = 6.1                  #-2.03
speed_ki = 0
speed_kd = 0

now_speed = 0
counter_speed = 0
counter_angle = 0
speed_sum_error = 0
speed_last_error = 0
angle_sum_error = 0
angle_last_error = 0
vel_last_error = 0
vel_prev_error = 0
turn_sum_error = 0
turn_last_error = 0
turn_output = 0
pwm = 0
current_angle = 0

angle_disturbance = 0
pwm_l_value = 0
pwm_r_value = 0
real_speed = 0
# ----------------- 控制回调函数 -----------------
def vel_loop_callback(pit1):
    global pwm_l_value, pwm_r_value
    global pwm, current_angle, turn_output
    global vel_last_error, vel_prev_error
    global turn_sum_error, turn_last_error
    global vel_kp, vel_ki, vel_kd
    global angle_disturbance
    global speed_kp, speed_ki, speed_kd
    global speed_sum_error, speed_last_error
    global balance_angle, angle_kp, angle_ki, angle_kd
    global angle_sum_error, angle_last_error
    global yaw_vel, counter_speed, counter_angle
    global imu, gyro_bias_y, gyro_bias_z
    global motor_l, motor_r,encl_data,encr_data
    global imu_data, now_speed 
    global vel_disturbance  # Add this line

    imu_data = imu.get()
    acc_x = imu_data[1]
    acc_z = imu_data[2]
    gyro_y = imu_data[3]
    tempgyro_z = imu_data[5]

    current_angle = update_filter(acc_x, gyro_y)
    yaw_vel = (tempgyro_z - gyro_bias_z) * 0.07
    pitch_vel = (gyro_y - gyro_bias_y) * 0.07

    counter_speed += 1
    if counter_speed >= 5:
        counter_speed = 0
        left = encoder_l.get()
        right =encoder_r.get()
        encl_data=left
        encr_data=right

#         max_speed = 2500
#         left = max(min(left, max_speed), -max_speed)
#         right = max(min(right, max_speed), -max_speed)
        
        #now_speed = now_speed*0.1 + ((left + right) / 2)*0.9
        now_speed=(left+right)/2.0
        angle_disturbance, speed_sum_error, speed_last_error = pid_controller(
            now_speed, speed_controller.target_speed, 
            speed_kp, speed_ki, speed_kd,
            speed_sum_error, speed_last_error
        )
        angle_disturbance = max(min(angle_disturbance, 500), -500)

    # Ensure vel_disturbance is initialized before use
    if 'vel_disturbance' not in globals():
        vel_disturbance = 0

    target_angle = balance_angle - angle_disturbance

    # 外环控制（每5ms执行）
    counter_angle += 1
    if counter_angle >= 2:
        counter_angle = 0
        vel_disturbance, angle_sum_error, angle_last_error = pid_controller(
            current_angle, target_angle,
            angle_kp, angle_ki, angle_kd,
            angle_sum_error, angle_last_error
        )
        #vel_disturbance = max(min(vel_disturbance, 60), -60)

    target_vel = vel_disturbance
    # 内环控制
    delta_pwm, vel_last_error, vel_prev_error = pid_increment(
        pitch_vel, target_vel,
        vel_kp, vel_ki, vel_kd, 
        vel_last_error, vel_prev_error
    )

    pwm += delta_pwm
    pwm = max(min(pwm, 6000), -6000)
    
    turn_output = 0
    pwm_output = pwm  # 极性修改

    pwm_l_value = pwm_output + turn_output
    pwm_r_value = pwm_output - turn_output
    
    
def my_limit(value, min_val, max_val):
    return max(min_val, min(value, max_val))   
# 新增转向控制相关变量
target_turn_angle = 0.0       # 目标转向角度（由遥控器设置）
current_turn_angle = 0.0      # 当前转向角度（通过陀螺仪积分）
target_yaw_vel = 0.0          # 外环输出的目标角速度
turn_out_sum_error = 0.0      # 外j环积分累积
turn_in_sum_error = 0.0       # 内环积分累积
counter_turn_out = 0
counter_turn_in = 0
turn_out_last_error = 0
turn_in_last_error = 0
turn_out_kp = -71.2
turn_out_ki = 0
turn_out_kd = -13.89
turn_in_kp = -2.3
turn_in_ki = -0.77
turn_in_kd = 0
turn_in_disturbance = 0.0
error = 0
# ----------------- 转向控制回调函数 -----------------
def turn_loop_callback(pit1):
    global current_turn_angle, target_yaw_vel, turn_output
    global turn_out_sum_error, turn_out_last_error, turn_in_sum_error, turn_in_last_error
    global counter_turn_out, yaw_vel
    global turn_out_kp, turn_out_ki, turn_out_kd
    global turn_in_kp, turn_in_ki, turn_in_kd
    global turn_in_disturbance
    global counter_turn_in
    
    # turn外环
    counter_turn_out += 5
    if counter_turn_out >= 40:
        counter_turn_out = 0
        turn_in_disturbance, turn_out_sum_error, turn_out_last_error = pid_controller(
            error, 0,
            turn_out_kp, turn_out_ki, turn_out_kd,
            turn_out_sum_error, turn_out_last_error
        )
    #turn_in_disturbance = max(min(turn_in_disturbance, 3999), -3999)

    
    # turn内环
    counter_turn_in += 5
    if counter_turn_in >= 20:
        counter_turn_in = 0
        imu_data = imu.get()
        turn_output, turn_in_sum_error, turn_in_last_error = pid_controller(
            turn_in_disturbance, imu_data[4] - gyro_bias_x,
            turn_in_kp, turn_in_ki, turn_in_kd,
            turn_in_sum_error, turn_in_last_error
        )
    #turn_output = max(min(turn_output, 3999), -3999)

    return turn_output

def death_pwm(value):
    if value > 0:
        return value + 650
    else:
        return value - 650
    
# 菜单相关变量
# point = 30
# main_menu_flag = 1
# car_go_flag = 0
# element_flag = 0
# key_cnt=0
# tmpgyro=0
# tmpdis=0
# def point_move(hight, low, key_data):
#     global point
#     if key_data[1]:
#         lcd.clear(0x0000)
#         point += 16
#         key.clear(2)
#         if point == hight + 16:
#             point = low
#     if key_data[0]:
#         lcd.clear(0x0000)
#         point -= 16
#         key.clear(1)
#         if point == low - 16:
#             point = hight
#     gc.collect()
# def menu(key_data):
#     global main_menu_flag, point, car_go_flag,start_flag,key_cnt
#     global turn_in_kp, turn_in_ki, turn_in_kd, turn_out_kp, turn_out_ki, turn_out_kd
#     lcd.str24(60, 0, "car_go_mode", 0x07E0)
#     lcd.str16(16, 30, "start_flag={} ".format(),0xFFFF)
#     lcd.str16(16, 46, "car_mode={}".format(key_cnt), 0x07E0)   #寻圆环/不寻圆环
#     lcd.str16(16, 62, "return ",0xFFFF)
#     lcd.str16(16, 126, "aim_speed={}".format(movementtype.speed),0xFFFF)
#     lcd.str16(16, 78, "real_speed={}".format(movementtype.speed),0xFFFF)
#     lcd.str12(0, point, ">", 0xF800)
#     point_move(62, 30, key_data)
#     
#     if point == 62 and key_data[2]:
#         lcd.clear(0x0000)
#         main_menu_flag = 1
#         car_go_flag = 0
#         key.clear(3)
#         point = 30
#     if key_data[2] and point == 30:
#         lcd.clear(0x0000)
#         start_flag = 1
#         key.clear(3)
#     if key_data[2] and point == 46:
#         key_cnt+=1
#         if key_cnt == 0:
#             pass    # default 啥也不改，直接跑
#         elif key_cnt == 1:
#             speed_controller.target_speed = 0
#             turn_in_kp = 0
#             turn_in_ki = 0
#             turn_in_kd = 0
#             turn_out_kp = 0
#             turn_out_ki = 0
#             turn_out_kd = 0
#         elif key_cnt >= 2:
#             key_cnt=0
#     
    
movementtype.mode=MOVEMENTTYPE.Mode_2
alldistance.start()
print("""   ____   _           _   _           /\/|
  / ___| (_)   __ _  | | | |   ___   |/\/ 
 | |     | |  / _` | | | | |  / _ \       
 | |___  | | | (_| | | | | | | (_) |      
  \____| |_|  \__,_| |_| |_|  \___/       """)
elementdetector.state = RoadElement.normal
elementdetector_flag = False
while True:
#     if elementdetector.state==RoadElement.stop:
#         stop_flag=0
    error=ccd_controller.get_error() + 10
    if elementdetector_flag:
        elementdetector.update()
        if elementdetector.state == RoadElement.normal:
            elementdetector_flag = False
    if end_switch.value() == 1:
        break  # 跳出判断
        
    if (ticker_flag_pid):
        # profiler_gyro.update()
        imu_data = imu.get()
        element_gyro.update(imu_data[5],0.01)
        element_distance.update(encl_data+encr_data,0.01)
        openart_distance.update(encl_data+encr_data,0.01)
        #speed_slow_distance.update(encl_data+encr_data, 0.01)
        #debug += (encoder_l.get() - encoder_r.get()) * 0.01
        vel_loop_callback(pit1)
        turn_loop_callback(pit1)
        motor_l.duty(my_limit(death_pwm(pwm_l_value - turn_output),-6000,6000))
        motor_r.duty(my_limit(death_pwm(pwm_r_value + turn_output),-6000,6000))
        ticker_flag_pid = False
        
    if (ticker_flag_menu):
        if read_detection_data_new() == 'green' or read_detection_data_new() == 'yellow':
            beep.start('short')
            elementdetector_flag = True
            openart_distance.data = 0
            if elementdetector.state == RoadElement.normal:
                elementdetector.state = RoadElement.l0
        if abs(openart_distance.data) > 800:
            elementdetector.state = 0
            openart_distance.data = 0
            ccd_controller.follow = 0
            ccd_controller.far = True
            ccd_controller.fix_error_value = 0
            elementdetector_flag = False
        key_data = key.get()
        speed_controller.start_update(key_data[0], elementdetector_flag)
        #menu(key_data)
        # # 弯道减速
#         if abs(ccd_far.mid - ccd_near.mid) > 15:
#             speed_controller.target_speed = int(speed_controller.tmp_speed * 0.85)
        # if abs(alldistance.data)<2000:
        #     speed_controller.target_speed = -24
        # elif (alldistance.data)>8000:
        #     speed_controller.targejt_speed = -24
        # else:
        #     speed_controller.target_speed = speed_controller.tmp_speed
        ticker_flag_menu=False

    if (ticker_flag_8ms):
        # profiler_8ms.update()
        beep.update()
        data_flag = wireless.data_analysis()
        for i in range(0, 8):
            # 判断哪个通道有数据更新
            if (data_flag[i]):
                # 数据更新到缓冲
                data_wave[i] = wireless.get_data(i)
                # 将更新的通道数据输出到 Thonny 的控制台
                print("Data[{:<6}] updata : {:<.3f}.\r\n".format(i, data_wave[i]))
                # 根据通道号单独更新对应参数

#                 if i == 0:
#                     vel_kp = data_wave[i]
#                 elif i == 1:
#                     vel_ki = data_wave[i]
#                 elif i == 2:
#                     angle_kp = data_wave[i]
#                 elif i == 3:
#                     angle_kd = data_wave[i]
#                 elif i == 4:
#                     speed_kp = data_wave[i]
#                 elif i == 7:
#                     balance_angle = data_wave[i]
#                 if i == 0:
#                     elementdetector.GYRO_Z_ring3_data = data_wave[i]
#                 elif i == 1:
#                     elementdetector.DISTANCE_ring3_data = data_wave[i]

#                 elif i == 3:
#                     ccd_controller.value = data_wave[i]
#                 elif i == 4:
#                     turn_out_ki = data_wave[i]
#                 elif i == 5:
#                     turn_out_kd = data_wave[i]
#                 elif i == 6:
#                     elementdetector.state = data_wave[i]
#                 elif i == 7:
#                     balance_angle = data_wave[i]
                if i == 0:
                    speed_controller.tmp_speed = data_wave[i]
                elif i == 1:
                    turn_in_kp = data_wave[i]
                elif i == 2:
                    turn_out_kp = data_wave[i]
                elif i == 3:
                    speed_kp = data_wave[i]
        # 将数据发送到示波器
        wireless.send_ccd_image(WIRELESS_UART.ALL_CCD_BUFFER_INDEX)
        wireless.send_oscilloscope(
        #vel_kp, vel_ki, vel_kd, angle_kp, vel_disturbance, angle_disturbance, current_angle
          elementdetector.state,speed_controller.target_speed, element_gyro.data, element_distance.data, ccd_controller.follow 
        #     #imu_data[3], imu_data[4], imu_data[5]
        #     #turn_in_disturbance,turn_output, error
        #     #gyro_bias_x , gyro_bias_y, gyro_bias_z
             )
        gc.collect()
        ticker_flag_8ms = False
    gc.collect()





