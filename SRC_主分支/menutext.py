from basic_data import *
import gc
stop_flag=0
main_point_item = 30
main_menu_flag = 1
car_go_flag = 0
speed_flag = 0
element_flag = 0
angle_pd_flag = 0
speed_pi_flag = 0
gyro_pi_flag = 0
ccd_image_flag = 0
parameter_flag = 0
screen_off_flag = 0
save_para_flag = 0
key_cnt=0
def point_move(hight, low, key_data):
    global main_point_item
    if key_data[1]:
        lcd.clear(0x0000)
        main_point_item += 16
        key.clear(2)
        if main_point_item == hight + 16:
            main_point_item = low
    if key_data[0]:
        lcd.clear(0x0000)
        main_point_item -= 16
        key.clear(1)
        if main_point_item == low - 16:
            main_point_item = hight
    gc.collect()


def menu(key_data):
    global main_menu_flag, car_go_flag, speed_flag, element_flag, angle_pd_flag, speed_pi_flag, gyro_pi_flag, ccd_image_flag, screen_off_flag, save_para_flag
    if (main_menu_flag == 1):
        main_menu(key_data)
    if (car_go_flag == 1):
        sec_menu_01(key_data)
    if (speed_flag == 1):
        sec_menu_02(key_data)
    if (element_flag == 1):
        sec_menu_03(key_data)
    if (angle_pd_flag == 1):
        sec_menu_04(key_data)
    if (speed_pi_flag == 1):
        sec_menu_05(key_data)
    if (gyro_pi_flag == 1):
        sec_menu_06(key_data)
    if (ccd_image_flag == 1):
        sec_menu_07(key_data)
    if (parameter_flag == 1):
        sec_menu_08(key_data)
    if (screen_off_flag == 1):
        sec_menu_09(key_data)
    if (save_para_flag == 1):
        sec_menu_10(key_data)

    gc.collect()


def main_menu(key_data):  # 一级菜单
    global main_point_item, main_menu_flag, car_go_flag, speed_flag, element_flag, angle_pd_flag, speed_pi_flag, gyro_pi_flag, ccd_image_flag, screen_off_flag, save_para_flag
    lcd.str24(60, 0, "main_menu", 0x07E0)
    lcd.str16(16, 30, "car_go", 0xFFFF)
    lcd.str16(16, 46, "speed", 0xFFFF)
    lcd.str16(16, 62, "element//don't touch it", 0xFFFF)
    lcd.str16(16, 78, "angle_pd", 0xFFFF)
    lcd.str16(16, 94, "speed_pi", 0xFFFF)
    lcd.str16(16, 110, "gyro_pi", 0xFFFF)
    lcd.str16(16, 126, "ccd_image", 0xFFFF)
    lcd.str16(16, 142, "parameter", 0xFFFF)
    lcd.str16(16, 158, "screen_off", 0xFFFF)
    lcd.str16(16, 174, "save_para", 0xFFFF)

    lcd.str16(0, main_point_item, ">", 0xF800)
    point_move(174, 30, key_data)

    if main_point_item == 30 and key_data[2]:
        key.clear(3)
        main_menu_flag = 0
        car_go_flag = 1
        main_point_item = 30
        lcd.clear(0x0000)
    if main_point_item == 46 and key_data[2]:
        lcd.clear(0x0000)
        speed_flag = 1
        main_menu_flag = 0
        main_point_item = 30
        key.clear(3)
    if main_point_item == 62 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 0
        element_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 78 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 0
        angle_pd_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 94 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 0
        speed_pi_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 110 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 0
        gyro_pi_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 126 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 0
        ccd_image_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 142 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 0
        parameter_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 158 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 0
        screen_off_flag = 1
        main_point_item = 30
        key.clear(3)
    if main_point_item == 174 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 0
        save_para_flag = 1
        main_point_item = 30
        key.clear(3)

    gc.collect()


def sec_menu_01(key_data):
    global  speed_flag, main_menu_flag, main_point_item, car_go_flag,stop_flag,key_cnt
    lcd.str24(60, 0, "car_go_mode", 0x07E0)
    lcd.str16(16, 30, "stop_flag={} ".format(stop_flag),0xFFFF)
    lcd.str16(16, 46, "car_mode={}".format(startmode.mode), 0x07E0)   #寻圆环/不寻圆环
    lcd.str16(16, 62, "return ",0xFFFF)
    lcd.str16(16, 126, "aim_speed={}".format(startmode.aim_speed),0xFFFF)
    lcd.str12(0, main_point_item, ">", 0xF800)
    point_move(62, 30, key_data)
    
    if main_point_item == 62 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 1
        car_go_flag = 0
        key.clear(3)
        main_point_item = 30
    if key_data[2] and main_point_item == 30:
        lcd.clear(0x0000)
        stop_flag = 1
        key.clear(3)
    if key_data[2] and main_point_item == 46:
        lcd.clear(0x0000)
        key_cnt+=1
        if key_cnt==0:
            startmode.mode=MovementType.default
        if key_cnt==1:
            startmode.mode=MovementType.Mode_1
        if key_cnt==2:
            startmode.mode=MovementType.Mode_2
        if key_cnt==3:
            startmode.mode=MovementType.Mode_3
        if key_cnt==4:
            startmode.mode=MovementType.Mode_4
        if key_cnt==5:
            startmode.mode=MovementType.Mode_5
        if key_cnt>=6:
            key_cnt=0
            startmode.mode=MovementType.default
        startmode._update_()
        key.clear(3)
    gc.collect()


def sec_menu_02(key_data):
    global speed_flag, main_menu_flag, aim_speed_l, aim_speed_r, main_point_item
    lcd.str24(60, 0, "speed", 0x07E0)
    lcd.str16(16, 126, "return", 0xFFFF)
    lcd.str16(16, 30, "aim_speed_l={}".format(aim_speed_l), 0xFFFF)
    lcd.str16(16, 46, "aim_speed_r={}".format(aim_speed_r), 0xFFFF)
    lcd.str16(16, 62, "motor_l.duty={}".format(motor_l.duty()), 0xFFFF)
    lcd.str16(16, 78, "motor_r.duty={}".format(motor_r.duty()), 0xFFFF)
    lcd.str16(16, 94, "encoder_l={:0>4d}".format(encl_data), 0xFFFF)
    lcd.str16(16, 110, "encoder_r={:0>4d}".format(encr_data), 0xFFFF)
    lcd.str16(0, main_point_item, ">", 0xF800)

    point_move(126, 30, key_data)

    if main_point_item == 30:
        if key_data[2]:
            lcd.clear(0x0000)
            aim_speed_l += speed_d
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            aim_speed_l -= speed_d
            key.clear(4)

    if main_point_item == 46:
        if key_data[2]:
            lcd.clear(0x0000)
            aim_speed_r += speed_d
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            aim_speed_r -= speed_d
            key.clear(4)
    if main_point_item == 94 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 1
        speed_flag = 0
        main_point_item = 30
        key.clear(3)

    gc.collect()


#  def sec_menu_03(key_data):  # 目前没用
#      lcd.str24(60, 0, "element", 0x07E0)  # 二级菜单标题
#      lcd.str16(16, 30, "return", 0xFFFF)  # 返回主菜单
#
#      if (flag is '左圆环1'):
#          lcd.str16(16, 110, 'left_round1', 0xFFFF)  # 左圆环1
#      elif (flag is '左圆环2'):
#          lcd.str16(16, 110, 'left_round1', 0xFFFF)  # 左圆环2
#      elif (flag is '右圆环1'):
#          lcd.str16(16, 110, 'right_round1', 0xFFFF)  # 右圆环1
#      elif (flag is '右圆环2'):
#          lcd.str16(16, 126, 'right_round2', 0xFFFF)  # 右圆环2
#      elif (flag is '斑马线'):
#          lcd.str16(16, 142, 'zebra', 0xFFFF)  # 斑马线
#
#      gc.collect()


def sec_menu_04(key_data):
    global main_point_item, main_menu_flag, angle_pd_flag, MedAngle
    lcd.str24(60, 0, "angle_pd", 0x07E0)
    lcd.str16(16, 30, "angle_KP={}".format(angle_pid.kp, '.2f'), 0xFFFF)
    lcd.str16(16, 46, "angle_Kp+/- 0.01", 0xFFFF)
    lcd.str16(16, 62, "angle_Kp+/- 0.1", 0xFFFF)
    lcd.str16(16, 78, "angle_KD={}".format(angle_pid.kd, '.2f'), 0xFFFF)
    lcd.str16(16, 94, "angle_KD+/- 0.01", 0xFFFF)
    lcd.str16(16, 110, "angle_KD+/- 0.1", 0xFFFF)
    # lcd.str16(16, 126,"mid+/-0.1",0xFFFF)
    lcd.str16(16, 126, "medAngle={}".format(MedAngle, '.2f'), 0xFFFF)
    lcd.str16(0, main_point_item, ">", 0xF800)

    lcd.str16(16, 142, "return", 0xFFFF)

    point_move(142, 30, key_data)

    if main_point_item == 46:
        if key_data[2]:
            lcd.clear(0x0000)
            angle_pid.kp += 0.01
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            angle_pid.kp -= 0.01
            key.clear(4)
    if main_point_item == 62:
        if key_data[2]:
            lcd.clear(0x0000)
            angle_pid.kp += 0.1
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            angle_pid.kp -= 0.1
            key.clear(4)
    if main_point_item == 94:
        if key_data[2]:
            lcd.clear(0x0000)
            angle_pid.kd += 0.01
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            angle_pid.kd -= 0.01
            key.clear(4)
    if main_point_item == 110:
        if key_data[2]:
            angle_pid.kd += 0.1
            key.clear(3)
        if key_data[3]:
            angle_pid.kd -= 0.1
            key.clear(4)
    if main_point_item == 126:
        if key_data[2]:
            lcd.clear(0x0000)
            MedAngle += 0.1
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            MedAngle -= 0.1
            key.clear(4)

    if main_point_item == 142:
        if key_data[2]:
            key.clear(3)
            lcd.clear(0x0000)
            angle_pd_flag = 0
            main_menu_flag = 1
            main_point_item = 30

    gc.collect()


def sec_menu_05(key_data):
    global speed_pi_flag, main_menu_flag, main_point_item
    lcd.str24(60, 0, "speed_pi", 0x07E0)
    lcd.str16(16, 30, "speed_kp={}".format(speed_pid.kp), 0xFFFF)
    lcd.str16(16, 46, "speed_kp +/- 0.01", 0xFFFF)
    lcd.str16(16, 62, "speed_kp +/- 0.1", 0xFFFF)
    lcd.str16(16, 78, "speed_ki={}".format(speed_pid.ki), 0xFFFF)
    lcd.str16(16, 94, "speed_ki +/- 0.01", 0xFFFF)
    lcd.str16(16, 110, "speed_ki +/- 0.1", 0xFFFF)
    lcd.str16(16, 126, "motor_l.duty()={:0>4d}".format(motor_l.duty()), 0xFFFF)
    lcd.str16(16, 142, "motor_r.duty()={:0>4d}".format(motor_r.duty()), 0xFFFF)
    lcd.str16(0, main_point_item, ">", 0xF800)

    # lcd.str16(16, 126, "return", 0xFFFF)
    point_move(126, 30, key_data)

    if main_point_item == 46:
        if key_data[2]:
            lcd.clear(0x0000)
            speed_pid.kp += 0.01
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            speed_pid.kp -= 0.01
            key.clear(4)
    if main_point_item == 62:
        if key_data[2]:
            lcd.clear(0x0000)
            speed_pid.kp += 0.1
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            speed_pid.kp -= 0.1
            key.clear(4)
    if main_point_item == 94:
        if key_data[2]:
            lcd.clear(0x0000)
            speed_pid.ki += 0.01
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            speed_pid.ki -= 0.01
            key.clear(4)
    if main_point_item == 110:
        if key_data[2]:
            lcd.clear(0x0000)
            speed_pid.ki += 0.1
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            speed_pid.ki -= 0.1
            key.clear(4)
    if main_point_item == 126:
        if key_data[2]:
            speed_pi_flag = 0
            main_menu_flag = 1
            lcd.clear(0x0000)
            main_point_item = 30
            key.clear(3)

    gc.collect()


def sec_menu_06(key_data):
    global main_point_item, main_menu_flag, gyro_pi_flag
    lcd.str24(60, 0, "gyro_pi", 0x07E0)
    lcd.str16(16, 30, "gyro_kp={}".format(gyro_pid.kp), 0xFFFF)
    lcd.str16(16, 46, "gyro_kp+/- 0.01", 0xFFFF)
    lcd.str16(16, 62, "gyro_kp+/- 0.1", 0xFFFF)
    lcd.str16(16, 78, "gyro_kd={}".format(gyro_pid.kd), 0xFFFF)
    lcd.str16(16, 94, "gyro_kd+/- 0.01", 0xFFFF)
    lcd.str16(16, 110, "gyro_kd+/- 0.1", 0xFFFF)
    lcd.str16(0, main_point_item, ">", 0xF800)
    lcd.str16(16, 126, "return", 0xFFFF)

    point_move(126, 30, key_data)
    if main_point_item == 46:
        if key_data[2]:
            lcd.clear(0x0000)
            gyro_pid.kp += 0.1
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            gyro_pid.kp -= 0.1
            key.clear(4)
    if main_point_item == 62:
        if key_data[2]:
            lcd.clear(0x0000)
            gyro_pid.kp += 1
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            gyro_pid.kp -= 1
            key.clear(4)
    if main_point_item == 94:
        if key_data[2]:
            lcd.clear(0x0000)
            gyro_pid.kd += 0.01
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            gyro_pid.kd -= 0.01
            key.clear(4)
    if main_point_item == 110:
        if key_data[2]:
            lcd.clear(0x0000)
            gyro_pid.kd += 0.1
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            gyro_pid.kd -= 0.1
            key.clear(4)
    if main_point_item == 126:
        if key_data[2]:
            lcd.clear(0x0000)
            gyro_pi_flag = 0
            main_menu_flag = 1
            main_point_item = 30
            key.clear(3)

    gc.collect()


def sec_menu_07(key_data):
    global main_point_item, main_menu_flag, ccd_image_flag, ccd_data1, ccd_data2
    lcd.str24(60, 0, "ccd,image", 0x07E0)
    lcd.str16(16, 30, "return", 0xFFFF)
    lcd.wave(0, 64, 128, 64, ccd_data1)
    lcd.wave(0, 64, 128, 64, ccd_data2)
    lcd.line(64, 64, 64, 192, color=0x001F, thick=1)
    if key_data[2]:
        lcd.clear(0x0000)
        ccd_image_flag = 0
        main_menu_flag = 1
        key.clear(3)

    gc.collect()


def sec_menu_08(key_data):
    global main_point_item, parameter_flag, error1, error2
    global Mid_point1, Mid_point2, tof_data, out_l, out_r, encl_data, encr_data
    global left_point_1, right_point_1, left_point_2, right_point_2
    lcd.str24(60, 0, "parameter", 0x07E0)  # 二级菜单标题
    lcd.str16(16, 30, "return", 0xFFFF)  # 返回主菜单

    lcd.str16(16, 46, "tof = {:<4d}".format(tof_data), 0xFFFF)  # TOF数据
    lcd.str16(16, 62, "out_l = {:<4d}".format(out_l), 0xFFFF)  # 左环pid输出
    lcd.str16(16, 78, "out_r = {:<4d}".format(out_r), 0xFFFF)  # 右环pid输出
    lcd.str16(16, 94, "encl = {:<4d}".format(encl_data), 0xFFFF)  # 左编码器值
    lcd.str16(16, 110, "encr = {:<4d}".format(encr_data), 0xFFFF)  # 右编码器值
    lcd.str16(16, 126, "Mid_point1 = {:<.2f}".format(
        Mid_point1), 0xFFFF)  # ccd1中点
    lcd.str16(16, 142, "Mid_point2 = {:<.2f}".format(
        Mid_point2), 0xFFFF)  # ccd2中点
    lcd.str16(16, 158, "error1 = {:<.2f}".format(error1), 0xFFFF)  # ccd1误差
    lcd.str16(16, 174, "error2 = {:<.2f}".format(error2), 0xFFFF)  # ccd2误差
    lcd.str16(16, 190, "left_point_1 = {:<3d}".format(
        left_point_1), 0xFFFF)  # 上摄像头左边点
    lcd.str16(16, 206, "right_point_1 = {:<3d}".format(
        right_point_1), 0xFFFF)  # 上摄像头右边点
    lcd.str16(16, 222, "left_point_2 = {:<3d}".format(
        left_point_2), 0xFFFF)  # 下摄像头左边点
    lcd.str16(16, 238, "right_point_2 = {:<3d}".format(
        right_point_2), 0xFFFF)  # 下摄像头右边点
    lcd.str16(16, 254, "width_1 = {:<3d}".format(
        right_point_1 - left_point_1), 0xFFFF)  # ccd1计算赛道宽度
    lcd.str16(16, 270, "width_2 = {:<3d}".format(
        right_point_2 - left_point_2), 0xFFFF)  # ccd2计算赛道宽度

    if main_point_item == 30:
        if key_data[2]:
            lcd.clear(0x0000)
            parameter_flag = 0
            main_menu_flag = 1
            key.clear(3)

    gc.collect()


def sec_menu_09(key_data):
    lcd.clear(0x0000)
    return


def sec_menu_10(key_data):
    #     write_flash()  # 写入缓冲区
    lcd.clear(0x0000)

    #    buzzer.value(1)  # 蜂鸣器开
    lcd.clear(0xF800)  # 清屏
    time.sleep_ms(100)  # 延时
    main_menu_item = 1  # 返回一级菜单

