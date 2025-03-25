from handware import *
from pid_controller import *
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
    if key_data[0]:
        lcd.clear(0x0000)
        main_point_item += 16
        key.clear(1)
        if main_point_item == 190:
            main_point_item = 30

    if key_data[1]:
        lcd.clear(0x0000)
        main_point_item -= 16
        key.clear(2)
        if main_point_item == 14:
            main_point_item = 174

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
    global aim_speed, speed_flag, main_menu_flag, main_point_item, car_go_flag
    lcd.str24(60, 0, "car_go", 0x07E0)
    lcd.str16(16, 62, "return", 0xFFFF)
    lcd.str16(16, 46, "It's mygo", 0xFFFF)
    lcd.str12(0, main_point_item, ">", 0xF800)

    if key_data[0]:
        lcd.clear(0x0000)
        main_point_item += 16
        key.clear(1)
        if main_point_item == 78:
            main_point_item = 30
    if key_data[1]:
        lcd.clear(0x0000)
        main_point_item -= 16
        key.clear(2)
        if main_point_item == 14:
            main_point_item = 62
    if main_point_item == 62 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 1
        car_go_flag = 0
        key.clear(3)
        main_point_item = 30
    if key_data[2] and main_point_item == 46:
        lcd.clear(0x0000)
        aim_speed = 100
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

    if key_data[0]:
        main_point_item += 16
        lcd.clear(0x0000)
        key.clear(1)
        if main_point_item == 78:
            main_point_item = 30

    if key_data[1]:
        main_point_item -= 16
        lcd.clear(0x0000)
        key.clear(2)
        if main_point_item == 14:
            main_point_item = 62

    if main_point_item == 30:
        if key_data[2]:
            lcd.clear(0x0000)
            aim_speed_l += sys.speed_d
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            aim_speed_l -= sys.speed_d
            key.clear(4)

    if main_point_item == 46:
        if key_data[2]:
            lcd.clear(0x0000)
            aim_speed_r += sys.speed_d
            key.clear(3)
        if key_data[3]:
            lcd.clear(0x0000)
            aim_speed_r -= sys.speed_d
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

    # lcd.str16(16, 126, "return", 0xFFFF)

    if key_data[0]:
        main_point_item += 16
        lcd.clear(0x0000)
        key.clear(1)
        if main_point_item == 158:
            main_point_item = 30

    if key_data[1]:
        main_point_item -= 16
        lcd.clear(0x0000)
        key.clear(2)
        if main_point_item == 14:
            main_point_item = 142

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
    if key_data[0]:
        main_point_item += 16
        lcd.clear(0x0000)
        key.clear(1)
        if main_point_item == 142:
            main_point_item = 30
    if key_data[1]:
        main_point_item -= 16
        lcd.clear(0x0000)
        key.clear(2)
        if main_point_item == 14:
            main_point_item = 126
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
    lcd.str16(16, 126, "motor_l.duty()={:0>4d}".format(motor_l.duty()), 0xFFFF)
    lcd.str16(16, 142, "motor_r.duty()={:0>4d}".format(motor_r.duty()), 0xFFFF)
    lcd.str16(0, main_point_item, ">", 0xF800)

    # lcd.str16(16, 126, "return", 0xFFFF)
    if key_data[0]:
        main_point_item += 16
        lcd.clear(0x0000)
        key.clear(1)
        if main_point_item == 142:
            main_point_item = 30
    if key_data[1]:
        main_point_item -= 16
        lcd.clear(0x0000)
        key.clear(2)
        if main_point_item == 14:
            main_point_item = 126
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


#    buzzer.value(0)  # 蜂鸣器关


# 写入缓冲区
# def write_flash():
#     global angle_pid.kp, angle_pid.kd, speed_pid.kp, speed_pid.ki, aim_speed_l, aim_speed_r
#     os.chdir("/flash")  # 切换到 /flash 目录
#     try:
#         # 通过 try 尝试打开文件 因为 r+ 读写模式不会新建文件
#         user_file = io.open("user_data.txt", "r+")
#     except:
#         # 如果打开失败证明没有这个文件 所以使用 w+ 读写模式新建文件
#         user_file = io.open("user_data.txt", "w+")
#
#     # 将指针移动到文件头 0 偏移的位置
#     user_file.seek(0, 0)
#     # 使用 write 方法写入数据到缓冲区
#
#     user_file.write("%.4f\n" % (angle_pid.kp))
#     user_file.write("%.4f\n" % (angle_pid.kd))
#     user_file.write("%.4f\n" % (speed_pid.kp))
#     user_file.write("%.4f\n" % (speed_pid.ki))
#     user_file.write("%d\n" % (aim_speed_l))
#     user_file.write("%d\n" % (aim_speed_r))
#
#     # 将缓冲区数据写入到文件 清空缓冲区 相当于保存指令
#     user_file.flush()
#
#     # 将指针重新移动到文件头
#     user_file.seek(0, 0)
#     # 读取三行数据 到临时变量 分别强制转换回各自类型
#     data1 = float(user_file.readline())
#     data2 = float(user_file.readline())
#     data3 = float(user_file.readline())
#     data4 = float(user_file.readline())
#     data5 = int(user_file.readline())
#     data6 = int(user_file.readline())
#
#     # 最后将文件关闭即可
#     user_file.close()