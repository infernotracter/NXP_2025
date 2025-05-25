from basic_data import *
from ccd_hander import *
import gc
stop_flag=0
point = 30
main_menu_flag = 1
car_go_flag = 0
element_flag = 0
key_cnt=0
def point_move(hight, low, key_data):
    global point
    if key_data[1]:
        lcd.clear(0x0000)
        point += 16
        key.clear(2)
        if point == hight + 16:
            point = low
    if key_data[0]:
        lcd.clear(0x0000)
        point -= 16
        key.clear(1)
        if point == low - 16:
            point = hight
    gc.collect()



def menu(key_data):
    global main_menu_flag, car_go_flag, element_flag
    if (main_menu_flag == 1):
        main_menu(key_data)
    if (car_go_flag == 1):
        sec_menu_01(key_data)
    if (element_flag == 1):
        sec_menu_02(key_data)

    gc.collect()


def main_menu(key_data):  # 一级菜单
    global point, main_menu_flag, car_go_flag,element_flag
    lcd.str16(16, 30, "car_go", 0xFFFF)
    lcd.str16(16, 46, "element_debug", 0xFFFF)
    lcd.str16(0, point, ">", 0xF800)
    point_move(46, 30, key_data)

    if point == 30 and key_data[2]:
        key.clear(3)
        main_menu_flag = 0
        car_go_flag = 1
        point = 30
        lcd.clear(0x0000)
    if point == 46 and key_data[2]:
        lcd.clear(0x0000)
        element_flag = 1
        main_menu_flag = 0
        point = 158
        key.clear(3)
    gc.collect()


def sec_menu_01(key_data):
    global main_menu_flag, point, car_go_flag,stop_flag,key_cnt
    lcd.str24(60, 0, "car_go_mode", 0x07E0)
    lcd.str16(16, 30, "stop_flag={} ".format(stop_flag),0xFFFF)
    lcd.str16(16, 46, "car_mode={}".format(movementtype.aim_speed), 0x07E0)   #寻圆环/不寻圆环
    lcd.str16(16, 62, "return ",0xFFFF)
    lcd.str16(16, 126, "aim_speed={}".format(movementtype.aim_speed),0xFFFF)
    lcd.str12(0, point, ">", 0xF800)
    point_move(62, 30, key_data)
    
    if point == 62 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 1
        car_go_flag = 0
        key.clear(3)
        point = 30
    if key_data[2] and point == 30:
        lcd.clear(0x0000)
        stop_flag = 1
        key.clear(3)
    if key_data[2] and point == 46:
        key_cnt+=1
        if key_cnt == 0:
            movementtype.aim_speed=0
        elif key_cnt == 1:
            movementtype.aim_speed=-100
        elif key_cnt == 2:
            movementtype.aim_speed=-200
        elif key_cnt == 3:
            movementtype.aim_speed=-300
        elif key_cnt == 4:
            movementtype.aim_speed=-400

        elif key_cnt>4:
            key_cnt=0
#         if key_cnt==0:
#             movementtype.mode=MovementType.default
#         if key_cnt==1:
#             movementtype.mode=MovementType.Mode_1
#         if key_cnt==2:
#             movementtype.mode=MovementType.Mode_2
#         if key_cnt==3:
#             movementtype.mode=MovementType.Mode_3
#         if key_cnt==4:
#             movementtype.mode=MovementType.Mode_4
#         if key_cnt==5:
#             movementtype.mode=MovementType.Mode_5
#         if key_cnt>=6:
#             key_cnt=0
#             movementtype.mode=MovementType.default
        #movementtype.update()
        lcd.clear(0x0000)
        key.clear(3)
    gc.collect()


def sec_menu_02(key_data):  #元素debug
    global main_menu_flag, point,element_flag
    lcd.str24(60, 0, "debug", 0x07E0)  # 二级菜单标题
    lcd.str16(16, 30,"ccd_near_l[0]:{}   ,ccd_near_l[1]:{}".format(elementdetector.ccd_near_l[0],elementdetector.ccd_near_l[1]),0xFFFF)
    lcd.str16(16, 46,"ccd_near_r[0]:{}   ,ccd_near_r[1]:{}".format(elementdetector.ccd_near_r[0],elementdetector.ccd_near_r[1]),0xFFFF)
    lcd.str16(16, 62,"ccd_far_l[0]:{}   ,ccd_far_l[1]:{}".format(elementdetector.ccd_far_l[0],elementdetector.ccd_far_l[1]),0xFFFF)
    lcd.str16(16, 78,"ccd_far_r[0]:{}   ,ccd_far_r[1]:{}".format(elementdetector.ccd_far_r[0],elementdetector.ccd_far_r[1]),0xFFFF)
    lcd.str16(16, 94,"movementtype.mode:{}  ,stage:{}".format(movementtype.mode,elementdetector.state),0xFFFF)
    lcd.str16(16, 110,"ccd_near.left:{} , ccd_near.right:{}".format(ccd_near.left, ccd_near.right),0xFFFF)
    lcd.str16(16, 126,"ccd_far.left:{} , ccd_far.right:{}".format(ccd_far.left,ccd_far.right),0xFFFF)
    lcd.str16(16, 142,"ccd_near_length:{} , gyro:{:.1f} , dis:{:.3f}".format(elementdetector.ccd_near_length,gyro_z.data, distance.data),0xFFFF)
    lcd.str16(16, 158,"data_update",0xFFFF)
    lcd.str16(16, 174,"gyro&dis_clear",0xFFFF)
    lcd.str16(16, 190,"return",0xFFFF)
    lcd.str12(0, point, ">", 0xF800)

    point_move(174, 158, key_data)
    if point == 158 and key_data[2]:
        lcd.clear(0x0000)
        elementdetector.update()
        elementdetector.state = 0
        key.clear(3)
    if point == 174 and key_data[2]:
        lcd.clear(0x0000)
        distance.data = 0
        gyro_z.data = 0
        gyro_z._getoffset()
        key.clear(3)
    if point == 190 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 1
        element_flag = 0
        point = 30
        key.clear(3)

    gc.collect()


