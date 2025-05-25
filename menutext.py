from basic_data import *
from ccd_hander import *
import gc
stop_flag=0
point = 30
main_menu_flag = 1
car_go_flag = 0
element_flag = 0
key_cnt=0


def check_speedmode(speed):
    if speed==0:
        return 'default'
    elif speed==1:
        return 'mode_1'
    elif speed==2:
        return 'mode_2'
def check_element(mod):
    if mod==-1:
        return 'stop'
    elif mod==0:                                               
        return 'normal'
    elif mod==1:
        return 'l1'
    elif mod==2:
        return 'l2'
    elif mod==3:
        return 'r1'
    elif mod==4:
        return 'r2'
    elif mod==5:
        return 'l3'
    elif mod==6:
        return 'r3'
    elif mod==7:
        return 'lin'
    elif mod==8:
        return 'lout'
    elif mod==9:
        return 'rin'
    elif mod==10:
        return 'rout'                   
    elif mod==11:
        return 'zebra'
    elif mod==12:
        return 'ramp'
    elif mod==13:
        return 'barrier'
    elif mod==14:
        return 'l3not'
    elif mod==15:
        return 'r3not'

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
        point = 174
        key.clear(3)
    gc.collect()


def sec_menu_01(key_data):
    global main_menu_flag, point, car_go_flag,stop_flag,key_cnt
    lcd.str24(60, 0, "car_go_mode", 0x07E0)
    lcd.str16(16, 30, "stop_flag={} ".format(stop_flag),0xFFFF)
    lcd.str16(16, 46, "car_mode={}".format(check_speedmode(movementtype.speed)), 0x07E0)   #寻圆环/不寻圆环
    lcd.str16(16, 62, "return ",0xFFFF)
    lcd.str16(16, 126, "aim_speed={}".format(movementtype.speed),0xFFFF)
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
            movementtype.speed=0
        elif key_cnt == 1:
            movementtype.speed=40
        elif key_cnt == 2:
            movementtype.speed=80
        elif key_cnt >=3:
            key_cnt=0
            movementtype.speed=0
            
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
    global main_menu_flag, point, element_flag
    lcd.str24(60, 0, "debug", 0x07E0)  # 二级菜单标题
    lcd.str16(16, 30,"cnl[0]:{} cnl[1]:{}".format(elementdetector.ccd_near_l[0],elementdetector.ccd_near_l[1]),0xFFFF)
    lcd.str16(16, 46,"cnr[0]:{} cnr[1]:{}".format(elementdetector.ccd_near_r[0],elementdetector.ccd_near_r[1]),0xFFFF)
    lcd.str16(16, 62,"cfl[0]:{}  cfl[1]:{}".format(elementdetector.ccd_far_l[0],elementdetector.ccd_far_l[1]),0xFFFF)
    lcd.str16(16, 78,"cfr[0]:{}  cfr[1]:{}".format(elementdetector.ccd_far_r[0],elementdetector.ccd_far_r[1]),0xFFFF)
    lcd.str16(16, 94,"stage:{}".format(check_element(elementdetector.state)),0xFFFF)
    lcd.str16(16, 110,"nl:{:<3}   nr:{:<3}".format(ccd_near.left, ccd_near.right),0xFFFF)
    lcd.str16(16, 126,"fl:{:<3}   fr:{:<3}".format(ccd_far.left,ccd_far.right),0xFFFF)
    lcd.str16(16, 142,"near length:{}".format(elementdetector.ccd_near_length),0xFFFF)
    lcd.str16(16, 158,"g:{:.1f} d:{:.3f}".format(gyro_z.data, distance.data),0xFFFF)
    lcd.str16(16, 174,"data_update",0xFFFF)
    lcd.str16(16, 190,"state_reset",0xFFFF)
    lcd.str16(16, 206,"gyro&dis_clear",0xFFFF)
    lcd.str16(16, 222,"return",0xFFFF)
    lcd.str16(0, point, ">", 0xF800)

    point_move(222, 174, key_data)
    if point == 174 and key_data[2]:
        lcd.clear(0x0000)
        elementdetector.debug()
        elementdetector.update()
        elementdetector.state = 0
        key.clear(3)
    
    if point == 190 and key_data[2]:
        lcd.clear(0x0000)
        elementdetector.state = 0
        key.clear(3)
        
    if point == 206 and key_data[2]:
        lcd.clear(0x0000)
        distance.data = 0
        gyro_z.data = 0
        gyro_z._getoffset()
        key.clear(3)
        
    if point == 222 and key_data[2]:
        lcd.clear(0x0000)
        main_menu_flag = 1
        element_flag = 0
        point = 30
        key.clear(3)

    gc.collect()



