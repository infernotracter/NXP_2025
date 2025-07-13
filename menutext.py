from basic_data import *
from ccd_handler import *


class MenuText:
    def __init__(self):
        self.main_menu = True
        self.ccd_menu = False
        self.turn_menu = False
        self.speed_meun = False
        self.ring_menu = False
        self.key=[0, 0, 0, 0]
        self.value = 31
        self.string_size = 16
        self.point = 30
    def show_controll(self):
        if self.main_menu:
            self.show_main_menu()
        if self.ccd_menu:
            self.show_ccd_menu()
        if self.turn_menu:
            self.show_turn_menu()
        if self.speed_meun:
            self.show_speed_menu()
        if self.ring_menu:
            self.show_ring_menu()
    def point_move(self, hight, low):
        if self.key[0]:
            lcd.clear(0x0000)
            self.point += self.string_size
            key.clear(1)
            if self.point == hight + self.string_size:
                self.point = low
        if self.key[1]:
            lcd.clear(0x0000)
            self.point -= self.string_size
            key.clear(2)
            if self.point == low - self.string_size:
                self.point = hight
    def show_main_menu(self):
        lcd.str16(16, 30, "ccd", 0xFFFF)
        lcd.str16(16, 46, "turn", 0xFFFF)
        lcd.str16(16, 62, "speed", 0xFFFF)
        lcd.str16(16, 78, "ring", 0xFFFF)
        lcd.str16(0, self.point, ">", 0xF800)
        self.point_move(78, 30)
        if self.point == 30 and self.key[2]:
            key.clear(3)
            self.main_menu = False
            self.ccd_menu = True
            self.point = 30
            lcd.clear(0x0000)
        
        if self.point == 46 and self.key[2]:
            lcd.clear(0x0000)
            self.turn_menu = True
            self.main_menu = False
            self.point = 30
            key.clear(3)
        if self.point == 62 and self.key[2]:
            lcd.clear(0x0000)
            self.speed_meun = True
            self.main_menu = False
            self.point = 30
            key.clear(3)
        if self.point == 78 and self.key[2]:
            lcd.clear(0x0000)
            self.ring_menu = True
            self.main_menu = False
            self.point = 30
            key.clear(3)
        
    def show_ccd_menu(self):
        lcd.wave(0,  0, 128, 64, ccd_near.data, max = 255)
        lcd.wave(0,  64, 128, 64, ccd_far.data, max = 255)
        lcd.str16(16 , 144, "value={}".format(self.value), 0xFFFF)
        lcd.str16(16, 160,"return" , 0xFFFF)
        lcd.str16(0, self.point, ">", 0xF800)
        self.point_move(160, 144)
        if self.point == 144:
            if self.key[2]:
                key.clear(3)
                self.value += 1
                lcd.clear(0x0000)
            if self.key[3]:
                key.clear(4)
                self.value -= 1
                lcd.clear(0x0000)
        if self.point == 160 and self.key[2]:
            key.clear(3)
            self.ccd_menu = False
            self.main_menu = True
            self.point = 30
            lcd.clear(0x0000)