class MenuSystem:
    def __init__(self, lcd):
        self.lcd = lcd
        self.current_menu = 'main'
        self.menu_stack = []
        self.selected_item = 0
        
        self.menus = {
            'main': {
                'title': "Main Menu",
                'items': [
                    ("Car Control", lambda: self.change_menu('car_control')),
                    ("Speed Control", lambda: self.change_menu('speed_control')),
                    ("PID Settings", lambda: self.change_menu('pid_settings')),
                    ("Sensor Data", lambda: self.change_menu('sensor_data')),
                    ("System Config", lambda: self.change_menu('system_config')),
                ]
            },
            'car_control': {
                'title': "Car Control",
                'items': [
                    ("Start Engine", self.start_engine),
                    ("Set Speed", lambda: self.change_menu('speed_set')),
                    ("Back", self.back)
                ]
            },
            'speed_control': {
                'title': "Speed Control",
                'items': [
                    ("Aim Speed L: {}".format(self.aim_speed_l), self.adjust_aim_speed_l),
                    ("Aim Speed R: {}".format(self.aim_speed_r), self.adjust_aim_speed_r),
                    ("Back", self.back)
                ]
            },
            'pid_settings': {
                'title': "PID Settings",
                'items': [
                    ("Angle KP: {:.2f}".format(self.angle_pid.kp), self.adjust_angle_kp),
                    ("Angle KD: {:.2f}".format(self.angle_pid.kd), self.adjust_angle_kd),
                    ("Speed KP: {:.2f}".format(self.speed_pid.kp), self.adjust_speed_kp),
                    ("Speed KI: {:.2f}".format(self.speed_pid.ki), self.adjust_speed_ki),
                    ("Back", self.back)
                ]
            },
            'sensor_data': {
                'title': "Sensor Data",
                'items': [
                    ("TOF: {}".format(self.tof_data), None),
                    ("Encoder L: {}".format(self.encl_data), None),
                    ("Encoder R: {}".format(self.encr_data), None),
                    ("Back", self.back)
                ]
            },
            'system_config': {
                'title': "System Config",
                'items': [
                    ("Screen Off", self.screen_off),
                    ("Save Config", self.save_config),
                    ("Back", self.back)
                ]
            }
        }
        
        # 初始化参数
        self.aim_speed_l = 0
        self.aim_speed_r = 0
        self.angle_pid = PID(kp=1.0, kd=0.5)
        self.speed_pid = PID(kp=0.8, ki=0.1)
        self.tof_data = 0
        self.encl_data = 0
        self.encr_data = 0

    def display_menu(self):
        menu = self.menus[self.current_menu]
        self.lcd.clear(0x0000)
        self.lcd.str24(60, 0, menu['title'], 0x07E0)
        
        for idx, item in enumerate(menu['items']):
            # 动态生成文本
            text = item[0]
            if callable(text):
                text = text()
            y_pos = 30 + idx*16
            color = 0xF800 if idx == self.selected_item else 0xFFFF
            self.lcd.str16(16, y_pos, text, color)

    def handle_input(self, key_data):
        if key_data[0]:  # Down
            self.selected_item = min(self.selected_item+1, len(self.menus[self.current_menu]['items'])-1)
        elif key_data[1]:  # Up
            self.selected_item = max(self.selected_item-1, 0)
        elif key_data[2]:  # Enter
            item = self.menus[self.current_menu]['items'][self.selected_item]
            if callable(item[1]):
                item[1]()
        elif key_data[3]:  # Back
            self.back()

    def change_menu(self, new_menu):
        self.menu_stack.append(self.current_menu)
        self.current_menu = new_menu
        self.selected_item = 0

    def back(self):
        if self.menu_stack:
            self.current_menu = self.menu_stack.pop()
            self.selected_item = 0

    # 功能回调函数
    def start_engine(self):
        print("Engine started")

    def adjust_aim_speed_l(self):
        # 这里可以进入速度调整子菜单或直接修改值
        self.aim_speed_l += 10  # 示例调整

    def adjust_aim_speed_r(self):
        self.aim_speed_r += 10

    def adjust_angle_kp(self):
        self.angle_pid.kp += 0.01  # 示例调整

    def adjust_angle_kd(self):
        self.angle_pid.kd += 0.01

    def adjust_speed_kp(self):
        self.speed_pid.kp += 0.01

    def adjust_speed_ki(self):
        self.speed_pid.ki += 0.01

    def screen_off(self):
        self.lcd.clear(0x0000)

    def save_config(self):
        print("Config saved")

