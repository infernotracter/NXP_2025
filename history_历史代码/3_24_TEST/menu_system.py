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
                    ("Car Control", self.car_control_menu),
                    ("PID Settings", self.pid_settings_menu),
                    ("Sensor Data", self.sensor_data_menu),
                    ("System Config", self.system_config_menu)
                ]
            },
            'car_control': {
                'title': "Car Control",
                'items': [
                    ("Start Engine", self.start_engine),
                    ("Speed Control", self.speed_control),
                    ("Back", lambda: self.back())
                ]
            }
        }
    
    def display_menu(self):
        menu = self.menus[self.current_menu]
        self.lcd.clear(0x0000)
        self.lcd.str24(60, 0, menu['title'], 0x07E0)
        
        for idx, item in enumerate(menu['items']):
            y_pos = 30 + idx*16
            color = 0xF800 if idx == self.selected_item else 0xFFFF
            self.lcd.str16(16, y_pos, item[0], color)
    
    def handle_input(self, key_data):
        if key_data[0]:  # Down
            self.selected_item = min(self.selected_item+1, len(self.menus[self.current_menu]['items'])-1)
        elif key_data[1]:  # Up
            self.selected_item = max(self.selected_item-1, 0)
        elif key_data[2]:  # Enter
            self.menus[self.current_menu]['items'][self.selected_item][1]()
    
    def back(self):
        if self.menu_stack:
            self.current_menu = self.menu_stack.pop()