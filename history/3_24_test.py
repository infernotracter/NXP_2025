from machine import Pin, SPI
from display import LCD
import gc

class MenuSystem:
    class Menu:
        def __init__(self, title, items, parent=None):
            self.title = title
            self.items = items  # (显示文本, 操作类型, 目标/回调)
            self.parent = parent
            self.selection = 0
            self.y_start = 30  # 菜单起始Y坐标
            self.item_height = 16  # 菜单项行高

        def handle_input(self, key_data, lcd):
            """处理按键输入并返回需要切换的菜单"""
            if key_data[0]:  # 下键
                self.selection = (self.selection + 1) % len(self.items)
            elif key_data[1]:  # 上键
                self.selection = (self.selection - 1) % len(self.items)
            elif key_data[2]:  # 确认键
                _, item_type, target = self.items[self.selection]
                if item_type == 'submenu':
                    return target
                elif item_type == 'action':
                    target()
            elif key_data[3]:  # 返回键
                return self.parent
            return self

        def render(self, lcd):
            """渲染菜单界面"""
            lcd.clear(0x0000)
            lcd.str24(60, 0, self.title, 0x07E0)
            
            # 绘制菜单项
            for idx, (text, _, _) in enumerate(self.items):
                y_pos = self.y_start + idx * self.item_height
                color = 0xFFFF if idx != self.selection else 0xF800
                lcd.str16(16, y_pos, text, color)
            
            # 绘制选择指示器
            selector_y = self.y_start + self.selection * self.item_height
            lcd.str16(0, selector_y, ">", 0xF800)

    def __init__(self, lcd):
        self.lcd = lcd
        self.current_menu = self.create_main_menu()
        
        # 共享状态
        self.aim_speed = 100
        self.aim_speed_l = 0
        self.aim_speed_r = 0
        self.speed_d = 50

    def create_main_menu(self):
        """创建主菜单结构"""
        return self.Menu(
            title="Main Menu",
            items=[
                ("Car Control", 'submenu', self.create_car_control_menu()),
                ("Speed Settings", 'submenu', self.create_speed_menu()),
                ("PID Tuning", 'submenu', self.create_pid_menu()),
                ("CCD Image", 'submenu', self.create_ccd_menu()),
                ("Parameters", 'submenu', self.create_param_menu()),
                ("Save Config", 'action', self.save_config)
            ]
        )

    def create_car_control_menu(self):
        """车辆控制子菜单"""
        return self.Menu(
            title="Car Control",
            items=[
                ("Set Speed 100", 'action', lambda: setattr(self, 'aim_speed', 100)),
                ("Emergency Stop", 'action', self.emergency_stop),
                ("Return", 'submenu', self.current_menu)
            ],
            parent=self.current_menu
        )

    def create_speed_menu(self):
        """速度设置菜单"""
        return self.Menu(
            title="Speed Settings",
            items=[
                (f"Target L: {self.aim_speed_l}", 'adjust', ('aim_speed_l', self.speed_d)),
                (f"Target R: {self.aim_speed_r}", 'adjust', ('aim_speed_r', self.speed_d)),
                ("Return", 'submenu', self.current_menu)
            ],
            parent=self.current_menu
        )

    def create_pid_menu(self):
        """PID调节菜单"""
        return self.Menu(
            title="PID Settings",
            items=[
                ("Angle PID", 'submenu', self.create_angle_pid_menu()),
                ("Speed PID", 'submenu', self.create_speed_pid_menu()),
                ("Gyro PID", 'submenu', self.create_gyro_pid_menu()),
                ("Return", 'submenu', self.current_menu)
            ],
            parent=self.current_menu
        )

    def create_angle_pid_menu(self):
        """角度PID子菜单"""
        return self.Menu(
            title="Angle PID",
            items=[
                (f"KP: {angle_pid.kp:.2f}", 'adjust', ('kp', 0.1)),
                (f"KD: {angle_pid.kd:.2f}", 'adjust', ('kd', 0.1)),
                ("Mid Angle", 'adjust', ('MedAngle', 0.1)),
                ("Return", 'submenu', self.create_pid_menu())
            ],
            parent=self.current_menu
        )

    def update(self, key_data):
        """更新菜单状态"""
        new_menu = self.current_menu.handle_input(key_data, self.lcd)
        if new_menu and new_menu != self.current_menu:
            self.current_menu = new_menu
            self.current_menu.render(self.lcd)
        elif new_menu == self.current_menu:
            self.current_menu.render(self.lcd)  # 仅更新变化项

    def emergency_stop(self):
        """紧急停止功能"""
        motor_l.duty(0)
        motor_r.duty(0)
        self.lcd.str24(60, 100, "EMERGENCY STOP!", 0xF800)

    def save_config(self):
        """配置保存功能"""
        # 实际的保存逻辑
        self.lcd.str24(60, 100, "Config Saved!", 0x07E0)

# 使用示例
lcd = LCD(...)  # 实际初始化参数
menu_system = MenuSystem(lcd)

# 在主循环中
while True:
    key_data = key.get()  # 获取按键状态
    menu_system.update(key_data)
    gc.collect()