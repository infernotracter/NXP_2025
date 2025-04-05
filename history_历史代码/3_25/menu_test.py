from handware import *
from pid_controller import *
import gc

class MenuSystem:
    def __init__(self):
        self.current_menu = None
        self.selected_index = 0
        self.menu_stack = []
        self.variable_temp = 0.0 # 临时变量存储
        self.adjust_mode = False  # 新增调整模式状态标志
        
        # 菜单结构配置
        self.menus = {
            'main': {
                'title': "Main Menu",
                'items': [
                    {'text': "Car Control", 'submenu': 'car_control'},
                    {'text': "Speed Settings", 'submenu': 'speed_settings'},
                    {'text': "PID Parameters", 'submenu': 'pid_menu'},
                    {'text': "Sensor Data", 'submenu': 'sensor_data'},
                    {'text': "System Actions", 'submenu': 'system_actions'}
                ]
            },
            'car_control': {
                'title': "Car Control",
                'items': [
                    {'text': "Start Movement", 'action': lambda: setattr(sys, 'aim_speed', 100)},
                    {'text': "Return", 'action': self.back}
                ]
            },
            'speed_settings': {
                'title': "Speed Settings",
                'items': [
                    {'text': f"Aim Speed L: {sys.aim_speed_l}", 
                     'adjust': {'var': 'sys.aim_speed_l', 'step': 1}},
                    {'text': f"Aim Speed R: {sys.aim_speed_r}", 
                     'adjust': {'var': 'sys.aim_speed_r', 'step': 1}},
                    {'text': "Return", 'action': self.back}
                ]
            },
            'pid_menu': {
                'title': "PID Parameters",
                'items': [
                    {'text': f"Angle KP: {angle_pid.kp:.2f}", 
                     'adjust': {'var': 'angle_pid.kp', 'step': 0.01}},
                    {'text': f"Angle KD: {angle_pid.kd:.2f}", 
                     'adjust': {'var': 'angle_pid.kd', 'step': 0.01}},
                    {'text': f"Speed KP: {speed_pid.kp:.2f}", 
                     'adjust': {'var': 'speed_pid.kp', 'step': 0.01}},
                    {'text': f"Speed KI: {speed_pid.ki:.2f}", 
                     'adjust': {'var': 'speed_pid.ki', 'step': 0.01}},
                    {'text': "Return", 'action': self.back}
                ]
            },
            'sensor_data': {
                'title': "Sensor Data",
                'items': [
                    {'text': f"Encoder L: {sys.encl_data}"},
                    {'text': f"Encoder R: {sys.encr_data}"},
                    {'text': f"TOF: {sys.tof_data}"},
                    {'text': "CCD Viewer", 'submenu': 'ccd_view'},
                    {'text': "Return", 'action': self.back}
                ]
            },
            'ccd_view': {
                'title': "CCD Viewer",
                'on_enter': self.show_ccd,
                'items': [
                    {'text': "Return", 'action': self.back}
                ]
            },
            'system_actions': {
                'title': "System Actions",
                'items': [
                    {'text': "Save Parameters", 'action': self.save_params},
                    {'text': "Screen Off", 'action': self.screen_off},
                    {'text': "Return", 'action': self.back}
                ]
            }
        }
        
        self.current_menu = self.menus['main']
    
    def back(self):
        if self.menu_stack:
            self.current_menu = self.menu_stack.pop()
            self.selected_index = 0
            self.adjust_mode = False  # 退出调整模式
            
    def show_ccd(self):
        lcd.wave(0, 64, 128, 64, sys.ccd_data1)
        lcd.wave(0, 64, 128, 64, sys.ccd_data2)
        lcd.line(64, 64, 64, 192, color=0x001F, thick=1)
    
    def save_params(self):
        # 这里添加保存参数的实现
        lcd.clear(0xF800)
        time.sleep_ms(100)
        self.back()
    
    def screen_off(self):
        lcd.clear(0x0000)
    
    def handle_input(self, key_data):
        if self.adjust_mode:
            # 调整模式下的按键处理
            if key_data[0]:  # 上键增加
                step = self.current_menu['items'][0].get('step', 0.01)
                self.variable_temp += step
                self.update_adjust_text()
                key.clear(0)
            elif key_data[1]:  # 下键减少
                step = self.current_menu['items'][0].get('step', 0.01)
                self.variable_temp -= step
                self.update_adjust_text()
                key.clear(1)
            elif key_data[2]:  # 确认键保存
                self.current_menu['items'][0]['confirm']()
                self.adjust_mode = False
                self.back()
                key.clear(2)
            elif key_data[3]:  # 返回键取消
                self.adjust_mode = False
                self.back()
                key.clear(3)
        else:
            # 正常模式下的按键处理
            if key_data[0]:  # 上
                self.selected_index = max(0, self.selected_index - 1)
                key.clear(0)
                lcd.clear(0x0000)
            elif key_data[1]:  # 下
                self.selected_index = min(len(self.current_menu['items'])-1, 
                                        self.selected_index + 1)
                key.clear(1)
                lcd.clear(0x0000)
            elif key_data[2]:  # 确认
                self.handle_confirm_action()
                key.clear(2)
            elif key_data[3]:  # 返回
                self.back()
                key.clear(3)
    
    def render(self):
        # 显示标题
        lcd.str24(60, 0, self.current_menu['title'], 0x07E0)
        
        # 显示菜单项
        y = 30
        for i, item in enumerate(self.current_menu['items']):
            color = 0xFFFF if i != self.selected_index else 0xF800
            text = item['text']
            if 'adjust' in item:
                var_path = item['adjust']['var']
                text = text.split(':')[0] + f": {eval(var_path):.2f}"
            lcd.str16(16, y, text, color)
            y += 16
            
        # 显示光标
        lcd.str16(0, 30 + self.selected_index*16, ">", 0xF800)
        
        # 特殊渲染（如CCD显示）
        if 'on_enter' in self.current_menu:
            self.current_menu['on_enter']()
        
    
    def handle_confirm_action(self):
        item = self.current_menu['items'][self.selected_index]
        if 'submenu' in item:
            self.enter_submenu(item['submenu'])
        elif 'action' in item:
            item['action']()
            lcd.clear(0x0000)
        elif 'adjust' in item:
            self.enter_adjust_mode(item)
    
    def enter_submenu(self, submenu):
        self.menu_stack.append(self.current_menu)
        self.current_menu = self.menus[submenu]
        self.selected_index = 0
        if 'on_enter' in self.current_menu:
            self.current_menu['on_enter']()
        lcd.clear(0x0000)
    
    def enter_adjust_mode(self, item):
        var_path = item['adjust']['var']
        self.variable_temp = eval(var_path)
        self.menu_stack.append(self.current_menu)
        self.current_menu = {
            'title': "Adjust Value",
            'items': [{
                'text': f"Current: {self.variable_temp}",
                'confirm': lambda var=var_path: exec(f"{var} = {self.variable_temp}"),
                'step': item['adjust']['step']
            }]
        }
        self.selected_index = 0
        self.adjust_mode = True  # 进入调整模式
        lcd.clear(0x0000)
    
    def update_adjust_text(self):
        self.current_menu['items'][0]['text'] = f"Current: {self.variable_temp:.2f}"

# 初始化菜单系统
menu = MenuSystem()

# def menu(key_data):
#     menu_system.handle_input(key_data)
#     menu_system.render()
#     gc.collect()