# 全局变量存储区（模拟参数）
global_vars = {
    'aim_speed_l': 0,
    'aim_speed_r': 0,
    'angle_kp': 1.0,
    'angle_kd': 0.5,
    'speed_kp': 0.8,
    'speed_ki': 0.1,
    'gyro_kp': 0.7,
    'gyro_kd': 0.3,
}

# 函数定义区
def car_go_function(speed):
    global global_vars
    global_vars['aim_speed_l'] = speed
    global_vars['aim_speed_r'] = speed
    print(f"Car started at speed: {speed}")

def show_ccd_image():
    print("Displaying CCD image")
    # 这里添加实际的CCD显示逻辑

def show_parameter_monitor():
    print("Showing parameter monitor")
    # 这里添加参数监控逻辑

# 菜单配置
menu_config = [
    # 参数显示/修改模式
    {
        'name': 'aim_speed_l',
        'type': 'parameter',
        'param': 'aim_speed_l',
        'min': 0,
        'max': 200,
        'step': 10,
        'format': '{}'
    },
    {
        'name': 'aim_speed_r',
        'type': 'parameter',
        'param': 'aim_speed_r',
        'min': 0,
        'max': 200,
        'step': 10,
        'format': '{}'
    },
    {
        'name': 'angle_pid',
        'type': 'parameter',
        'param': 'angle_kp',
        'sub_params': ['angle_kp', 'angle_kd'],
        'steps': [0.01, 0.1],
        'format': '{:.2f}'
    },
    {
        'name': 'speed_pi',
        'type': 'parameter',
        'param': 'speed_kp',
        'sub_params': ['speed_kp', 'speed_ki'],
        'steps': [0.01, 0.1],
        'format': '{:.2f}'
    },
    {
        'name': 'gyro_pi',
        'type': 'parameter',
        'param': 'gyro_kp',
        'sub_params': ['gyro_kp', 'gyro_kd'],
        'steps': [0.01, 0.1],
        'format': '{:.2f}'
    },
    
    # 函数执行模式
    {
        'name': 'car_go',
        'type': 'function',
        'func': lambda: car_go_function(100)
    },
    {
        'name': 'ccd_image',
        'type': 'function',
        'func': show_ccd_image
    },
    {
        'name': 'parameter_monitor',
        'type': 'function',
        'func': show_parameter_monitor
    }
]

# 菜单状态变量
current_index = 0
editing_mode = False
param_changed = False
menu_active = True
current_param = None
sub_index = 0


# 菜单处理函数
def menu(key_data):
    global current_index, editing_mode, param_changed, menu_active
    global current_param, sub_index
    
    if not menu_active: 
        return
    
    # 清屏
    lcd.clear(0x0000)
    
    # 显示菜单标题
    title = "PARAM EDIT" if editing_mode else "MAIN MENU"
    lcd.str24(60, 0, title, 0x07E0)
    
    # 遍历菜单项并显示
    for i, item in enumerate(menu_config):
        y_pos = 30 + i * 16
        
        # 当前选中项标记
        cursor = ">" if i == current_index else " "
        lcd.str16(0, y_pos, cursor, 0xF800 if i == current_index else 0xFFFF)
        
        if item['type'] == 'parameter':
            # 显示参数项
            value = global_vars[item['param']]
            fmt = item.get('format', '{}')
            
            if editing_mode and i == current_index:
                # 编辑模式下的特殊显示
                name = f"*{item['name']}*"
            else:
                name = item['name']
                
            text = f"{name}: {fmt.format(value)}"
            lcd.str16(16, y_pos, text, 0xFFFF)
            
        elif item['type'] == 'function':
            # 显示函数项
            lcd.str16(16, y_pos, item['name'], 0xFFFF)
    
    # 按键处理
    if editing_mode:
        handle_edit_mode_keys(key_data)
    else:
        handle_normal_mode_keys(key_data)

def handle_normal_mode_keys(key_data):
    global current_index, editing_mode, menu_config, current_param
    
    # 上下键：切换选中项
    if key_data[0]:  # 下键
        current_index = (current_index + 1) % len(menu_config)
        key.clear(0)
        
    if key_data[1]:  # 上键
        current_index = (current_index - 1) % len(menu_config)
        key.clear(1)
        
    # 确认键
    if key_data[2]:
        item = menu_config[current_index]
        if item['type'] == 'function':
            # 执行函数
            item['func']()
        elif item['type'] == 'parameter':
            # 进入参数编辑模式
            editing_mode = True
            current_param = item
        key.clear(2)

def handle_edit_mode_keys(key_data):
    global current_index, editing_mode, param_changed
    global global_vars, current_param, sub_index
    
    # 获取当前参数项
    item = current_param
    
    # 编辑模式按键处理
    if key_data[0]:  # 下键 - 减小值
        if item is not None and 'sub_params' in item:
            # 分组参数：调整当前选中的子参数
            param = item['sub_params'][sub_index]
            step = item['steps'][sub_index]
            global_vars[param] = max(global_vars[param] - step, 0)
        else:
            # 单参数：直接调整
            step = item.get('step', 1) if item is not None else 1
            if item is not None:
                global_vars[item['param']] = max(global_vars[item['param']] - step, item.get('min', 0))
        param_changed = True
        key.clear(0)
        
    if key_data[1]:  # 上键 - 增加值
        if item is not None and 'sub_params' in item:
            param = item['sub_params'][sub_index]
            step = item['steps'][sub_index]
            global_vars[param] += step
        else:
            step = item.get('step', 1) if item is not None else 1
            if item is not None:
                global_vars[item['param']] = min(
                    global_vars[item['param']] + step, 
                    item.get('max', float('inf'))
                )
        param_changed = True
        key.clear(1)
        
    # 切换子参数/退出编辑
    if key_data[2]:  # 确认键
        if item is not None and 'sub_params' in item:
            # 分组参数：切换子参数
            sub_index = (sub_index + 1) % len(item['sub_params'])
        else:
            # 单参数：退出编辑
            editing_mode = False
            current_param = None
        key.clear(2)
        
    if key_data[3]:  # 返回键
        editing_mode = False
        current_param = None
        key.clear(3)

# 示例使用
while True:
    key_data = key.get_data()  # 获取按键数据
    menu(key_data)  # 处理菜单
    time.sleep_ms(20)  # 控制刷新率