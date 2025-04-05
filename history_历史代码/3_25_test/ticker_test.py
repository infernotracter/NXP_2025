from smartcar import ticker
from handware import *

# 全局定时标志字典
ticker_flags = {
    '1ms': False,
    '2ms': False,
    '4ms': False,
    '5ms': False,
    '8ms': False,
    '10ms': False,
    '50ms': False,
}

# 定时器配置列表
ticker_configs = [
    {
        'pit_id': 0,
        'interval': 1,
        'captures': [ccd, imu, key, encoder_l, encoder_r],
        'tasks': [
            {'current': 0, 'target': 5, 'flag': '2ms'},
            {'current': 0, 'target': 25, 'flag': '10ms'},
            {'current': 0, 'target': 125, 'flag': '50ms'},
        ]
    },
    {
        'pit_id': 1,
        'interval': 1,
        'captures': [imu, key],
        'tasks': [
            {'current': 0, 'target': 1, 'flag': '1ms'},
        ]
    },
    {
        'pit_id': 2,
        'interval': 5,
        'captures': [ccd, key, encoder_l, encoder_r],
        'tasks': [
            {'current': 0, 'target': 1, 'flag': '5ms'},
        ]
    },
    {
        'pit_id': 3,
        'interval': 1,
        'captures': [ccd, imu],
        'tasks': [
            {'current': 0, 'target': 4, 'flag': '4ms'},
            {'current': 0, 'target': 8, 'flag': '8ms'},
        ]
    },
]

def create_handler(tasks):
    """生成通用回调处理器"""
    def handler(time):
        for task in tasks:
            task['current'] += 1
            if task['current'] >= task['target']:
                ticker_flags[task['flag']] = True
                task['current'] = 0
    return handler

def check_flag(flag_name):
    """仅检查标志状态不重置"""
    return ticker_flags[flag_name]

# 初始化所有定时器
for config in ticker_configs:
    pit = ticker(config['pit_id'])
    pit.capture_list(*config['captures'])
    pit.callback(create_handler(config['tasks']))
    pit.start(config['interval'])