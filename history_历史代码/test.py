# 基础库、NXP库、第三方库
from machine import *
from display import *
from smartcar import *
from seekfree import *
from math import *
import gc
import time
import math
import os
import io

# 常量定义
ACC_SPL = 4096.0
GYRO_SPL = 16.4
OFFSETNUM = 100
MED_ANGLE_INIT = 64.0
SPEED_INCREMENT = 50
LCD_COLOR_BG = 0x0000
LCD_COLOR_TEXT = 0xFFFF
LCD_COLOR_HIGHLIGHT = 0xF800

class CarController:
    def __init__(self):
        # 硬件初始化
        self._init_hardware()
        # 控制变量初始化
        self._init_control_vars()
        # PID控制器初始化
        self._init_pid_controllers()
        # 定时器初始化
        self._init_timers()
        # 菜单系统初始化
        self._init_menu_system()
        
    def _init_hardware(self):
        """初始化所有硬件组件"""
        # 无线通信模块
        self.wireless = WIRELESS_UART(460800)
        # 显示屏组件
        self._init_display()
        # 电机和编码器
        self._init_motors()
        self._init_encoders()
        # IMU传感器
        self.imu = IMU963RA()
        # 输入设备
        self._init_inputs()
        # CCD摄像头
        self.ccd = TSL1401(3)

    def _init_display(self):
        """初始化显示屏相关组件"""
        cs = Pin('C5', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        rst = Pin('B9', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        dc = Pin('B8', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        blk = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        drv = LCD_Drv(SPI_INDEX=1, BAUDRATE=60000000, DC_PIN=dc,
                      RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
        self.lcd = LCD(drv)
        self.lcd.color(LCD_COLOR_TEXT, LCD_COLOR_BG)
        self.lcd.mode(2)
        self.lcd.clear(LCD_COLOR_BG)

    def _init_motors(self):
        """初始化电机控制"""
        self.motor_l = MOTOR_CONTROLLER(
            MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty=0, invert=True)
        self.motor_r = MOTOR_CONTROLLER(
            MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty=0, invert=True)

    def _init_encoders(self):
        """初始化编码器"""
        self.encoder_l = encoder("D0", "D1", True)
        self.encoder_r = encoder("D2", "D3")

    def _init_inputs(self):
        """初始化输入设备"""
        self.go_button = Pin('C21', Pin.IN, pull=Pin.PULL_UP_47K, value=0)
        self.end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
        self.switch_3 = Pin('B14', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
        self.switch_4 = Pin('B15', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
        self.led1 = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
        self.key = KEY_HANDLER(10)

    def _init_control_vars(self):
        """初始化控制变量"""
        # 传感器数据
        self.ccd_data1 = [0] * 128
        self.ccd_data2 = [0] * 128
        self.encl_data = 0
        self.encr_data = 0
        
        # 控制目标
        self.aim_speed = 0
        self.aim_speed_l = 0
        self.aim_speed_r = 0
        self.MedAngle = MED_ANGLE_INIT
        
        # IMU相关
        self.current_pitch = 0
        self.current_roll = 0
        self.current_yaw = 0
        self._init_imu_offsets()

    def _init_imu_offsets(self):
        """初始化IMU零偏"""
        self.gyrooffsetx = self.gyrooffsety = self.gyrooffsetz = 0
        self.accoffsetx = self.accoffsety = self.accoffsetz = 0
        self._calibrate_imu()

    def _calibrate_imu(self):
        """执行IMU校准"""
        acc_temp = [0.0]*3
        gyro_temp = [0.0]*3
        for _ in range(OFFSETNUM):
            imu_data = [float(x) for x in self.imu.get()]
            acc_temp = [a + b for a, b in zip(acc_temp, imu_data[:3])]
            gyro_temp = [g + b for g, b in zip(gyro_temp, imu_data[3:6])]
        self.accoffsetx, self.accoffsety, self.accoffsetz = [x/OFFSETNUM for x in acc_temp]
        self.gyrooffsetx, self.gyrooffsety, self.gyrooffsetz = [x/OFFSETNUM for x in gyro_temp]

    def _init_pid_controllers(self):
        """初始化PID控制器"""
        # 速度环PID
        self.speed_pid = PID(
            kp=0.8, ki=0.1,
            integral_limits=(-2000, 2000)
        )
        # 角度环PID
        self.angle_pid = PID(kp=0.3, kd=0.0)
        # 陀螺仪PID
        self.gyro_pid = PID(
            kp=500.0, kd=20.0,
            output_adjustment=lambda x: x + 700 if x >=0 else x -700
        )
        # 转向PID
        self.dir_in_pid = PID(kp=0.0, ki=0.0)
        self.dir_out_pid = PID(kp=0.0, kd=0.0)

    def _init_timers(self):
        """初始化定时器和中断处理"""
        self.ticker_flags = {
            '1ms': False, '2ms': False, '4ms': False,
            '5ms': False, '8ms': False, '10ms': False, '50ms': False
        }
        
        # 主PID定时器
        self.pit0 = ticker(0)
        self.pit0.capture_list(self.ccd, self.imu, self.key, 
                              self.encoder_l, self.encoder_r)
        self.pit0.callback(self._time_pit_pid_handler)
        self.pit0.start(1)
        
        # 1ms定时器
        self.pit1 = ticker(1)
        self.pit1.capture_list(self.imu, self.key)
        self.pit1.callback(self._time_pit_1ms_handler)
        self.pit1.start(1)
        
        # 其他定时器初始化类似...

    def _time_pit_pid_handler(self, time):
        """主PID定时器中断处理"""
        global pit_cont_gyro, pit_cont_angle, pit_cont_speed
        pit_cont_gyro += 1
        pit_cont_angle += 1
        pit_cont_speed += 1
        
        self.ticker_flags['2ms'] = (pit_cont_gyro % 5 == 0)
        self.ticker_flags['10ms'] = (pit_cont_angle % 25 == 0)
        self.ticker_flags['50ms'] = (pit_cont_speed % 125 == 0)

    def _time_pit_1ms_handler(self, time):
        """1ms定时器中断处理"""
        self.ticker_flags['1ms'] = True

    # 其他中断处理函数...

    def _init_menu_system(self):
        """初始化菜单系统"""
        self.menu_state = {
            'main': True,
            'car_go': False,
            'speed': False,
            # 其他菜单状态...
        }
        self.menu_pointer = 30  # 初始菜单指针位置

    def run(self):
        """主运行循环"""
        while True:
            self._update_sensors()
            self._update_pid()
            self._update_motors()
            self._handle_menu()
            
            if self.end_switch.value():
                self._emergency_stop()
                break

    def _update_sensors(self):
        """更新传感器数据"""
        # 编码器数据
        if self.ticker_flags['5ms']:
            self.encl_data = self.encoder_l.get()
            self.encr_data = self.encoder_r.get()
            self.ticker_flags['5ms'] = False
            
        # IMU数据处理
        if self.ticker_flags['1ms']:
            self._process_imu_data()
            self.ticker_flags['1ms'] = False

    def _process_imu_data(self):
        """处理IMU数据并进行姿态解算"""
        imu_data = [float(x) for x in self.imu.get()]
        # 零偏校正
        imu_data[:3] = [(v - offset)/ACC_SPL for v, offset in zip(imu_data[:3], 
            [self.accoffsetx, self.accoffsety, self.accoffsetz])]
        imu_data[3:] = [(math.radians((v - offset)/GYRO_SPL)) for v, offset in zip(imu_data[3:], 
            [self.gyrooffsetx, self.gyrooffsety, self.gyrooffsetz])]
        
        # 四元数更新
        self._quaternion_update(*imu_data)

    def _quaternion_update(self, ax, ay, az, gx, gy, gz):
        """四元数姿态解算"""
        # 实现略，保持原有算法
        pass

    def _update_pid(self):
        """更新所有PID控制器"""
        if self.ticker_flags['2ms']:
            self.gyro_pid_out = self.gyro_pid.calculate(
                self.angle_pid_out, self.current_roll)
            self.ticker_flags['2ms'] = False
            
        if self.ticker_flags['10ms']:
            self.angle_pid_out = self.angle_pid.calculate(
                self.speed_pid_out + self.MedAngle, self.current_roll)
            self.ticker_flags['10ms'] = False
            
        if self.ticker_flags['50ms']:
            self.speed_pid_out = self.speed_pid.calculate(
                self.aim_speed, (self.encl_data + self.encr_data) / 2)
            self.ticker_flags['50ms'] = False

    def _update_motors(self):
        """更新电机输出"""
        motor_l_duty = my_limit(self.gyro_pid_out - self.dir_in_out, -3000, 3000)
        motor_r_duty = my_limit(self.gyro_pid_out + self.dir_in_out, -3000, 3000)
        self.motor_l.duty(motor_l_duty)
        self.motor_r.duty(motor_r_duty)

    def _handle_menu(self):
        """处理菜单系统"""
        key_data = self.key.get()
        if self.menu_state['main']:
            self._draw_main_menu(key_data)
        # 其他菜单状态处理...

    def _draw_main_menu(self, key_data):
        """绘制主菜单"""
        self.lcd.clear(LCD_COLOR_BG)
        self.lcd.str24(60, 0, "Main Menu", 0x07E0)
        menu_items = [
            ("Car Go", 30), ("Speed", 46), ("Element", 62),
            ("Angle PD", 78), ("Speed PI", 94), ("Gyro PI", 110),
            ("CCD Image", 126), ("Parameters", 142),
            ("Screen Off", 158), ("Save Config", 174)
        ]
        
        for text, y in menu_items:
            color = LCD_COLOR_HIGHLIGHT if y == self.menu_pointer else LCD_COLOR_TEXT
            self.lcd.str16(16, y, text, color)
        
        self._handle_menu_navigation(key_data)

    def _handle_menu_navigation(self, key_data):
        """处理菜单导航"""
        if key_data[0]:  # 下键
            self.menu_pointer = min(self.menu_pointer + 16, 174)
        if key_data[1]:  # 上键
            self.menu_pointer = max(self.menu_pointer - 16, 30)
        if key_data[2]:  # 确认键
            self._select_menu_item()

    def _select_menu_item(self):
        """处理菜单项选择"""
        if self.menu_pointer == 30:
            self.menu_state.update(main=False, car_go=True)
        elif self.menu_pointer == 46:
            self.menu_state.update(main=False, speed=True)
        # 其他菜单项处理...

    def _emergency_stop(self):
        """紧急停止处理"""
        self.pit1.stop()
        self.pit2.stop()
        self.pit3.stop()
        self.motor_l.duty(0)
        self.motor_r.duty(0)

def my_limit(value, min_val, max_val):
    """通用限幅函数"""
    return max(min_val, min(value, max_val))

class PID:
    def __init__(self, kp=0, ki=0, kd=0, integral_limits=None, output_limits=None, output_adjustment=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.integral_limits = integral_limits
        self.output_limits = output_limits
        self.output_adjustment = output_adjustment

    def calculate(self, target, current):
        error = target - current
        self.integral += error * self.ki
        
        # 积分限幅
        if self.integral_limits:
            self.integral = my_limit(self.integral, *self.integral_limits)
        
        # 微分项计算
        derivative = error - self.prev_error
        
        # PID输出
        output = self.kp * error + self.integral + self.kd * derivative
        
        # 输出限幅
        if self.output_limits:
            output = my_limit(output, *self.output_limits)
        
        # 输出调整
        if self.output_adjustment:
            output = self.output_adjustment(output)
        
        self.prev_error = error
        return output

if __name__ == "__main__":
    car = CarController()
    car.run()