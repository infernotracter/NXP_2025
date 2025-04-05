
def my_limit(value, min_val, max_val):
    return max(min_val, min(value, max_val))

class PID:
    def __init__(self, kp=0, ki=0, kd=0,
                 integral_limits=None, output_limits=None,
                 output_adjustment=None):
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

        # 积分项处理
        self.integral += error * self.ki
        if self.integral_limits:
            self.integral = my_limit(self.integral, *self.integral_limits)

        # 微分项计算
        derivative = error - self.prev_error

        # PID输出计算
        output = (self.kp * error) + self.integral + (self.kd * derivative)

        # 输出限幅
        if self.output_limits:
            output = my_limit(output, *self.output_limits)

        # 特殊输出调整
        if self.output_adjustment:
            output = self.output_adjustment(output)

        self.prev_error = error
        return output

# 特殊输出调整函数
def gyro_adjustment(output):
    return output + 700 if output >= 0 else output - 700

# PID实例化
speed_pid = PID(kp=0.8, ki=0.1, integral_limits=(-2000, 2000))
# output_limits=(-500, 500))

angle_pid = PID(kp=0.3, kd=0.0)
# , integral_limits=(-2000, 2000))

gyro_pid = PID(kp=500.0, kd=20.0,  # kp=
               #    , integral_limits=(-2000, 2000),
               # output_limits=(-500, 500),
               output_adjustment=gyro_adjustment)

dir_in = PID(kp=0.0, ki=0.0)
#  integral_limits=(-2000, 2000))

dir_out = PID(kp=0.0, kd=0.0)

class PID_data:
    def __init__(self):
        self.speed_pid_out = 0
        self.angle_pid_out = 0
        self.gyro_pid_out = 0
        self.dir_in_out = 0
        self.dir_out_out = 0

        self.out_l = 0
        self.out_r = 0
    def pid_out(self):
        self.out_l = self.gyro_pid_out + self.dir_in_out
        self.out_r = self.gyro_pid_out - self.dir_in_out
        self.out_l = my_limit(self.out_l, -5000, 5000)
        self.out_r = my_limit(self.out_r, -5000, 5000)

pid = PID_data()