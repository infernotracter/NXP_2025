class PIDController:
    def __init__(self, kp=0, ki=0, kd=0, limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.limits = limits

    def calculate(self, target, current):
        error = target - current
        self.integral += error * self.ki
        derivative = error - self.prev_error
        
        output = (self.kp * error) + self.integral + (self.kd * derivative)
        
        if self.limits:
            output = max(min(output, self.limits[1]), self.limits[0])
            
        self.prev_error = error
        return output

class PIDManager:
    def __init__(self):
        self.speed_pid = PIDController(kp=0.8, ki=0.1)
        self.angle_pid = PIDController(kp=0.3)
        self.gyro_pid = PIDController(kp=500.0, kd=20.0)