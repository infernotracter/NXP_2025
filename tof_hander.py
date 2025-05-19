from basic_data import *
tof = DL1B()
class Tof_hander:
    def __init__(self, barrier_threshold, ramp_threshold):
        self.data= 0
        self.barrier_threshold = barrier_threshold
        self.ramp_threshold = ramp_threshold
        self.last_data = 0
        self.ramp_count = 0  # 用于坡道检测的计数器

    def update(self):
        self.last_data = self.data
        self.data = tof.get()
        if(abs(self.data - self.last_data) >= self.barrier_threshold):
            self.last_data = self.data
            return 1
        elif(self.barrier_threshold>=abs(self.data-self.last_data) >= self.ramp_threshold):
            self.last_data = self.data
            self.ramp_count += 1
            if self.ramp_count >= 3:  # 连续检测到 3 次变化才判定为坡道
                self.ramp_count = 0
                return -1
        
        self.last_data = self.data
        
tof_hander = Tof_hander(100, 10)       