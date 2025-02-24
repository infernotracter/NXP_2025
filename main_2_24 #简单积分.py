from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 从 seekfree 库包含 IMU963RA
from seekfree import IMU963RA

# 包含 gc 类
import gc
import math
# 开发板上的 C19 是拨码开关
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
end_state = end_switch.value()

# 调用 IMU963RA 模块获取 IMU963RA 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 可以不填 默认参数为 1 调整这个参数相当于调整采集分频
imu = IMU963RA()

# 单位换算用
ACC_SPL = 4096.0
GYRO_SPL = 16.4

ticker_flag = False
ticker_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)
# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
pit1.capture_list(imu)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(1)



zero_calc_count=1000
imu.count=0
imu.zero_calc_flag = True
def gyro_get_angle():
   imu.gyro.angleZ -=imu_data[5] / GYRO_SPL *0.005
   imu.gyro.angleX +=imu_data[3] / GYRO_SPL *0.005
   imu.gyro.angleY +=imu_data[4] / GYRO_SPL *0.005

   if(imu.zero_calc_flag):
      if(imu.count < zero_calc_count):
         imu.gyro.zero_angleZ += imu.gyro.angleZ -imu.gyro.last_angleZ
         imu.gyro.last_angleZ=imu.gyro.angleZ


         imu.gyro.zero_angleX +=imu.gyro.angleX - imu.gyro.last_angleX
         imu.gyro.last_angleX=imu.gyro.angleX


         imu.gyro.zero_angleY += imu.gyro.angleY-imu.gyro.last_angleY
         imu.gyro.last_angleY = imu.gyro.angleY

         imu.count+=1

      else:
         imu.gyro.angleZ -= imu.gyro.zero_angleZ
         imu.gyro.zero_angleZ /= zero_calc_count

         imu.gyro.angleY -= imu.gyro.zero_angleY
         imu.gyro.zero_angleY /= zero_calc_count

         imu.gyro.angleX -= imu.gyro.zero_angleX
         imu.gyro.zero_angleX /= zero_calc_count

         imu.zero_calc_flag = False
         imu.count = 0
   else:
     imu.gyro.angleZ -= imu.gyro.zero_angleZ
     imu.gyro.angleX -= imu.gyro.zero_angleX
     imu.gyro.angleY -= imu.gyro.zero_angleY
     if(imu.gyro.angleZ >360):
        imu.gyro.angleZ -= 360
     elif(imu.gyro.angleZ < 0):
        imu.gyro.angleZ += 360
   return imu.gyro.angleX,imu.gyro.angleY,imu.gyro.angleZ

def acc_get_angle():
   acc_x=imu_data[0] / ACC_SPL
   acc_y=imu_data[1] / ACC_SPL
   acc_z=imu_data[2] / ACC_SPL


   imu.acc.angleX = math.degrees(atan(acc_y/sqrt(acc_x**2+acc_z**2)))
   imu.acc.angleY = math.degrees(atan(-acc_x/sqrt(acc_y**2+acc_z**2)))

   return imu.acc.angleX,imu.acc.angleY,imu.acc.angleZ


while True:
    if (ticker_flag and ticker_count % 1 == 0):
        imu_data = imu.get()
        gyro_get_angle()
        acc_get_angle()
        print(f"{imu.gyro.angleX},{imu.gyro.angleY},{imu.gyro.angleZ},{imu.acc.angleX},{imu.acc.angleY},{imu.acc.angleZ}")
        ticker_flag = False
    if end_switch.value() != end_state:
        pit1.stop()
        print("Ticker stop.")
        break
    gc.collect()