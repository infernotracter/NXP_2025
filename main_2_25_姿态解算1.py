
# 本示例程序演示如何使用 seekfree 库的 IMU963RA 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板与 IMU963RA 模块测试

# 示例程序运行效果为每 1000ms(1s) 通过 Type-C 的 CDC 虚拟串口输出信息
# 可以通过 C19 的电平状态来控制是否退出测试程序
# 如果看到 Thonny Shell 控制台输出 ValueError: Module init fault. 报错
# 就证明 IMU963RA 模块连接异常 或者模块型号不对 或者模块损坏
# 请检查模块型号是否正确 接线是否正常 线路是否导通 无法解决时请联系技术支持

# IMU963RA 的更新周期计算方式
# IMU963RA 通过 IMU963RA(x) 初始化构建对象时 传入的 x 代表需要进行几次触发才会更新一次数据
# Ticker 通过 start(y) 启动时 y 代表 Ticker 的周期
# 此时每 y 毫秒会触发一次 IMU963RA 的更新
# 当触发次数大于等于 x 时 IMU963RA 才会更新一次数据
# 因此 IMU963RA 的更新周期时间等于 y * x 本例程中就是 10ms * 1 = 10ms

# 从 machine 库包含所有内容
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

# 需要注意的是 ticker 是底层驱动的 这导致 Thonny 的 Stop 命令在这个固件版本中无法停止它
# 因此一旦运行了使用了 ticker 模块的程序 要么通过复位核心板重新连接 Thonny
# 或者像本示例一样 使用一个 IO 控制停止 Ticker 后再使用 Stop/Restart backend 按钮
# V1.1.2 以上版本则可以直接通过 Stop/Restart backend 按钮停止 Ticker


def my_limit(value, minn, maxn):
    if value < minn:
        value = minn
    if value > maxn:
        value = maxn

# 四元数姿态解算相关变量
q0 = 1.0
q1 = q2 = q3 = 0.0
I_ex = I_ey = I_ez = 0.0
imu_kp = 1.5       # 比例增益（调整滤波响应速度）
imu_ki = 0.0005    # 积分增益（调整积分速度）
delta_T = 0.001    # 采样周期（与1ms中断对应）
current_pitch = 0  # 当前俯仰角
current_roll = 0   # 当前横滚角
current_yaw = 0    # 当前偏航角

# 姿态角度计算函数
def quaternion_update(ax, ay, az, gx, gy, gz):
    global q0, q1, q2, q3, I_ex, I_ey, I_ez, current_pitch, current_roll, current_yaw
    
    # 归一化加速度计数据
    norm = math.sqrt(ax**2 + ay**2 + az**2)
    if norm == 0:
        return
    ax /= norm
    ay /= norm
    az /= norm

    # 计算预测的重力方向
    vx = 2 * (q1*q3 - q0*q2)
    vy = 2 * (q0*q1 + q2*q3)
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3

    # 计算误差（叉积）
    ex = (ay * vz - az * vy)
    ey = (az * vx - ax * vz)
    ez = (ax * vy - ay * vx)

    # 误差积分
    I_ex += ex * imu_ki
    I_ey += ey * imu_ki
    I_ez += ez * imu_ki

    # 限幅
    my_limit(I_ex, -100, 100)
    my_limit(I_ey, -100, 100)
    my_limit(I_ez, -100, 100)

    # 调整陀螺仪数据（弧度制）
    gx = math.radians(gx) + imu_kp * ex + I_ex
    gy = math.radians(gy) + imu_kp * ey + I_ey
    gz = math.radians(gz) + imu_kp * ez + I_ez

    # 四元数积分（一阶龙格库塔法）
    half_T = 0.5 * delta_T
    q0_temp = (-q1*gx - q2*gy - q3*gz) * half_T
    q1_temp = ( q0*gx + q2*gz - q3*gy) * half_T
    q2_temp = ( q0*gy - q1*gz + q3*gx) * half_T
    q3_temp = ( q0*gz + q1*gy - q2*gx) * half_T

    # 更新四元数
    q0 += q0_temp
    q1 += q1_temp
    q2 += q2_temp
    q3 += q3_temp

    # 四元数归一化
    norm = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
    q0 /= norm
    q1 /= norm
    q2 /= norm
    q3 /= norm

    # 计算欧拉角
    current_pitch = math.degrees(math.asin(2 * (q0*q2 - q1*q3)))
    current_roll = math.degrees(math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)))
    current_yaw = math.degrees(math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)))

# 零飘定义（初始化为浮点类型）
gyrooffsetx = 0.0
gyrooffsety = 0.0
gyrooffsetz = 0.0
accoffsetx = 0.0
accoffsety = 0.0
accoffsetz = 0.0
OFFSETNUM = 100

def imuoffsetinit():
    global gyrooffsetx, gyrooffsety, gyrooffsetz, accoffsetx, accoffsety, accoffsetz
    # 初始化累加器为浮点数
    acc_temp = [0.0, 0.0, 0.0]
    gyro_temp = [0.0, 0.0, 0.0]
    
    for _ in range(OFFSETNUM):
        data = imu.get()  # 获取原始数据
        data = [float(x) for x in data]  # 强制转换为浮点数
        
        # 累加加速度计数据（索引 0,1,2）
        acc_temp[0] += data[0]
        acc_temp[1] += data[1]
        acc_temp[2] += data[2]
        
        # 累加陀螺仪数据（索引 3,4,5）
        gyro_temp[0] += data[3]
        gyro_temp[1] += data[4]
        gyro_temp[2] += data[5]
    
    # 计算平均值（浮点运算）
    accoffsetx = acc_temp[0] / OFFSETNUM
    accoffsety = acc_temp[1] / OFFSETNUM
    accoffsetz = acc_temp[2] / OFFSETNUM
    gyrooffsetx = gyro_temp[0] / OFFSETNUM
    gyrooffsety = gyro_temp[1] / OFFSETNUM
    gyrooffsetz = gyro_temp[2] / OFFSETNUM

# 初始化偏移量（在主循环前调用）
imuoffsetinit()

while True:
    if (ticker_flag and ticker_count % 1 == 0):
        # 获取数据并强制转换为浮点数列表
        imu_data = [float(x) for x in imu.get()]
        
        # 低通滤波处理（加速度计）
        alpha = 0.2
        imu.accX = (imu_data[0] - accoffsetx / ACC_SPL) * alpha + imu.accX * (1 - alpha)
        imu.accY = (imu_data[1] - accoffsety / ACC_SPL) * alpha + imu.accY * (1 - alpha)
        imu.accZ = (imu_data[2] - accoffsetz / ACC_SPL) * alpha + imu.accZ * (1 - alpha)

        # 陀螺仪单位转换（减去偏移后除以灵敏度）
        imu.gyroX = math.radians((imu_data[3] - gyrooffsetx) / GYRO_SPL)
        imu.gyroY = math.radians((imu_data[4] - gyrooffsety) / GYRO_SPL)
        imu.gyroZ = math.radians((imu_data[5] - gyrooffsetz) / GYRO_SPL)
        # 四元数更新（使用解包后的变量）
        ax, ay, az = imu.accX, imu.accY, imu.accZ
        gx, gy, gz = imu.gyroX, imu.gyroY, imu.gyroZ
        quaternion_update(ax, ay, az, gx, gy, gz)
        
        # 输出姿态角
        print(f"{current_roll},{current_pitch},{current_yaw}")
        ticker_flag = False
    
    if end_switch.value() != end_state:
        pit1.stop()
        print("Ticker stop.")
        break
    gc.collect()
