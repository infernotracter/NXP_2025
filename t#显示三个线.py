import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# 配置串口参数
SERIAL_PORT = 'COM9'  # 根据实际情况修改
BAUD_RATE = 115200
PLOT_RANGE = 10000
# 初始化数据存储
MAX_DATA_POINTS = 5000  # 显示最近50100个数据点
data = {
    'acc_x': deque(maxlen=MAX_DATA_POINTS),
    'acc_y': deque(maxlen=MAX_DATA_POINTS),
    'acc_z': deque(maxlen=MAX_DATA_POINTS),
    'gyro_x': deque(maxlen=MAX_DATA_POINTS),
    'gyro_y': deque(maxlen=MAX_DATA_POINTS),
    'gyro_z': deque(maxlen=MAX_DATA_POINTS),
    'mag_x': deque(maxlen=MAX_DATA_POINTS),
    'mag_y': deque(maxlen=MAX_DATA_POINTS),
    'mag_z': deque(maxlen=MAX_DATA_POINTS),
}

# 创建图表
fig, axs = plt.subplots(3, 1, figsize=(10, 8))
fig.suptitle('IMU963RA Real-time Data Visualization')

# 配置子图
plots = []
titles = ['Accelerometer (m/s²)', 'Gyroscope (deg/s)', 'Magnetometer']
for i, ax in enumerate(axs):
    ax.set_xlim(0, MAX_DATA_POINTS)
    ax.set_ylim(-PLOT_RANGE, PLOT_RANGE)  # 16位有符号整数范围
    ax.set_title(titles[i])
    lines = [
        ax.plot([], [], label='X', color='r')[0],
        ax.plot([], [], label='Y', color='g')[0],
        ax.plot([], [], label='Z', color='b')[0]
    ]
    ax.legend(loc='upper right')
    plots.append(lines)

# 初始化串口连接
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

def update(frame):
    # 读取并处理数据
    while ser.in_waiting:
        try:
            line = ser.readline().decode().strip()
            if line:
                values = list(map(int, line.split(',')))
                if len(values) == 9:
                    data['acc_x'].append(values[0])
                    data['acc_y'].append(values[1])
                    data['acc_z'].append(values[2])
                    data['gyro_x'].append(values[3])
                    data['gyro_y'].append(values[4])
                    data['gyro_z'].append(values[5])
                    data['mag_x'].append(values[6])
                    data['mag_y'].append(values[7])
                    data['mag_z'].append(values[8])
        except Exception as e:
            print(f"Error processing data: {e}")

    # 更新加速度计图表
    x_data = range(len(data['acc_x']))
    plots[0][0].set_data(x_data, data['acc_x'])
    plots[0][1].set_data(x_data, data['acc_y'])
    plots[0][2].set_data(x_data, data['acc_z'])
    
    # 更新陀螺仪图表
    plots[1][0].set_data(x_data, data['gyro_x'])
    plots[1][1].set_data(x_data, data['gyro_y'])
    plots[1][2].set_data(x_data, data['gyro_z'])
    
    # 更新磁力计图表
    plots[2][0].set_data(x_data, data['mag_x'])
    plots[2][1].set_data(x_data, data['mag_y'])
    plots[2][2].set_data(x_data, data['mag_z'])

    # 调整X轴范围
    for ax in axs:
        ax.set_xlim(max(0, len(data['acc_x'])-MAX_DATA_POINTS), len(data['acc_x']))
    
    return [line for sublist in plots for line in sublist]

# 启动动画
ani = FuncAnimation(fig, update, blit=True, interval=50)
plt.tight_layout()
plt.show()

# 关闭串口连接
ser.close()