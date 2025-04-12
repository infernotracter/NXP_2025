import serial
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

# 配置串口参数
SERIAL_PORT = 'COM7'      # 根据实际情况修改
BAUD_RATE = 115200

# 初始化立方体顶点（边长为2，中心在原点）
vertices = np.array([[-1, -1, -1],
                     [1, -1, -1],
                     [1, 1, -1],
                     [-1, 1, -1],
                     [-1, -1, 1],
                     [1, -1, 1],
                     [1, 1, 1],
                     [-1, 1, 1]])

# 定义立方体的面（顶点索引）
faces = [
    [0, 1, 2, 3],  # 底面
    [4, 5, 6, 7],  # 顶面
    [0, 1, 5, 4],  # 前面
    [2, 3, 7, 6],  # 后面
    [0, 3, 7, 4],  # 左面
    [1, 2, 6, 5],  # 右面
]

# 创建3D图表
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-2, 2)
ax.set_title('Euler Angles Visualization')
ax.view_init(elev=20, azim=30)  # 初始视角

# 初始化串口连接
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

def update(frame):
    # 默认欧拉角值
    roll, pitch, yaw = 0, 0, 0
    
    # 读取串口数据
    while ser.in_waiting:
        try:
            line = ser.readline().decode().strip()
            if line:
                values = list(map(float, line.split(',')))
                if len(values) == 3:
                    roll, pitch, yaw = values
        except Exception as e:
            print(f"Error processing data: {e}")

    # 清除之前的图形
    ax.cla()
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)
    ax.set_title(f'Euler Angles: Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°')
    
    # 计算旋转矩阵（XYZ顺序）
    rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    rotated_vertices = rotation.apply(vertices)
    
    # 绘制立方体线框
    for face in faces:
        # 闭合多边形需要重复第一个点
        points = face + [face[0]]
        x = rotated_vertices[points, 0]
        y = rotated_vertices[points, 1]
        z = rotated_vertices[points, 2]
        ax.plot(x, y, z, color='blue', linewidth=1)
    
    # 绘制坐标轴
    axis_length = 2.5
    ax.quiver(0, 0, 0, axis_length, 0, 0, color='r', arrow_length_ratio=0.1)
    ax.quiver(0, 0, 0, 0, axis_length, 0, color='g', arrow_length_ratio=0.1)
    ax.quiver(0, 0, 0, 0, 0, axis_length, color='b', arrow_length_ratio=0.1)

    return []

# 启动动画（blit=False 保证稳定运行）
ani = FuncAnimation(fig, update, interval=50, blit=False)
plt.show()

# 关闭串口连接
ser.close()