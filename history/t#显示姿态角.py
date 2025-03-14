import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# 配置串口参数
SERIAL_PORT = 'COM9'      # 根据实际情况修改
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

# 初始化3D图形
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')
ax.view_init(elev=20, azim=30)

# 定义立方体初始顶点（边长为2，中心在原点）
vertices = np.array([[-1, -1, -1],
                     [1, -1, -1],
                     [1, 1, -1],
                     [-1, 1, -1],
                     [-1, -1, 1],
                     [1, -1, 1],
                     [1, 1, 1],
                     [-1, 1, 1]])

# 定义立方体边连接关系
edges = [[0,1], [1,2], [2,3], [3,0],
         [4,5], [5,6], [6,7], [7,4],
         [0,4], [1,5], [2,6], [3,7]]

# 旋转矩阵函数
def rotation_matrix(roll, pitch, yaw):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    
    return Rz @ Ry @ Rx  # 旋转顺序：ZYX（yaw-pitch-roll）

# 初始化绘图元素
lines = [ax.plot([], [], [], 'b-')[0] for _ in edges]
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-2, 2)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 动画更新函数
def update(frame):
    # 读取串口数据
    try:
        data = ser.readline().decode().strip()
        if data:
            angles = list(map(float, data.split(',')))
            if len(angles) == 3:
                roll, pitch, yaw = np.deg2rad(angles)
                
                # 计算旋转后的顶点
                R = rotation_matrix(roll, pitch, yaw)
                rotated_verts = vertices @ R.T
                
                # 更新所有边
                for line, edge in zip(lines, edges):
                    x = rotated_verts[edge, 0]
                    y = rotated_verts[edge, 1]
                    z = rotated_verts[edge, 2]
                    line.set_data(x, y)
                    line.set_3d_properties(z)
    
    except Exception as e:
        print(f"Error: {e}")
    
    return lines

# 创建动画
ani = FuncAnimation(fig, update, interval=50, blit=True)

plt.show()