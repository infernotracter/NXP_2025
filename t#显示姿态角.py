import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import math

# 配置参数
PLOT_LIMIT = 2  # 坐标系显示范围
FRAME_INTERVAL = 50  # 刷新间隔(ms)

class IMUVisualizer:
    def __init__(self):
        # 初始化3D图形
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim(-PLOT_LIMIT, PLOT_LIMIT)
        self.ax.set_ylim(-PLOT_LIMIT, PLOT_LIMIT)
        self.ax.set_zlim(-PLOT_LIMIT, PLOT_LIMIT)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.view_init(elev=20, azim=45)
        
        # 初始化立方体模型
        self.cube = self.create_cube()
        self.quiver = None  # 用于坐标轴箭头
        self.texts = []     # 用于显示文本
        
        # 初始化欧拉角
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def create_cube(self):
        """创建表示IMU的立方体模型"""
        # 定义立方体顶点 (边长1.5)
        vertices = np.array([[-0.75, -0.75, -0.75],
                             [0.75, -0.75, -0.75],
                             [0.75, 0.75, -0.75],
                             [-0.75, 0.75, -0.75],
                             [-0.75, -0.75, 0.75],
                             [0.75, -0.75, 0.75],
                             [0.75, 0.75, 0.75],
                             [-0.75, 0.75, 0.75]])
        
        # 定义立方体面
        faces = [[0,1,2,3], [4,5,6,7], [0,1,5,4],
                 [2,3,7,6], [0,3,7,4], [1,2,6,5]]
        
        # 绘制初始立方体
        cube = self.ax.add_collection3d(
            plt.Poly3DCollection([vertices[face] for face in faces],
                                facecolors='cyan', linewidths=1, 
                                edgecolors='darkblue', alpha=0.8))
        return cube

    def update_attitude(self, roll, pitch, yaw):
        """更新姿态角（单位：弧度）"""
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def rotation_matrix(self):
        """生成旋转矩阵 (Z-Y-X顺序)"""
        # 绕Z轴旋转 (yaw)
        Rz = np.array([[math.cos(self.yaw), -math.sin(self.yaw), 0],
                      [math.sin(self.yaw), math.cos(self.yaw), 0],
                      [0, 0, 1]])
        
        # 绕Y轴旋转 (pitch)
        Ry = np.array([[math.cos(self.pitch), 0, math.sin(self.pitch)],
                      [0, 1, 0],
                      [-math.sin(self.pitch), 0, math.cos(self.pitch)]])
        
        # 绕X轴旋转 (roll)
        Rx = np.array([[1, 0, 0],
                      [0, math.cos(self.roll), -math.sin(self.roll)],
                      [0, math.sin(self.roll), math.cos(self.roll)]])
        
        return Rz @ Ry @ Rx

    def update_frame(self, frame):
        """动画更新函数"""
        # 清除旧箭头和文本
        if self.quiver is not None:
            self.quiver.remove()
        for text in self.texts:
            text.remove()
        self.texts.clear()
        
        # 计算旋转矩阵
        R = self.rotation_matrix()
        
        # 更新立方体顶点
        vertices = self.cube.get_verts()
        rotated_verts = np.dot(vertices, R.T)  # 应用旋转
        self.cube.set_verts(rotated_verts)
        
        # 添加坐标轴箭头
        arrow_length = 1.5
        arrow_colors = ['r', 'g', 'b']
        for i in range(3):
            self.quiver = self.ax.quiver(
                0, 0, 0, 
                R[0,i]*arrow_length, R[1,i]*arrow_length, R[2,i]*arrow_length,
                color=arrow_colors[i], lw=2)
        
        # 添加姿态角文本显示
        text_template = 'Roll: {:.1f}°\nPitch: {:.1f}°\nYaw: {:.1f}°'
        text_str = text_template.format(
            math.degrees(self.roll), 
            math.degrees(self.pitch),
            math.degrees(self.yaw))
        
        self.texts.append(self.ax.text2D(
            0.05, 0.95, text_str, transform=self.ax.transAxes,
            fontsize=12, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)))
        
        return self.cube, self.quiver, *self.texts

# ================== 使用示例 ==================
if __name__ == "__main__":
    vis = IMUVisualizer()
    
    # 模拟姿态变化函数（替换为实际IMU数据输入）
    def simulate_euler_angles(frame):
        t = frame * 0.1
        roll = math.radians(30 * math.sin(t))
        pitch = math.radians(45 * math.sin(0.8*t))
        yaw = math.radians(60 * t)
        vis.update_attitude(roll, pitch, yaw)
    
    # 创建动画
    ani = FuncAnimation(vis.fig, vis.update_frame,
                        init_func=lambda: vis.update_frame(0),
                        interval=FRAME_INTERVAL, blit=True)
    
    plt.show()