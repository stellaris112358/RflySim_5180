import cv2
import numpy as np
import math
import sys
import heapq
import time
from scipy.interpolate import splprep, splev
from collections import defaultdict
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from AStar import PathPlanningApp

# 假设的配置类（保持与原代码兼容）
class Config:
    PLOT_CONFIG = {
        'figure_size_2d': (10, 8),
        'path_colors': ['green', 'red', 'blue', 'orange', 'purple', 'cyan'],  # 多路径颜色池
        'path_linewidth': 1,
        'path_alpha': 0.7,
        'path_scatter_size': 10,
        'scatter_alpha': 0.5,
        'start_styles': [
            ('red', 'o'),    # 红色圆形
            ('orange', '^'), # 橙色三角形
            ('magenta', 'D'),# 品红色菱形
            ('brown', 'p'),  # 棕色五边形
        ],
        # 终点样式池（颜色+形状）：与起点样式区分开
        'goal_styles': [
            ('blue', 's'),   # 蓝色方形
            ('green', 'D'),  # 绿色菱形
            ('cyan', '^'),   # 青色三角形
            ('purple', 'p'), # 紫色五边形
        ],
    }

# 假设的可视化工具类（保持原起点终点绘制逻辑）
class Visualizer:
    @staticmethod
    def _plot_start_goal(start_pixel, goal_pixel):
        """绘制起点（红色）和终点（蓝色）"""
        # 起点：红色圆形
        plt.scatter(start_pixel[1], start_pixel[0], c='red', s=100, marker='o', 
                    edgecolors='black', linewidth=2, label='起点')
        # 终点：蓝色方形
        plt.scatter(goal_pixel[1], goal_pixel[0], c='blue', s=100, marker='s', 
                    edgecolors='black', linewidth=2, label='终点')

class PathVisualizer2D:
    def __init__(self, img, start_pixel=None, goal_pixel=None):
        """
        初始化多路径实时可视化器
        :param img: 输入图像（如SLAM的PGM/地图图像）
        :param start_pixel: 起点像素坐标 (行, 列)
        :param goal_pixel: 终点像素坐标 (行, 列)
        """
        # 启用交互式模式
        plt.ion()
        
        # 1. 创建图表和坐标轴（仅创建一次）
        self.fig, self.ax = plt.subplots(figsize=Config.PLOT_CONFIG['figure_size_2d'])
        self.fig.suptitle('2D路径规划实时结果', fontsize=14)
        
        # 2. 显示地图（BGR转RGB，兼容OpenCV读取的图像）
        # if len(img.shape) == 2:  # 灰度图（如PGM）转RGB
        #     img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        # else:  # BGR图（如普通图像）转RGB
        #     img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.ax.imshow(img)
        
        # 3. 绘制起点和终点（仅绘制一次）
        # Visualizer._plot_start_goal(start_pixel, goal_pixel)
        
        # 4. 初始化路径存储：key=路径名称，value=(线对象, 散点对象)
        self.paths = {}
        self.start_and_goal = {}
        self.color_pool = Config.PLOT_CONFIG['path_colors']
        self.start_style = Config.PLOT_CONFIG['start_styles']
        self.goal_styles = Config.PLOT_CONFIG['goal_styles']
        self.next_color_idx = 0  # 下一个路径的颜色索引
        
        # 5. 配置坐标轴和图例
        self.ax.set_xlabel('列坐标 (像素)')
        self.ax.set_ylabel('行坐标 (像素)')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='upper right', framealpha=0.8)
        
        # 6. 刷新初始显示
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def add_or_update_path(self, path_name, path_pixels):
        """
        添加或更新一条路径
        :param path_name: 路径唯一名称（如"路径1"、"A*路径"）
        :param path_pixels: 路径像素坐标列表，每个元素为 (行, 列)
        """
        # 提取路径的x（列）、y（行）坐标
        x_coords = [point[1] for point in path_pixels]
        y_coords = [point[0] for point in path_pixels]

        # 若路径已存在：更新坐标（实时刷新）
        if path_name in self.paths:
            start_obj, goal_obj = self.start_and_goal[path_name]
            line_obj, scatter_obj = self.paths[path_name]
            # 更新起点和终点坐标
            start_obj.set_offsets(np.array([[x_coords[0], y_coords[0]]]))
            goal_obj.set_offsets(np.array([[x_coords[-1], y_coords[-1]]]))
            # 更新线的坐标
            line_obj.set_data(x_coords, y_coords)
            # 更新散点的坐标
            scatter_obj.set_offsets(np.column_stack((x_coords, y_coords)))
        
        # 若路径不存在：创建新路径对象（分配颜色）
        else:
            # 循环使用颜色池
            color = self.color_pool[self.next_color_idx % len(self.color_pool)]
            start_style = self.start_style[self.next_color_idx % len(self.start_style)]
            goal_styles = self.goal_styles[self.next_color_idx % len(self.goal_styles)]
            self.next_color_idx += 1

            # 起点：红色圆形
            start_obj = self.ax.scatter(x_coords[0], y_coords[0], c=start_style[0], s=100, marker=start_style[1], 
                        edgecolors='black', linewidth=2, label=f'起点{path_name}')
            # 终点：蓝色方形
            goal_obj = self.ax.scatter(x_coords[-1], y_coords[-1], c=goal_styles[0], s=100, marker=goal_styles[1], 
                        edgecolors='black', linewidth=2, label=f'终点{path_name}')
            
            # 绘制路径线
            line_obj, = self.ax.plot(
                x_coords, y_coords, 
                color=color, linewidth=Config.PLOT_CONFIG['path_linewidth'],
                alpha=Config.PLOT_CONFIG['path_alpha'],
                label=f"飞行轨迹{path_name}"
            )
            
            # 绘制路径散点（可选，增强可视化）
            scatter_obj = self.ax.scatter(
                x_coords, y_coords, c=color, s=Config.PLOT_CONFIG['path_scatter_size'],
                alpha=Config.PLOT_CONFIG['scatter_alpha']
            )
            
            # 存储路径对象
            self.paths[path_name] = (line_obj, scatter_obj)
            self.start_and_goal[path_name] = (start_obj, goal_obj)
            # 更新图例（避免重复）
            self.ax.legend(loc='upper right', framealpha=0.8)
        
        # 实时刷新图表
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def remove_path(self, path_name):
        """删除一条路径"""
        if path_name in self.paths:
            line_obj, scatter_obj = self.paths.pop(path_name)
            start_obj, goal_obj = self.start_and_goal.pop(path_name)
            # 移除起点和终点
            start_obj.remove()
            goal_obj.remove()
            # 移除绘图对象
            line_obj.remove()
            scatter_obj.remove()
            # 更新图例
            self.ax.legend(loc='upper right', framealpha=0.8)
            # 刷新
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def clear_all_paths(self):
        """清空所有路径（保留地图、起点终点）"""
        for path_name in list(self.paths.keys()):
            self.remove_path(path_name)
        self.next_color_idx = 0  # 重置颜色索引

    def keep_showing(self):
        """阻塞程序，保持图表显示（程序结束前调用）"""
        plt.ioff()  # 关闭交互式模式
        plt.show()

# ==============================
# 程序入口
# ==============================
if __name__ == "__main__":
    num = 3
    flag = False
    total_step = 100
    app = PathPlanningApp()
    start = [[120, 200, 40],
             [20, 20, 20],
             [120, 20, 20]]
    goal = [[80, 2, 40],
            start[0],
            start[0]]
    visualizer = PathVisualizer2D(app.img)
    for i in range(total_step):
        print(i)
        Path = []
        for j in range(num):
            path = app.run(start[j], goal[j])
            visualizer.add_or_update_path(str(j), path)
            if len(path) < 5:
                flag = True
            Path.append(path)
        if flag:
            break
        for j in range(num):
            if j == 0:
                start[j] = list(map(int, Path[j][3]))
                if np.linalg.norm(np.array(start[j])-np.array(goal[j])) < 10:
                    flag = True
            else:
                if np.linalg.norm(np.array(start[j])-np.array(start[0])) < 10:
                    flag = True
                start[j] = list(map(int, Path[j][3]))
                goal[j] = list(map(int, Path[0][0]))

    visualizer.keep_showing()
                
