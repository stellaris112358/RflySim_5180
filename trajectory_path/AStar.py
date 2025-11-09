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

# ==============================
# 配置设置
# ==============================
class Config:
    """配置参数类 - 集中管理所有系统参数"""
    
    # 可视化设置 - 控制图形显示和字体相关参数
    PLOT_CONFIG = {
        'font_family': ['SimHei', 'Microsoft YaHei', 'DejaVu Sans'],  # 字体家族，优先使用中文字体
        'unicode_minus': False,  # 禁用Unicode负号显示，确保负号正常显示
        'figure_size_2d': (12, 8),   # 2D图形窗口尺寸（宽度，高度），单位英寸
        'figure_size_3d': (12, 10)   # 3D图形窗口尺寸（宽度，高度），单位英寸
    }
    
    # 地图处理参数 - 控制地图加载、处理和网格化相关参数
    MAP_CONFIG = {
        'resolution': 0.05,           # 地图分辨率，单位：米/像素，每个像素代表0.05米
        'inflation_radius': 0.1,      # 障碍物膨胀半径，单位：米，为路径规划提供安全边界
        'obstacle_threshold_low': 100,  # 障碍物判断低阈值，像素值低于此值视为障碍物
        'obstacle_threshold_high': 150, # 障碍物判断高阈值，像素值高于此值视为自由空间
        'grid_threshold': 200         # 网格化阈值，灰度值<=200设为障碍物(1)，>200设为可通行(0)
    }
    
    # 路径规划参数 - 控制A*算法和无人机运动约束相关参数
    PLANNER_CONFIG = {
        'heuristic_weight': 0.8,      # 启发式函数权重，平衡精确性和搜索速度
        'diagonal_cost': 1.414,       # 对角线移动代价，近似√2，考虑斜向移动距离
        'grid_cost': 1000,            # 地图的代价
        'max_speed': 2.0,             # 无人机最大飞行速度，单位：米/秒
        'max_acceleration': 0.5,      # 无人机最大加速度，单位：米/秒²
        'max_climb_angle': 30.0,      # 无人机最大爬升角，单位：度，限制飞行坡度
        'max_turn_angle': 10.0,       # 无人机最大转弯角，单位：度，限制路径曲率
        'grid_resolution': 1.0,       # 规划网格分辨率，单位：米，影响搜索精度和速度
        'cruise_speed': 2.0,          # 巡航速度，单位：米/秒
        'turn_speed': 0.8,            # 转弯速度，单位：米/秒
        'takeoff_land_speed': 0.5     # 起飞降落速度，单位：米/秒
    }

# 应用配置 - 将配置参数应用到matplotlib全局设置
plt.rcParams['font.sans-serif'] = Config.PLOT_CONFIG['font_family']  # 设置中文字体
plt.rcParams['axes.unicode_minus'] = Config.PLOT_CONFIG['unicode_minus']  # 设置负号显示


# ==============================
# 地图处理模块
# ==============================
class MapProcessor:
    """地图处理类"""
    
    @staticmethod
    def preprocess_map(image_path):
        """预处理地图"""
        config = Config.MAP_CONFIG
        
        # 读取图像
        img = cv2.imread(image_path)
        if img is None:
            print("无法加载地图图像!")
            return None, None, None
        
        print(f"原始图像尺寸: 高度={img.shape[0]}像素, 宽度={img.shape[1]}像素")
        
        # 图像二值化处理
        img_processed = MapProcessor._binarize_image(img)
        
        # 创建网格地图
        grid = MapProcessor._create_grid(img_processed)
        print(f"网格地图尺寸: 行数={len(grid)}像素, 列数={len(grid[0])}像素")
        
        # 障碍物膨胀
        inflated_grid, gaussian_grid = MapProcessor._inflate_obstacles(grid, config['inflation_radius'], config['resolution'])
        #print(f"膨胀后网格尺寸: 行数={len(inflated_grid)}像素, 列数={len(inflated_grid[0])}像素")
        
        return inflated_grid.tolist(), img, gaussian_grid
    
    @staticmethod
    def _binarize_image(img):
        """图像二值化"""
        config = Config.MAP_CONFIG
        img_processed = img.copy()
        img_processed[img_processed < config['obstacle_threshold_low']] = 0
        img_processed[img_processed > config['obstacle_threshold_high']] = 255
        return img_processed
    
    @staticmethod
    def _create_grid(img_processed):
        """创建网格地图"""
        config = Config.MAP_CONFIG
        grid = cv2.cvtColor(img_processed, cv2.COLOR_BGR2GRAY)
        grid[grid <= config['grid_threshold']] = 1  # 障碍物
        grid[grid > config['grid_threshold']] = 0   # 自由空间
        return grid
    
    @staticmethod
    def _inflate_obstacles(grid, inflation_radius, resolution):
        """障碍物膨胀处理"""
        inflation_pixels = int(inflation_radius / resolution)
        kernel_size = 2 * inflation_pixels + 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        inflated_grid = cv2.dilate(grid, kernel, iterations=1)
        float_grid = grid.astype(np.float32)
        gaussian_grid = cv2.GaussianBlur(float_grid, (12*kernel_size+1, 12*kernel_size+1), sigmaX=3)
        
        return inflated_grid, gaussian_grid


# ==============================
# 坐标转换模块
# ==============================
class CoordinateConverter:
    """坐标转换类"""
    
    @staticmethod
    def pixel_to_world(path_pixels, map_resolution=0.05):
        """像素坐标转世界坐标（绝对坐标）"""
        world_path = []
        for pixel_point in path_pixels:
            world_x = pixel_point[1] * map_resolution  # 列坐标 -> x
            world_y = pixel_point[0] * map_resolution  # 行坐标 -> y  
            world_z = pixel_point[2] * map_resolution  # 高度坐标 -> z
            world_path.append([world_x, world_y, world_z])
        return world_path
    
    # 新增：计算相对起点的坐标
    @staticmethod
    def world_to_relative(world_path):
        """世界坐标（绝对）转相对起点的坐标"""
        if not world_path:
            return []
        # 起点的绝对坐标
        start_x, start_y, start_z = world_path[0]
        # 每个点的相对坐标 = 绝对坐标 - 起点坐标
        relative_path = [
            [x - start_x, y - start_y, z - start_z] 
            for x, y, z in world_path
        ]
        return relative_path


# ==============================
# 轨迹点生成模块
# ==============================
class TrajectoryGenerator:
    """轨迹生成类 - 生成带时间和速度的轨迹"""
    
    @staticmethod
    def generate_trajectory_with_time_and_velocity(world_path):
        """
        简化版：生成带时间和速度的轨迹（不进行中间点插值）
        """
        config = Config.PLANNER_CONFIG
        
        if len(world_path) < 2:
            return []
        
        trajectory = []
        current_time = 0.0
        
        # 添加起点（速度为0）
        start_point = world_path[0]
        trajectory.append([current_time, start_point[0], start_point[1], start_point[2], 0.0, 0.0, 0.0])
        
        total_path_length = 0
        for i in range(len(world_path) - 1):
            current_point = world_path[i]
            next_point = world_path[i + 1]
            
            # 计算段长度和方向
            segment_length, direction_vector = TrajectoryGenerator._calculate_segment_info(current_point, next_point)
            total_path_length += segment_length
            
            # 根据段类型确定速度
            speed = TrajectoryGenerator._determine_segment_speed(current_point, next_point, i, len(world_path))
            
            # 计算段持续时间
            segment_duration = TrajectoryGenerator._calculate_segment_duration(
                segment_length, speed, i, len(world_path)
            )
            
            # 计算速度分量
            velocity_vector = [speed * direction_vector[0], speed * direction_vector[1], speed * direction_vector[2]]
            
            # 直接添加段终点（不进行中间点插值）
            end_time = current_time + segment_duration
            trajectory.append([
                end_time,
                next_point[0], next_point[1], next_point[2],
                velocity_vector[0], velocity_vector[1], velocity_vector[2]
            ])
            
            current_time = end_time
        
        print(f"总路径长度: {total_path_length:.2f}m, 轨迹点数: {len(trajectory)}")
        print(f"平均点间距: {total_path_length/len(trajectory):.3f}m")
        return trajectory

    @staticmethod
    def _calculate_segment_info(current_point, next_point):
        """计算段长度和方向向量"""
        dx = next_point[0] - current_point[0]
        dy = next_point[1] - current_point[1]
        dz = next_point[2] - current_point[2]
        
        segment_length = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if segment_length > 0:
            direction_vector = [dx/segment_length, dy/segment_length, dz/segment_length]
        else:
            direction_vector = [0, 0, 0]
        
        return segment_length, direction_vector

    @staticmethod
    def _calculate_segment_duration(segment_length, speed, segment_index, total_segments):
        """段持续时间计算"""
        if speed <= 0:
            return 0
        
        # 基础持续时间
        base_duration = segment_length / speed
        
        # 第一段和最后一段考虑加速/减速
        if segment_index == 0 or segment_index == total_segments - 2:
            # 起飞/降落段，增加额外时间用于加速/减速
            return base_duration * 1.2
        else:
            return base_duration

    @staticmethod
    def _determine_segment_speed(current_point, next_point, segment_index, total_segments):
        """速度确定策略"""
        config = Config.PLANNER_CONFIG
        
        # 计算段的高度变化和水平距离
        height_change = abs(next_point[2] - current_point[2])
        horizontal_distance = math.sqrt(
            (next_point[0] - current_point[0])**2 + 
            (next_point[1] - current_point[1])**2
        )
        
        # 段总长度
        segment_length = math.sqrt(horizontal_distance**2 + height_change**2)
        
        # 根据多种因素确定速度
        base_speed = config['cruise_speed']
        
        # 高度变化影响
        if height_change > 1.0:
            base_speed = config['takeoff_land_speed']
        elif height_change > 0.3:
            base_speed *= 0.8
        
        # 段长度影响
        if segment_length < 0.5:
            base_speed = min(base_speed, config['turn_speed'])
        
        # 首尾段减速
        if segment_index == 0 or segment_index == total_segments - 2:
            base_speed = config['takeoff_land_speed']
        
        return max(config['takeoff_land_speed'], min(config['cruise_speed'], base_speed))
    
    @staticmethod
    def print_trajectory_info(trajectory):
        """打印轨迹信息"""
        if not trajectory:
            print("没有轨迹数据")
            return
            
        print("\n=== 轨迹信息 ===")
        print("格式: [时间(s), 北坐标(m), 东坐标(m), 高度(m), 北向速度(m/s), 东向速度(m/s), 垂直速度(m/s)]")
        
        # 只打印部分点，避免输出太长
        max_display_points = 20
        if len(trajectory) > max_display_points:
            print(f"（显示前{max_display_points}个点，总共{len(trajectory)}个点）")
            display_points = trajectory[:max_display_points]
        else:
            display_points = trajectory
        
        for i, point in enumerate(display_points):
            time_stamp, x, y, z, vx, vy, vz = point
            speed = math.sqrt(vx*vx + vy*vy + vz*vz)
            print(f"点 {i+1:3d}: 时间{time_stamp:6.2f}s, 位置({x:6.2f}, {y:6.2f}, {z:6.2f}), "
                  f"速度({vx:5.2f}, {vy:5.2f}, {vz:5.2f}), 总速度{speed:5.2f}m/s")
        
        # 打印统计信息
        total_time = trajectory[-1][0]
        total_distance = TrajectoryGenerator._calculate_total_distance(trajectory)
        avg_speed = total_distance / total_time if total_time > 0 else 0
        
        print(f"\n统计信息:")
        print(f"总时间: {total_time:.2f}秒")
        print(f"总距离: {total_distance:.2f}米")
        print(f"平均速度: {avg_speed:.2f}米/秒")
        print(f"轨迹点数: {len(trajectory)}")
        
        # 计算速度统计
        speeds = [math.sqrt(point[4]**2 + point[5]**2 + point[6]**2) for point in trajectory]
        max_speed = max(speeds) if speeds else 0
        min_speed = min(speeds) if speeds else 0
        
        print(f"最大速度: {max_speed:.2f}米/秒")
        print(f"最小速度: {min_speed:.2f}米/秒")
        
        # 计算点间距统计
        if len(trajectory) > 1:
            spacings = []
            for i in range(len(trajectory) - 1):
                current = trajectory[i]
                next_point = trajectory[i + 1]
                dx = next_point[1] - current[1]
                dy = next_point[2] - current[2]
                dz = next_point[3] - current[3]
                spacing = math.sqrt(dx*dx + dy*dy + dz*dz)
                spacings.append(spacing)
            
            avg_spacing = sum(spacings) / len(spacings)
            max_spacing = max(spacings)
            min_spacing = min(spacings)
            
            print(f"平均点间距: {avg_spacing:.3f}米")
            print(f"最大点间距: {max_spacing:.3f}米")
            print(f"最小点间距: {min_spacing:.3f}米")
    
    @staticmethod
    def _calculate_total_distance(trajectory):
        """计算轨迹总距离"""
        if len(trajectory) < 2:
            return 0.0
            
        total_distance = 0.0
        for i in range(len(trajectory) - 1):
            current = trajectory[i]
            next_point = trajectory[i + 1]
            dx = next_point[1] - current[1]
            dy = next_point[2] - current[2]
            dz = next_point[3] - current[3]
            segment_length = math.sqrt(dx*dx + dy*dy + dz*dz)
            total_distance += segment_length
        return total_distance
    
    @staticmethod
    def export_trajectory_for_control(trajectory, filename=None):
        """导出轨迹数据用于控制层"""
        if not trajectory:
            print("没有轨迹数据可导出")
            return None
            
        # 创建控制层友好的格式
        control_data = {
            'waypoints': [],
            'metadata': {
                'total_points': len(trajectory),
                'total_time': trajectory[-1][0] if trajectory else 0,
                'total_distance': TrajectoryGenerator._calculate_total_distance(trajectory),
                'timestamp': time.time()
            }
        }
        
        for i, point in enumerate(trajectory):
            time_stamp, x, y, z, vx, vy, vz = point
            control_data['waypoints'].append({
                'id': i,
                'time': time_stamp,
                'position': [x, y, z],
                'velocity': [vx, vy, vz],
                'speed': math.sqrt(vx*vx + vy*vy + vz*vz)
            })
        
        # 如果指定了文件名，保存到文件
        if filename:
            import json
            try:
                with open(filename, 'w') as f:
                    json.dump(control_data, f, indent=2)
                print(f"轨迹数据已导出到: {filename}")
            except Exception as e:
                print(f"导出轨迹数据失败: {e}")
        
        return control_data


# ==============================
# 可视化模块
# ==============================
class Visualizer:
    """可视化类"""
    
    @staticmethod
    def show_2d_visualization(img, path_pixels, start_pixel, goal_pixel):
        """显示2D可视化"""
        plt.figure(figsize=Config.PLOT_CONFIG['figure_size_2d'])
        
        # 显示地图
        plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        
        # 提取坐标
        x_coords = [point[1] for point in path_pixels]  # 列坐标
        y_coords = [point[0] for point in path_pixels]  # 行坐标
        
        # 绘制路径
        plt.plot(x_coords, y_coords, 'g-', linewidth=1, alpha=0.7, label='飞行路径')
        plt.scatter(x_coords, y_coords, c='green', s=10, alpha=0.5)
        
        # 绘制起点终点
        Visualizer._plot_start_goal(start_pixel, goal_pixel)
        
        plt.xlabel('列坐标 (像素)')
        plt.ylabel('行坐标 (像素)')
        plt.title('2D路径规划结果')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
    
    @staticmethod
    def _plot_start_goal(start_pixel, goal_pixel):
        """绘制起点和终点"""
        plt.scatter([start_pixel[1]], [start_pixel[0]], c='red', s=100, marker='o', label='起点')
        plt.scatter([goal_pixel[1]], [goal_pixel[0]], c='blue', s=100, marker='^', label='终点')
    
    @staticmethod
    def show_3d_visualization(world_path, grid, map_resolution=0.05):
        """显示3D可视化"""
        fig = plt.figure(figsize=Config.PLOT_CONFIG['figure_size_3d'])
        ax = fig.add_subplot(111, projection='3d')
        
        # 提取并转换坐标（Z轴取负）
        x_coords, y_coords, z_coords = Visualizer._extract_and_transform_coords(world_path)
        
        # 绘制3D路径
        ax.plot(x_coords, y_coords, z_coords, 'g-', linewidth=2, label='飞行路径')
        ax.scatter([x_coords[0]], [y_coords[0]], [z_coords[0]], c='r', s=100, marker='o', label='起点')
        ax.scatter([x_coords[-1]], [y_coords[-1]], [z_coords[-1]], c='b', s=100, marker='^', label='终点')
        
        # 绘制高度限制平面
        Visualizer._plot_height_limit_plane(ax, grid, map_resolution)
        
        # 设置图形属性
        Visualizer._setup_3d_plot(ax, x_coords, y_coords, z_coords)
        plt.tight_layout()
        plt.show()
    
    @staticmethod
    def _extract_and_transform_coords(world_path):
        """提取并转换坐标"""
        x_coords = [point[0] for point in world_path]
        y_coords = [point[1] for point in world_path]
        z_coords = [-point[2] for point in world_path]  # Z轴取负
        return x_coords, y_coords, z_coords
    
    @staticmethod
    def _plot_height_limit_plane(ax, grid, map_resolution):
        """绘制高度限制平面"""
        if len(grid) > 0 and len(grid[0]) > 0:
            x_max = len(grid[0]) * map_resolution
            y_max = len(grid) * map_resolution
            
            xx, yy = np.meshgrid([0, x_max], [0, y_max])
            zz = np.full_like(xx, -2.0)  # -2米高度限制
            
            ax.plot_surface(xx, yy, zz, color='red', alpha=0.1, label='-2米高度限制')
    
    @staticmethod
    def _setup_3d_plot(ax, x_coords, y_coords, z_coords):
        """设置3D图形属性"""
        ax.set_xlabel('北方向 (米)')
        ax.set_ylabel('东方向 (米)')
        ax.set_zlabel('高度 (米)')
        ax.set_title('3D路径规划可视化')
        ax.legend()
        
        # 设置等比例坐标轴
        max_range = max(max(x_coords)-min(x_coords), 
                       max(y_coords)-min(y_coords),
                       max(z_coords)-min(z_coords)) * 0.5
        
        mid_x = (max(x_coords) + min(x_coords)) * 0.5
        mid_y = (max(y_coords) + min(y_coords)) * 0.5
        mid_z = (max(z_coords) + min(z_coords)) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        ax.set_box_aspect([1, 1, 1])  # 等比例显示
        ax.grid(True)
        ax.view_init(elev=30, azim=45)
    
    @staticmethod
    def show_velocity_profile(trajectory):
        """显示速度剖面图"""
        if not trajectory:
            return
        
        times = [point[0] for point in trajectory]
        velocities = [math.sqrt(point[4]**2 + point[5]**2 + point[6]**2) for point in trajectory]
        vx = [point[4] for point in trajectory]
        vy = [point[5] for point in trajectory]
        vz = [point[6] for point in trajectory]
        
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 1, 1)
        plt.plot(times, velocities, 'b-', linewidth=2, label='总速度')
        plt.plot(times, vx, 'r--', alpha=0.7, label='北向速度')
        plt.plot(times, vy, 'g--', alpha=0.7, label='东向速度')
        plt.plot(times, vz, 'm--', alpha=0.7, label='垂直速度')
        plt.xlabel('时间 (s)')
        plt.ylabel('速度 (m/s)')
        plt.title('速度剖面图')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(2, 1, 2)
        positions = [math.sqrt(point[1]**2 + point[2]**2) for point in trajectory]
        heights = [point[3] for point in trajectory]
        plt.plot(times, positions, 'b-', linewidth=2, label='水平位置')
        plt.plot(times, heights, 'r-', linewidth=2, label='高度')
        plt.xlabel('时间 (s)')
        plt.ylabel('位置 (m)')
        plt.title('位置-时间曲线')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()


# ==============================
# 路径规划器模块
# ==============================
class AStarPlanner3D:
    """3D A*路径规划器 - 优化版本"""
    
    def __init__(self):
        config = Config.PLANNER_CONFIG
        self.heuristic_weight = config['heuristic_weight']
        self.diagonal_cost = config['diagonal_cost']
        self.max_climb_angle = config['max_climb_angle']
        self.grid_resolution = config['grid_resolution']
        self.grid_cost = config['grid_cost']
        
        # 移动方向定义
        self.directions = self._define_movement_directions()
    
    def _define_movement_directions(self):
        """定义移动方向 - 8水平 + 2垂直"""
        return [
            (-1, 0, 0, 1.0), (1, 0, 0, 1.0), (0, -1, 0, 1.0), (0, 1, 0, 1.0),
            (-1, 1, 0, self.diagonal_cost), (1, 1, 0, self.diagonal_cost),
            (-1, -1, 0, self.diagonal_cost), (1, -1, 0, self.diagonal_cost),
            (0, 0, 1, 1.0), (0, 0, -1, 1.0)
        ]
    
    def heuristic(self, a, b):
        """启发式函数 - 综合考虑距离和高度"""
        dx, dy, dz = abs(a[0]-b[0]), abs(a[1]-b[1]), abs(a[2]-b[2])
        planar_distance = math.sqrt(dx*dx + dy*dy)
        
        # 高度代价（考虑爬升角约束）
        height_penalty = self._calculate_height_penalty(dz, planar_distance)
        
        return (planar_distance + height_penalty) * self.heuristic_weight
    
    def _calculate_height_penalty(self, dz, planar_distance):
        """计算高度惩罚项"""
        if planar_distance == 0:
            return abs(dz) * 3.0  # 纯高度变化代价更高
        
        actual_angle = math.degrees(math.atan2(abs(dz), planar_distance))
        if actual_angle <= self.max_climb_angle:
            return abs(dz) * 0.8  # 在允许角度内，代价较低
        else:
            # 超出最大爬升角，惩罚递增
            return abs(dz) * (1.5 + (actual_angle - self.max_climb_angle) * 0.2)
    
    def is_valid_position(self, grid, x, y, z):
        """检查位置是否有效（边界和障碍物检查）"""
        x_int, y_int, z_int = int(round(x)), int(round(y)), int(round(z))
        return (0 <= x_int < len(grid) and 
                0 <= y_int < len(grid[0]) and 
                0 <= z_int < 50 and
                grid[x_int][y_int] == 0)
    
    def find_path(self, grid, gaussian_grid, start, goal):
        """主路径规划函数"""
        start_time = time.time()
        
        # 验证起点终点
        if not self._validate_start_goal(grid, start, goal):
            return [], set()
        
        if start == goal:
            return [start], set()
        
        # 执行A*搜索
        path, explored, nodes_expanded = self._a_star_search(grid, gaussian_grid, start, goal)
        # return path, explored
    
        if path:
            optimized_path = self._optimize_path(path, grid)
            elapsed_time = time.time() - start_time
            
            print(f"3D路径规划成功! 耗时: {elapsed_time:.3f}s")
            print(f"扩展节点数: {nodes_expanded}")
            return optimized_path, explored
        
        print("无法找到可行路径!")
        return [], explored
    
    def _validate_start_goal(self, grid, start, goal):
        """验证起点和终点的有效性"""
        if not (self.is_valid_position(grid, *start) and self.is_valid_position(grid, *goal)):
            print("错误：起点或终点不可通行")
            return False
        return True
    
    def _a_star_search(self, grid, gaussian_grid, start, goal):
        """A*搜索算法核心实现"""
        open_list, closed_set, parent = [], set(), {}
        g_cost, explored = defaultdict(lambda: float('inf')), set()
        g_cost[tuple(start)] = 0
        nodes_expanded = 0
        
        # 初始化开放列表
        heapq.heappush(open_list, (self.heuristic(start, goal), *start))
        
        while open_list:
            current_f, current_x, current_y, current_z = heapq.heappop(open_list)
            current = (current_x, current_y, current_z)
            
            if current == tuple(goal):
                return self._reconstruct_path(parent, tuple(start), tuple(goal)), explored, nodes_expanded
            
            if current in closed_set:
                continue
                
            closed_set.add(current)
            explored.add(current)
            nodes_expanded += 1
            
            # 探索邻居节点
            self._explore_neighbors(grid, gaussian_grid, current, goal, open_list, closed_set, g_cost, parent)
        
        return [], explored, nodes_expanded
    
    def _explore_neighbors(self, grid, gaussian_grid, current, goal, open_list, closed_set, g_cost, parent):  
        """探索当前节点的所有邻居节点"""
        current_x, current_y, current_z = current
        
        for dx, dy, dz, move_cost in self.directions:
            neighbor = (current_x + dx, current_y + dy, current_z + dz)
            
            if not self.is_valid_position(grid, *neighbor) or neighbor in closed_set:  ### 该探索逻辑有点bug，这步会导致每个点只有一次更新机会
                continue
            
            # 计算考虑地形约束的实际移动代价
            actual_cost = self._calculate_actual_move_cost(current, neighbor, move_cost)
            tentative_g = g_cost[current] + actual_cost

            if tentative_g < g_cost[neighbor]:
                parent[neighbor] = current
                g_cost[neighbor] = tentative_g
                f_neighbor = tentative_g + self.heuristic(neighbor, goal) + gaussian_grid[int(current_x)][int(current_y)] * self.grid_cost
                heapq.heappush(open_list, (f_neighbor, *neighbor))
    
    def _calculate_actual_move_cost(self, current, neighbor, base_cost):
        """计算考虑爬升角约束的实际移动代价"""
        current_z, neighbor_z = current[2], neighbor[2]
        dx, dy = neighbor[0]-current[0], neighbor[1]-current[1]
        planar_distance = math.sqrt(dx*dx + dy*dy)
        
        if planar_distance > 0:
            climb_angle = math.degrees(math.atan2(abs(neighbor_z - current_z), planar_distance))
            if climb_angle > self.max_climb_angle:
                # 超出最大爬升角，增加代价
                return base_cost * (1.0 + (climb_angle - self.max_climb_angle) * 0.1)
        
        return base_cost
    
    def _reconstruct_path(self, parent, start, goal):
        """从父节点映射重建完整路径"""
        path, current = [], goal
        while current != start:
            path.append(list(current))
            current = parent[current]
        path.append(list(start))
        path.reverse()
        return path
    
    def _optimize_path(self, path, grid):
        """路径优化流程：简化 → 高度优化 → 平滑"""
        if len(path) <= 2:
            return path
        
        print(f"原始路径点数: {len(path)}")
        
        # 三步优化流程
        simplified = self._simplify_and_optimize_path(path)
        print(f"优化后路径点数: {len(simplified)}")
        
        return simplified
    
    def _simplify_and_optimize_path(self, path):
        """合并的路径优化函数：简化 + 高度优化 + 平滑"""
        if len(path) <= 2:
            return path
        
        # 1. 智能路径简化
        simplified = self._smart_path_simplification(path)
        return simplified
        # 2. 高度优化（平滑高度变化）
        height_optimized = self._optimize_height_profile(simplified)
        
        # 3. B样条平滑
        smoothed = self._adaptive_bspline_smooth(height_optimized)
        
        return smoothed
    
    def _smart_path_simplification(self, path):
        """混合路径简化方法 - 平衡简化效果和点数保留"""
        if len(path) <= 2:
            print(f"路径过短，无需简化: {len(path)}点")
            return path
        
        # 第一步：基于关键特征保留重要点
        key_points = self._extract_key_points(path)
        
        # 第二步：在关键点之间均匀采样，确保最小密度
        simplified = self._uniform_sampling_between_keypoints(path, key_points)
        
        print(f"混合简化: {len(path)} → {len(simplified)} 点")
        return simplified

    def _extract_key_points(self, path):
        """提取关键点（起点、终点、转弯点、高度变化点）"""
        if len(path) <= 2:
            return path
        
        key_points = [path[0]]  # 起点
        
        # 更宽松的阈值，保留更多关键点
        angle_threshold = math.radians(8)   # 适中角度阈值
        distance_threshold = 2.0            # 适中距离阈值
        height_threshold = 1.5              # 适中高度阈值
        
        for i in range(1, len(path) - 1):
            # 使用相邻点计算角度（更准确）
            angle_score = self._calculate_turn_angle_score(path, i)
            
            # 使用前一个路径点而不是上一个关键点计算距离
            if i > 0:
                distance_score = self._calculate_distance_score(path[i-1], path[i], path[i+1])
            else:
                distance_score = 0
                
            height_change = abs(path[i][2] - path[i-1][2])
            
            # 判断是否为关键点
            is_key_point = (
                angle_score > angle_threshold or
                distance_score > distance_threshold or
                height_change > height_threshold or
                i % 20 == 0  # 每20个点强制保留一个，防止长直线段过度简化
            )
            
            if is_key_point:
                key_points.append(path[i])
        
        key_points.append(path[-1])  # 终点
        return key_points

    def _uniform_sampling_between_keypoints(self, original_path, key_points):
        """在关键点之间进行均匀采样"""
        if len(key_points) <= 2:
            return key_points
        
        sampled_points = [key_points[0]]
        
        for i in range(len(key_points) - 1):
            start_point = key_points[i]
            end_point = key_points[i + 1]
            
            # 找到原始路径中这两个关键点之间的所有点
            start_idx = original_path.index(start_point)
            end_idx = original_path.index(end_point)
            
            if end_idx > start_idx + 1:
                # 计算这段子路径
                sub_path = original_path[start_idx:end_idx + 1]
                
                # 根据子路径长度决定采样点数
                sub_length = self._calculate_path_length(sub_path)
                num_samples = max(2, min(10, int(sub_length / 5)))  # 每5像素采样1点
                
                # 均匀采样
                if len(sub_path) > num_samples:
                    step = len(sub_path) / num_samples
                    for j in range(1, num_samples):
                        sample_idx = min(start_idx + int(j * step), end_idx - 1)
                        sampled_points.append(original_path[sample_idx])
                else:
                    # 如果点数不多，保留所有点
                    sampled_points.extend(sub_path[1:-1])
            
            sampled_points.append(end_point)
        
        return sampled_points

    def _calculate_turn_angle_score(self, path, index):
        """计算转弯角度得分"""
        if index <= 0 or index >= len(path) - 1:
            return 0
        
        prev, curr, next_p = path[index-1], path[index], path[index+1]
        
        vec1 = [curr[0]-prev[0], curr[1]-prev[1], curr[2]-prev[2]]
        vec2 = [next_p[0]-curr[0], next_p[1]-curr[1], next_p[2]-curr[2]]
        
        len1 = math.sqrt(sum(v**2 for v in vec1))
        len2 = math.sqrt(sum(v**2 for v in vec2))
        
        if len1 == 0 or len2 == 0:
            return 0
        
        dot_product = sum(v1 * v2 for v1, v2 in zip(vec1, vec2))
        cos_angle = max(-1.0, min(1.0, dot_product / (len1 * len2)))
        angle = math.acos(cos_angle)
        
        return angle

    def _calculate_distance_score(self, last_point, current_point, next_point):
        """计算距离得分 - 当前点到前后点连线的距离"""
        if last_point == next_point:
            return math.sqrt(
                (current_point[0]-last_point[0])**2 + 
                (current_point[1]-last_point[1])**2 + 
                (current_point[2]-last_point[2])**2
            )
        
        return self._perpendicular_distance(current_point, last_point, next_point)

    def _perpendicular_distance(self, point, line_start, line_end):
        """计算点到线段的垂直距离 - 新增方法"""
        if line_start == line_end:
            return math.sqrt(
                (point[0]-line_start[0])**2 + 
                (point[1]-line_start[1])**2 + 
                (point[2]-line_start[2])**2
            )
        
        # 计算线段长度平方
        line_length_sq = (
            (line_end[0]-line_start[0])**2 + 
            (line_end[1]-line_start[1])**2 + 
            (line_end[2]-line_start[2])**2
        )
        
        # 计算投影比例 t
        t = max(0, min(1, (
            (point[0]-line_start[0])*(line_end[0]-line_start[0]) +
            (point[1]-line_start[1])*(line_end[1]-line_start[1]) +
            (point[2]-line_start[2])*(line_end[2]-line_start[2])
        ) / line_length_sq))
        
        # 计算投影点
        projection = [
            line_start[0] + t*(line_end[0]-line_start[0]),
            line_start[1] + t*(line_end[1]-line_start[1]),
            line_start[2] + t*(line_end[2]-line_start[2])
        ]
        
        # 返回点到投影点的距离
        return math.sqrt(
            (point[0]-projection[0])**2 + 
            (point[1]-projection[1])**2 + 
            (point[2]-projection[2])**2
        )

    
    def _optimize_height_profile(self, path):
        """优化高度剖面 - 生成平滑的高度变化"""
        if len(path) < 2:
            return path
        
        print("应用高度优化...")
        optimized = [path[0]]
        
        for i in range(1, len(path)):
            progress = i / (len(path) - 1)
            eased_progress = progress ** 0.7  # 缓动函数实现平滑过渡
            
            # 计算目标高度（从起点到终点平滑过渡）
            target_height = path[0][2] + (path[-1][2] - path[0][2]) * eased_progress
            new_point = [path[i][0], path[i][1], target_height]
            optimized.append(new_point)
        
        return optimized
    
    def _adaptive_bspline_smooth(self, path, smoothness=0.5):
        """自适应B样条平滑 - 根据路径特征调整参数"""
        if len(path) < 4:
            return path
        
        try:
            x, y, z = [p[0] for p in path], [p[1] for p in path], [p[2] for p in path]
            
            # 计算路径特征
            path_length = self._calculate_path_length(path)
            path_complexity = self._estimate_path_complexity(path)
            
            # 根据特征调整平滑参数
            points_per_meter, smoothness_factor = self._get_smoothing_parameters(path_complexity)
            
            # 智能计算目标点数
            resolution = Config.MAP_CONFIG['resolution']
            world_length = path_length * resolution
            num_points = self._calculate_optimal_point_count(world_length, path_complexity, len(path))
            
            print(f"B样条平滑: 长度{world_length:.2f}m, 复杂度{path_complexity:.2f} -> {num_points}点")
            
            # B样条拟合
            tck, u = splprep([x, y, z], k=3, s=smoothness * smoothness_factor * len(path))
            u_new = np.linspace(0, 1, num_points)
            x_new, y_new, z_new = splev(u_new, tck)
            
            return [[float(x_new[i]), float(y_new[i]), float(z_new[i])] for i in range(len(x_new))]
            
        except Exception as e:
            print(f"B样条平滑失败: {e}")
            return path
    
    def _get_smoothing_parameters(self, complexity):
        """根据路径复杂度获取平滑参数"""
        if complexity > 0.7:    # 复杂路径
            return 10, 0.3      # 高密度，低平滑度
        elif complexity > 0.3:  # 中等复杂度
            return 5, 0.5
        else:                   # 简单路径
            return 3, 0.8      # 低密度，高平滑度
    
    def _calculate_optimal_point_count(self, world_length, complexity, original_points):
        """计算最优点数"""
        base_points = int(world_length * 15)  # 基准密度
        
        # 根据复杂度调整
        if complexity > 0.7:
            base_points = int(base_points * 1.3)
        elif complexity < 0.3:
            base_points = int(base_points * 0.7)
        
        # 设置合理范围
        min_points = max(30, original_points * 2)
        max_points = min(1000, int(original_points * 10))
        
        return max(min_points, min(max_points, base_points))
    
    def _estimate_path_complexity(self, path):
        """估计路径复杂度 (0-1) - 综合考虑转向和高度变化"""
        if len(path) < 3:
            return 0
        
        total_angle_change, total_height_change = 0, 0
        
        for i in range(1, len(path) - 1):
            # 计算转向角度
            angle = self._calculate_turn_angle(path[i-1], path[i], path[i+1])
            if angle is not None:
                total_angle_change += angle
            
            # 计算高度变化
            total_height_change += abs(path[i][2] - path[i-1][2])
        
        # 综合复杂度评估
        angle_complexity = total_angle_change / ((len(path) - 2) * math.pi)
        height_complexity = min(1.0, total_height_change / (len(path) * 5))
        
        return min(1.0, 0.7 * angle_complexity + 0.3 * height_complexity)
    
    def _calculate_turn_angle(self, prev, curr, next_p):
        """计算三点间的转向角度"""
        vec1 = [curr[0]-prev[0], curr[1]-prev[1], curr[2]-prev[2]]
        vec2 = [next_p[0]-curr[0], next_p[1]-curr[1], next_p[2]-curr[2]]
        
        len1 = math.sqrt(sum(v**2 for v in vec1))
        len2 = math.sqrt(sum(v**2 for v in vec2))
        
        if len1 == 0 or len2 == 0:
            return None
        
        dot_product = sum(v1 * v2 for v1, v2 in zip(vec1, vec2))
        cos_angle = max(-1.0, min(1.0, dot_product / (len1 * len2)))
        return math.acos(cos_angle)
    
    def _calculate_path_length(self, path):
        """计算路径总长度"""
        if len(path) < 2:
            return 0
        
        return sum(math.sqrt(
            (path[i+1][0]-path[i][0])**2 + 
            (path[i+1][1]-path[i][1])**2 + 
            (path[i+1][2]-path[i][2])**2
        ) for i in range(len(path)-1))


# ==============================
# 主程序
# ==============================
class PathPlanningApp:
    """路径规划应用程序"""
    
    def __init__(self):

        """
        初始化方法，设置类的基本属性
        """
        self.grid = None  # 网格地图，用于存储环境信息
        self.img = None  # 图像数据，可能用于可视化或处理
        self.start_pixel = [80, 20, 20]  # [行, 列, 高度] 起始点的像素坐标和高度信息
        self.goal_pixel = [100, 200, 40]   # [行, 列, 高度] 目标点的像素坐标和高度信息
        self.trajectory = None  # 存储生成的轨迹，用于存储规划后的路径点序列
        self._load_and_process_map()

    def run(self, start_pixel=None, goal_pixel=None):
        """运行应用程序"""
        if start_pixel and goal_pixel:
            self.start_pixel = start_pixel  # [行, 列, 高度] 起始点的像素坐标和高度信息
            self.goal_pixel = goal_pixel   # [行, 列, 高度] 目标点的像素坐标和高度信息
        # # 加载和处理地图
        # if not self._load_and_process_map():
        #     return None
        
        # 验证坐标
        if not self._validate_coordinates():
            return None
        
        # 执行路径规划
        path_pixels, explored = self._execute_path_planning()
        if not path_pixels:
            return None
        
        # 生成轨迹并显示结果
        # self._generate_and_display_trajectory(path_pixels, explored)
        # print("3D路径规划完成")
        return path_pixels
    
    def _load_and_process_map(self):
        """加载和处理地图"""
        img_path = sys.path[0] + "\\cropped_map.pgm"
        self.grid, self.img, self.gaussian_grid = MapProcessor.preprocess_map(img_path)
        return self.grid is not None
    
    def _validate_coordinates(self):
        """验证坐标有效性"""
        # 检查边界
        if not self._check_coordinate_bounds():
            return False
        
        # 检查可通行性
        return self._check_coordinate_accessibility()
    
    def _check_coordinate_bounds(self):
        """检查坐标边界"""
        grid_rows, grid_cols = len(self.grid), len(self.grid[0])
        
        if (self.start_pixel[0] < 0 or self.start_pixel[0] >= grid_rows or 
            self.start_pixel[1] < 0 or self.start_pixel[1] >= grid_cols or
            self.goal_pixel[0] < 0 or self.goal_pixel[0] >= grid_rows or 
            self.goal_pixel[1] < 0 or self.goal_pixel[1] >= grid_cols or
            self.start_pixel[2] < 0 or self.start_pixel[2] >= 50 or 
            self.goal_pixel[2] < 0 or self.goal_pixel[2] >= 50):
            
            print("起点或终点坐标超出地图范围!")
            print(f"地图范围: 行[0, {grid_rows-1}], 列[0, {grid_cols-1}], 高度[0, 49]")
            return False
        
        return True
    
    def _check_coordinate_accessibility(self):
        """检查坐标可通行性"""
        start_valid = self.grid[self.start_pixel[0]][self.start_pixel[1]] == 0
        goal_valid = self.grid[self.goal_pixel[0]][self.goal_pixel[1]] == 0
        
        if not start_valid or not goal_valid:
            print("起点或终点在障碍物膨胀区域内!")
            return False
        
        return True
    
    def _execute_path_planning(self):
        """执行路径规划"""
        print(f"起点坐标: 行={self.start_pixel[0]}, 列={self.start_pixel[1]}, 高度={self.start_pixel[2]}")
        print(f"终点坐标: 行={self.goal_pixel[0]}, 列={self.goal_pixel[1]}, 高度={self.goal_pixel[2]}")
        
        planner = AStarPlanner3D()
        path_pixels, explored = planner.find_path(self.grid, self.gaussian_grid, self.start_pixel, self.goal_pixel)
        
        return path_pixels, explored
    
    def _generate_and_display_trajectory(self, path_pixels, explored):
        """生成轨迹并显示结果（修改为相对坐标）"""
        # 1. 转换为绝对世界坐标
        world_path = CoordinateConverter.pixel_to_world(path_pixels)
        
        # 2. 转换为相对起点的坐标（新增步骤）
        relative_path = CoordinateConverter.world_to_relative(world_path)
        print("轨迹已转换为相对起点的坐标（起点为(0,0,0))")
        
        # 3. 使用相对坐标生成轨迹（替换原world_path为relative_path）
        self.trajectory = TrajectoryGenerator.generate_trajectory_with_time_and_velocity(relative_path)
        
        # 4. 打印轨迹信息（此时位置为相对坐标）
        TrajectoryGenerator.print_trajectory_info(self.trajectory)
        
        # 5. 可视化时使用相对坐标（替换原world_path为relative_path）
        Visualizer.show_2d_visualization(self.img, path_pixels, self.start_pixel, self.goal_pixel)
        # Visualizer.show_3d_visualization(relative_path, self.grid)  # 3D图显示相对坐标
        # Visualizer.show_velocity_profile(self.trajectory)
    
    def get_trajectory_for_control(self):
        """获取用于控制的轨迹数据"""
        if self.trajectory is None:
            return []
        
        # 返回格式: [时间(s), x(m), y(m), z(m), vx(m/s), vy(m/s), vz(m/s)]
        return self.trajectory


# ==============================
# 程序入口
# ==============================
if __name__ == "__main__":
    app = PathPlanningApp()
    app.run()
    
    # 获取轨迹数据用于控制层
    control_trajectory = app.get_trajectory_for_control()
    print(f"\n控制层可用的轨迹点数: {len(control_trajectory)}")