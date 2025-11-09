# -*- coding: utf-8 -*-
import math

def calculate_drone_positions(target_pos, r = 2, n = 3):
    # 1. 提取目标位置的坐标
    x0, y0, z0 = target_pos
    
    # 2. 计算每架无人机的角度间隔（弧度制，避免角度制计算误差）
    angle_step = 2 * math.pi / n  # 总圆周角2π，平均分成n份
    
    # 3. 循环计算每架无人机的位置
    drone_positions = []
    for i in range(n):  # i从0到n-1，对应第1到第n架无人机
        # 3.1 计算第i架无人机的角度（从x轴正方向开始逆时针旋转）
        angle = i * angle_step
        
        # 3.2 极坐标转直角坐标（计算相对于目标的偏移量）
        delta_x = r * math.cos(angle)  # x方向偏移
        delta_y = r * math.sin(angle)  # y方向偏移
        
        # 3.3 计算最终坐标（z坐标与目标一致）
        xi = x0 + delta_x
        yi = y0 + delta_y
        zi = z0  # 同一水平面的核心：z坐标不变
        
        # 3.4 将结果加入列表
        drone_positions.append([xi, yi, zi] )
    
    # 4. 返回所有无人机的目标位置
    return drone_positions