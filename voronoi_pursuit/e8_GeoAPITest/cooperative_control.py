import math

def cooperative_control(x, y, vx, vy, xt, yt, ships_x, ships_y, k_att, k_rep, d_desired):
    """
    多无人船协同围捕控制算法（3船围捕）
    
    输入：
        x, y: 当前无人船位置坐标
        vx, vy: 当前无人船速度（预留参数，当前算法未使用）
        xt, yt: 目标船位置坐标
        ships_x: 其他无人船的x坐标列表
        ships_y: 其他无人船的y坐标列表
        k_att: 吸引力增益系数
        k_rep: 排斥力增益系数
        d_desired: 无人船之间的期望距离（安全距离）
    
    输出：
        u, v: 控制输入（力或速度指令）
    """
    
    # 吸引力计算（朝向目标船）
    dx_att = xt - x
    dy_att = yt - y
    dist_att = math.sqrt(dx_att**2 + dy_att**2)
    
    # 避免除零错误（当无人船与目标位置重合时）
    if dist_att < 1e-6:
        u_att = 0.0
        v_att = 0.0
    else:
        u_att = k_att * dx_att / dist_att
        v_att = k_att * dy_att / dist_att
    
    # 排斥力计算（与其他无人船保持安全距离）
    u_rep = 0.0
    v_rep = 0.0
    
    # 遍历其他所有无人船
    for i in range(len(ships_x)):
        dx_rep = ships_x[i] - x  # 其他船到当前船的x方向距离
        dy_rep = ships_y[i] - y  # 其他船到当前船的y方向距离
        dist_rep = math.sqrt(dx_rep**2 + dy_rep**2)
        
        # 当距离小于期望安全距离时，产生排斥力
        if dist_rep < d_desired and dist_rep > 1e-6:  # 避免除零（自身与自身的距离）
            # 排斥力计算（基于人工势场法）
            rep_factor = k_rep * (1 / dist_rep - 1 / d_desired)
            u_rep -= rep_factor * (dx_rep / (dist_rep**2))
            v_rep -= rep_factor * (dy_rep / (dist_rep**2))
    
    # 总控制输入 = 吸引力 + 排斥力
    u = u_att + u_rep
    v = v_att + v_rep
    
    return u, v