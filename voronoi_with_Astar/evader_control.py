import numpy as np

def get_evader_velocity(pursuer_positions, evader_position, x_max, x_min, y_max, y_min, output_format='xy'):
    """
    使用人工势场法计算逃逸者在多追捕者场景下的速度。
    输入参数现在要求为 numpy 数组：
        pursuer_positions: np.ndarray, shape (n, 2)
        evader_position:  np.ndarray, shape (2,) 或 (1, 2)

    Args:
        pursuer_positions (np.ndarray): 追捕者位置数组，shape 为 (n,2)
        evader_position (np.ndarray): 逃逸者当前的位置，shape 为 (2,) 或 (1,2)
        x_max (float): 环境 X 轴正向边界
        x_min (float): 环境 X 轴负向边界
        y_max (float): 环境 Y 轴正向边界
        y_min (float): 环境 Y 轴负向边界
        output_format (str): 输出速度的格式, 'xy' 代表 (vx, vy), 'vw' 代表 (线速度, 角速度/方向)

    Returns:
        tuple: 根据 `output_format` 返回逃逸者的速度。
    """
    # 输入检查并规范化为一维坐标 (2,)
    pursuers = np.asarray(pursuer_positions)
    if pursuers.ndim == 1 and pursuers.size == 2:
        pursuers = pursuers.reshape(1, 2)
    if pursuers.ndim != 2 or pursuers.shape[1] != 2:
        raise ValueError("pursuer_positions must be an array of shape (n,2)")

    evader_arr = np.asarray(evader_position)
    if evader_arr.ndim == 2 and evader_arr.shape[0] == 1 and evader_arr.shape[1] == 2:
        evader_pos = evader_arr.flatten()
    elif evader_arr.ndim == 1 and evader_arr.size == 2:
        evader_pos = evader_arr
    else:
        raise ValueError("evader_position must be an array of shape (2,) or (1,2)")

    total_repulsive_force = np.zeros(2)

    # --- 1. 来自追捕者的斥力（向量化计算） ---
    pursuer_repulsion_gain = 1.0
    if pursuers.shape[0] > 0:
        vecs = evader_pos[np.newaxis, :] - pursuers   # shape (n,2)
        dists = np.linalg.norm(vecs, axis=1)         # shape (n,)
        dists_safe = np.maximum(dists, 1e-6)
        # repulsive force ~ vec / dist^3  (与原实现一致)
        repulsive_per_pursuer = pursuer_repulsion_gain * (vecs / (dists_safe[:, np.newaxis]**3))
        # 对于距离为0的追捕者，repulsive_per_pursuer 将被限制为有限值（由 dists_safe 防止除零）
        total_repulsive_force += np.sum(repulsive_per_pursuer, axis=0)

    # --- 2. 来自环境边界的斥力 ---
    # 边界斥力增益，通常需要比追捕者斥力更大以保证不会出界
    boundary_repulsion_gain = 2.0 
    # 开始计算边界斥力的安全距离
    safety_margin = 1.0  

    # 右边界 (X_MAX)
    dist_to_x_max = x_max - evader_pos[0]
    if dist_to_x_max < safety_margin:
        total_repulsive_force += np.array([-boundary_repulsion_gain / max(dist_to_x_max, 1e-6), 0])

    # 左边界 (X_MIN)
    dist_to_x_min = evader_pos[0] - x_min
    if dist_to_x_min < safety_margin:
        total_repulsive_force += np.array([boundary_repulsion_gain / max(dist_to_x_min, 1e-6), 0])

    # 上边界 (Y_MAX)
    dist_to_y_max = y_max - evader_pos[1]
    if dist_to_y_max < safety_margin:
        total_repulsive_force += np.array([0, -boundary_repulsion_gain / max(dist_to_y_max, 1e-6)])

    # 下边界 (Y_MIN)
    dist_to_y_min = evader_pos[1] - y_min
    if dist_to_y_min < safety_margin:
        total_repulsive_force += np.array([0, boundary_repulsion_gain / max(dist_to_y_min, 1e-6)])


    # --- 3. 计算最终速度 ---
    # 如果合力不为零，则归一化为方向向量
    if np.linalg.norm(total_repulsive_force) > 0:
        desired_direction = total_repulsive_force / np.linalg.norm(total_repulsive_force)
    else:
        # 如果没有受到任何力（例如在场地正中心且周围没有追捕者），可以设置一个随机游走策略
        desired_direction = np.random.rand(2) * 2 - 1
        nd = np.linalg.norm(desired_direction)
        if nd > 0:
            desired_direction /= nd
        else:
            # 极端情况处理
            desired_direction = np.array([1.0, 0.0])

    # 设定逃逸者的最大速度
    evader_max_speed = 1
    velocity_xy = desired_direction * evader_max_speed

    # --- 4. 格式化输出 ---
    if output_format == 'vw':
        # v 是线速度大小
        v = np.linalg.norm(velocity_xy)
        # w 是速度向量的方向角 (弧度)
        w = np.arctan2(velocity_xy[1], velocity_xy[0])
        return np.array([v, w])
    else: # 默认 'xy'
        return velocity_xy