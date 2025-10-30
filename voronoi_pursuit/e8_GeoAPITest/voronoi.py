import numpy as np
from scipy.spatial import Delaunay, Voronoi, cKDTree
import warnings

# --- 核心算法函数 ---

def voronoi_shrink(pursuers, evader, x_max, x_min, y_max, y_min):
    """
    根据MATLAB脚本实现的Voronoi围捕算法主函数。

    输入:
    - pursuers: (N, 2) numpy 数组，N个追捕者的位置。
    - evader: (1, 2)或(2,) numpy 数组，1个逃逸者的位置。
    - x_max, x_min, y_max, y_min: 环境边界。

    输出:
    - final_velocities: (N, 2) numpy 数组，N个追捕者的(x, y)速度向量。
    """
    
    # 1. 准备数据
    # 将逃逸者放在索引0的位置
    evader = np.array(evader).reshape(1, 2)
    pursuers = np.array(pursuers)
    P = np.vstack((evader, pursuers))
    idx = 0  # 逃逸者在P中的索引
    
    # 初始化一个列表来存储所有智能体（包括逃逸者）的速度
    # P中的索引 0 是逃逸者, 1...N 是追捕者
    miu_star = [None] * len(P)

    # 2. 计算 Voronoi 和 Delaunay
    try:
        dt = Delaunay(P)
        vor = Voronoi(P)
    except Exception as e:
        warnings.warn(f"Delaunay/Voronoi 计算失败: {e}. 可能是点共线。")
        # 尝试添加微小扰动
        P_ = P + np.random.rand(*P.shape) * 1e-6
        try:
            dt = Delaunay(P_)
            vor = Voronoi(P_)
        except Exception:
            warnings.warn("扰动后计算仍失败。返回零速度。")
            return np.zeros_like(pursuers)
            
    V = vor.vertices
    
    # 3. 找出逃逸者的有限顶点
    # vor.point_region 给出点P[i]对应的区域索引
    # vor.regions 给出每个区域的顶点索引列表（-1表示无穷远）
    evader_region_idx = vor.point_region[idx]
    evader_vertex_indices = vor.regions[evader_region_idx]
    
    finite_vertex_indices = [i for i in evader_vertex_indices if i != -1]
    if not finite_vertex_indices:
        warnings.warn("逃逸者没有有限的Voronoi顶点。")
        adjv = np.empty((0, 2))
    else:
        adjv = V[finite_vertex_indices]

    # 4. 找出逃逸者的邻居（通过Delaunay三角剖分）
    # dt.simplices 是构成三角剖分的顶点索引
    adjacentIndices = set()
    for simplex in dt.simplices:
        if idx in simplex:
            adjacentIndices.update([p for p in simplex if p != idx])
    
    adjacentIndices = list(adjacentIndices)
    # Pe 是所有邻居的位置
    Pe = P[adjacentIndices] 
    
    # 5. 遍历所有邻居，计算最优控制
    for pursuer_p_idx in adjacentIndices:
        # pursuer_p_idx 是该追捕者在 P 中的索引 (1 到 N)
        
        q1 = P[idx]         # 逃逸者位置
        q2 = P[pursuer_p_idx] # 追捕者位置
        
        epsilon = q1 - q2
        epsilon_len = np.linalg.norm(epsilon)
        if epsilon_len < 1e-9:
            epsilon_len = 1e-9 # 避免除以零

        # 找到 q1 和 q2 中垂线上的 Voronoi 顶点
        # ck_indices 是 adjv 中的索引
        ck_indices, k = findMidPerpendicularPoints(q1, q2, adjv)
        ck = adjv[ck_indices]
        
        v1, v2 = None, None
        neibor = False
        
        if len(ck) == 2:
            # 情况1：找到两个顶点，构成一条有限边
            v1, v2 = ck[0], ck[1]
            neibor = True
        elif len(ck) == 1:
            # 情况2：找到一个顶点，构成一条射线（开放单元）
            v1 = ck[0]
            # 检查v1是否在边界内
            if (x_min < v1[0] < x_max) and (y_min < v1[1] < y_max):
                neibor = True
                # 计算射线与边界的交点 v2
                v2 = findInfVertex(q1, q2, v1, k, Pe, x_max, x_min, y_max, y_min)
                if v2 is None:
                    neibor = False # 射线未与边界相交
        
        if neibor:
            # 6. 计算 nh, nv, l, L
            mid = (q1 + q2) / 2
            try:
                nh, nv, l, L = findhvlL(mid, q1, q2, epsilon, v1, v2, k)
            except Exception as e:
                warnings.warn(f"findhvlL failed for pursuer {pursuer_p_idx}: {e}")
                continue

            # 7. 计算最优控制 miu_star
            alphah = -L / 2.0
            alphav_numerator = (l**2 - (l - L)**2)
            alphav = alphav_numerator / epsilon_len / 2.0
            
            denom = np.sqrt(alphah**2 + alphav**2)
            if denom < 1e-9:
                denom = 1e-9 # 避免除以零
                
            miustar = -( (alphah / denom) * nh + (alphav / denom) * nv )
            miu_star[pursuer_p_idx] = miustar

    # 8. 组装最终的速度输出
    final_velocities = np.zeros_like(pursuers)
    for i in range(1, len(P)): # 遍历P中所有追捕者 (索引 1 到 N)
        pursuer_list_idx = i - 1 # 对应到 pursuers 数组中的索引 (0 到 N-1)
        
        if miu_star[i] is not None:
            # 该追捕者是邻居，使用计算出的 miu_star
            final_velocities[pursuer_list_idx] = miu_star[i]
        else:
            # 该追捕者不是邻居，简单地朝向逃逸者移动
            dir_vec = P[idx] - P[i]
            norm = np.linalg.norm(dir_vec)
            if norm < 1e-9:
                final_velocities[pursuer_list_idx] = np.zeros(2)
            else:
                final_velocities[pursuer_list_idx] = dir_vec / norm
                
    return final_velocities

# --- 辅助工具函数 (Utils) ---

def lineardis(q1, q2):
    """计算两个点之间的欧几里得距离"""
    return np.linalg.norm(np.array(q1) - np.array(q2))

def findMidPerpendicularPoints(q1, q2, points, epsilon=1e-6):
    """
    找到点集中位于 q1 和 q2 中垂线上的点。
    返回: (点在points中的索引, 中垂线的斜率k)
    """
    q1, q2 = np.array(q1), np.array(q2)
    points = np.array(points)
    
    if points.shape[0] == 0:
        return np.array([], dtype=int), 0

    dx = q2[0] - q1[0]
    dy = q2[1] - q1[1]

    if np.abs(dx) < epsilon and np.abs(dy) < epsilon:
        raise ValueError("Points q1 and q2 are coincident")

    # 计算中垂线斜率 k
    if np.abs(dx) < epsilon: # 原直线垂直
        k = 0.0                # 中垂线水平
    elif np.abs(dy) < epsilon: # 原直线水平
        k = np.inf             # 中垂线垂直
    else:
        k = -1.0 / (dy / dx)
        
    m = (q1 + q2) / 2.0
    
    # 中垂线方程: dx * (x - m[0]) + dy * (y - m[1]) = 0
    # -> dx * x + dy * y = dx * m[0] + dy * m[1]
    lhs = dx * points[:, 0] + dy * points[:, 1]
    rhs = dx * m[0] + dy * m[1]
    errors = np.abs(lhs - rhs)
    
    result_indices = np.where(errors < epsilon)[0]
    return result_indices, k

def isPointOnSegment(q1, q2, p, tol=1e-6):
    """
    检查点 p 是否在线段 q1-q2 上。
    使用距离检查，比MATLAB中的坐标检查更鲁棒。
    """
    q1, q2, p = np.array(q1), np.array(q2), np.array(p)
    d_q1_q2 = lineardis(q1, q2)
    if d_q1_q2 < tol:
        return lineardis(p, q1) < tol
    
    d_p_q1 = lineardis(p, q1)
    d_p_q2 = lineardis(p, q2)
    
    # 检查 p到q1 和 p到q2 的距离之和是否等于 q1到q2的距离
    return np.abs(d_p_q1 + d_p_q2 - d_q1_q2) < tol

def getOrthogonalUnitVector(v):
    """获取一个2D单位向量的正交单位向量 (逆时针旋转90度)"""
    v = np.array(v)
    # v 假定已经是单位向量
    return np.array([-v[1], v[0]])

def findhvlL(mid, e, p, epsilon, v1, v2, k):
    """计算 nh, nv, l, L"""
    l1 = lineardis(mid, v1)
    l2 = lineardis(mid, v2)
    L = lineardis(v1, v2)
    
    epsilon_len = np.linalg.norm(epsilon)
    if epsilon_len < 1e-9:
        raise ValueError("Epsilon length is zero")
    nh = epsilon / epsilon_len
    
    if isPointOnSegment(v1, v2, mid, 1e-3):
        nv = getOrthogonalUnitVector(nh)
        # 确定nv的方向
        if np.dot(nv, (v1 - mid)) > 0:
            l = l2
        elif np.dot(nv, (v2 - mid)) > 0:
            l = l1
        else:
            # mid 几乎与 v1 或 v2 重合
            l = max(l1, l2)
    else:
        # mid 不在 L 上
        nv = getOrthogonalUnitVector(nh)
        # nv 必须指向 (v1 - mid) 和 (v2 - mid) 的相反方向
        if np.dot(nv, (v1 - mid)) > 0 or np.dot(nv, (v2 - mid)) > 0:
            nv = -nv
        l = max(l1, l2)
        
    return nh, nv, l, L

def findRayIntersection(x0, y0, slope, xmin, xmax, ymin, ymax, dirx, diry):
    """
    计算从 (x0, y0) 出发，斜率为 slope，方向为 (dirx, diry) 的射线
    与矩形 (xmin, xmax, ymin, ymax) 的交点。
    """
    intersectionPoints = []
    
    # 射线方程: y = slope * (x - x0) + y0
    # -> y = slope * x + b
    b = y0 - slope * x0
    
    # 情况1: 垂直射线 (slope = inf)
    if np.isinf(slope):
        if diry == -1: intersectionPoints.append([x0, ymin])
        elif diry == 1: intersectionPoints.append([x0, ymax])
        else: raise ValueError("diry 必须是 1 或 -1 (垂直射线)")
        
    # 情况2: 水平射线 (slope = 0)
    elif np.abs(slope) < 1e-9:
        if dirx == -1: intersectionPoints.append([xmin, y0])
        elif dirx == 1: intersectionPoints.append([xmax, y0])
        else: raise ValueError("dirx 必须是 1 或 -1 (水平射线)")
        
    # 情况3: 有斜率的射线
    else:
        # 与左边界 X = xmin 相交
        yLeft = slope * xmin + b
        if ymin <= yLeft <= ymax: intersectionPoints.append([xmin, yLeft])
        # 与右边界 X = xmax 相交
        yRight = slope * xmax + b
        if ymin <= yRight <= ymax: intersectionPoints.append([xmax, yRight])
        # 与下边界 Y = ymin 相交
        xBottom = (ymin - b) / slope
        if xmin <= xBottom <= xmax: intersectionPoints.append([xBottom, ymin])
        # 与上边界 Y = ymax 相交
        xTop = (ymax - b) / slope
        if xmin <= xTop <= xmax: intersectionPoints.append([xTop, ymax])
    
    if not intersectionPoints:
        return None

    intersectionPoints = np.array(intersectionPoints)
    
    # 2. 根据射线方向 (dirx, diry) 筛选交点
    mask = np.ones(intersectionPoints.shape[0], dtype=bool)
    tol = 1e-9 # 容忍度
    
    if dirx == 1:
        mask &= (intersectionPoints[:, 0] >= x0 - tol)
    elif dirx == -1:
        mask &= (intersectionPoints[:, 0] <= x0 + tol)
        
    if diry == 1:
        mask &= (intersectionPoints[:, 1] >= y0 - tol)
    elif diry == -1:
        mask &= (intersectionPoints[:, 1] <= y0 + tol)
        
    filteredPoints = intersectionPoints[mask]

    # 去除重复点 (例如交在角落)
    if filteredPoints.shape[0] > 0:
        unique_points, indices = np.unique(np.round(filteredPoints, 8), axis=0, return_index=True)
        filteredPoints = filteredPoints[indices]
    
    if filteredPoints.shape[0] == 0:
        return None # v1在边界外，且射线方向背离边界
    elif filteredPoints.shape[0] == 1:
        return filteredPoints[0]
    else:
        # 如果有多个交点（不太可能，除非v1在边界上），
        # 选择距离 v1(x0, y0) 最近的一个
        dists = np.linalg.norm(filteredPoints - np.array([x0, y0]), axis=1)
        return filteredPoints[np.argmin(dists)]

def findInfVertex(e, p, v1, k, Pe, x_max, x_min, y_max, y_min):
    """
    计算开放Voronoi单元的射线与边界的交点。
    e: 逃逸者, p: 追捕者, v1: 射线起点, k: 射线斜率, Pe: 所有邻居
    """
    e, p, v1 = np.array(e), np.array(p), np.array(v1)
    
    # 计算 e-p 连线的斜率 k_
    dx = p[0] - e[0]
    dy = p[1] - e[1]
    
    if np.abs(dx) < 1e-9:
        k_ = np.inf # 垂直
        b = np.inf
    else:
        k_ = dy / dx
        b = p[1] - k_ * p[0]
        
    if np.abs(k_) < 1e-3: k_ = 1e-3 # 避免除以0

    # 找到除 p 之外的任意一个邻居
    other_node = None
    for node in Pe:
        if not np.all(np.isclose(node, p)):
            other_node = node
            break
    if other_node is None:
        raise ValueError("no other node found")

    # 检查 other_node 在 e-p 连线的哪一侧
    if np.isinf(k_): # e-p 垂直
        df = other_node[0] - p[0]
    else: # e-p 非垂直
        # 方程 y = k_ * x + b
        # df = y_node - (k_ * x_node + b)
        lhs = b + k_ * other_node[0]
        rhs = other_node[1]
        df = rhs - lhs
        
    dirx, diry = 0, 0

    # 这个逻辑来自MATLAB脚本，用于确定射线的方向
    # k 是 Voronoi 边的斜率, k_ 是 e-p 连线的斜率
    
    if np.isinf(k_): # e-p 垂直, k=0 (Voronoi 边水平)
        diry = 0
        if df > 0: dirx = -1 # other_node 在右侧, 射线向左
        else: dirx = 1      # other_node 在左侧, 射线向右
    elif np.abs(k_) < 1e-3: # e-p 水平, k=inf (Voronoi 边垂直)
        dirx = 0
        if df > 0: diry = -1 # other_node 在上方, 射线向下
        else: diry = 1      # other_node 在下方, 射线向上
    elif df > 0 and k_ > 0:
        dirx = 1; diry = -1
    elif df < 0 and k_ < 0:
        dirx = 1; diry = 1
    elif df < 0 and k_ > 0:
        dirx = -1; diry = 1
    elif df > 0 and k_ < 0:
        dirx = -1; diry = -1
    else:
        # 处理 k_ 和 df 符号相同但未在上面列出的情况
        # 这是一个备用逻辑，以防万一
        if k > 0: # Voronoi 边斜率
            dirx = -1 if df > 0 else 1
            diry = -1 if df > 0 else 1
        else:
            dirx = 1 if df > 0 else -1
            diry = -1 if df > 0 else 1
            
    if dirx == 0 and diry == 0:
        warnings.warn("Ray direction (dirx, diry) is (0,0). Defaulting to (1,1).")
        dirx, diry = 1, 1 # 防止(0,0)

    v2 = findRayIntersection(v1[0], v1[1], k, x_min, x_max, y_min, y_max, dirx, diry)
    return v2


if __name__ == '__main__':
    # 环境边界
    X_MAX, X_MIN = 3, -2
    Y_MAX, Y_MIN = 3, -2

    # 计算追捕者的速度
    # 注意：matlab脚本中的P将逃逸者放在了idx=2的位置
    # 而我的python函数将逃逸者放在了P[0]的位置。
    # 为了匹配你的matlab示例 P (idx=2是逃逸者)
    
    P_matlab = np.array([
        [ 0.5,  0.0],  # P[0] -> pursuer
        [ 0.0,  0.5],  # P[1] -> pursuer
        [-0.5, -0.5],  # P[2] -> EVADER (idx=2)
        [-0.2, -0.1],  # P[3] -> pursuer
        [-0.1,  0.1],  # P[4] -> pursuer
        [ 0.1, -0.1],  # P[5] -> pursuer
        [ 0.1,  0.1]   # P[6] -> pursuer
    ])
    
    evader_pos = P_matlab[2]
    pursuer_pos = np.delete(P_matlab, 2, axis=0)
    
    print("逃逸者位置:\n", evader_pos)
    print("追捕者位置 ({}个):\n".format(len(pursuer_pos)), pursuer_pos)
    
    # 调用算法
    velocities = voronoi_shrink(pursuer_pos, evader_pos, X_MAX, X_MIN, Y_MAX, Y_MIN)
    
    print("\n计算出的追捕者速度 ({}个):\n".format(len(velocities)), velocities)