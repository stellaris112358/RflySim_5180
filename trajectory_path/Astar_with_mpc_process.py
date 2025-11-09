import numpy as np
import VisionCaptureApi
import PX4MavCtrlV4 as PX4MavCtrl
import time
import cv2
import math
import sys
import copy
import UE4CtrlAPI
from mpc import uav_mpc
from AStar import PathPlanningApp
import threading
from multiprocessing import Process, Queue  # 导入Queue用于进程通信


# -------------------------- 工具函数（无修改，保持不变）--------------------------
def pixel2world(path_pixels):
    if not path_pixels:
        return path_pixels
    resolution = 0.05
    world_path = []
    for pixel_point in path_pixels:
        world_x = pixel_point[1] * resolution + 22.45  # 列坐标 -> x
        world_y = pixel_point[0] * resolution - 4  # 行坐标 -> y  
        world_z = -pixel_point[2] * resolution - 0.1 # 高度坐标 -> z
        world_path.append([world_x, world_y, world_z])
    return world_path

def world2pixel(point):
    resolution = 0.05
    pixel_x = (point[0] - 22.45) / resolution  # 列坐标 -> x
    pixel_y = (point[1] + 4) / resolution  # 行坐标 -> y  
    pixel_z = -(point[2] + 0.091) / resolution  # 高度坐标 -> z
    pixel = [int(pixel_y), int(pixel_x), int(pixel_z)]
    return pixel

def calc_yaw_from_two_points(P1, P2, degrees=False):
    P1 = np.array(P1, dtype=np.float64)
    P2 = np.array(P2, dtype=np.float64)
    vec = P2 - P1
    x = vec[0]
    y = vec[1]
    if np.isclose(x, 0) and np.isclose(y, 0):
        return 0.0
        # raise ValueError("两点重合，无法计算yaw角")
    yaw_rad = np.arctan2(y, x)
    return yaw_rad

def calc_angular_velocity_2d(P1, P2, P3):
    P1 = np.array(P1, dtype=np.float64)
    P2 = np.array(P2, dtype=np.float64)
    P3 = np.array(P3, dtype=np.float64)
    v1 = P2 - P1
    v2 = P3 - P2
    v1_xy = v1[:2].flatten()
    v2_xy = v2[:2].flatten()
    norm_v1 = np.linalg.norm(v1[:2])
    norm_v2 = np.linalg.norm(v2[:2])
    if norm_v1 < 1e-8 or norm_v2 < 1e-8:
        return 0.0, 0.0, 0.0
    dot_product = np.dot(v1, v2)
    cos_theta = np.clip(dot_product / (norm_v1 * norm_v2), -1.0, 1.0)
    theta = np.arccos(cos_theta)
    cross_z = v1_xy[0] * v2_xy[1] - v2_xy[0] * v1_xy[1]
    direction = 1 if cross_z > 1e-8 else -1
    delta_theta = direction * theta
    delta_t = 1/30.0  # 时间间隔，与主循环一致
    if delta_t < 1e-8:
        raise ValueError("时间间隔过小，无法计算角速度")
    velocity = norm_v1 / delta_t
    omega = delta_theta / delta_t
    vz = v1[2] / delta_t
    return velocity, omega, vz

def mpc_info(pos, path=None):
    ur = np.zeros([5,3])
    Xr = pos
    idx = 0
    if path:
        distance = 1000
        for i in range(len(path)):
            dis = np.linalg.norm(np.array(path[i])-pos[:3])
            if dis < distance:
                distance = dis
            else:
                yaw = calc_yaw_from_two_points(path[i-1], path[i])
                Xr = np.array([path[i][0], path[i][1], path[i][2], yaw])
                idx = i
                break
        if len(path) - idx > 6:
            for i in range(5):
                vp, wp, vz = calc_angular_velocity_2d(path[i+idx-1], path[i+idx], path[i+idx+1])
                ur[i,0]+=vp
                ur[i,1]+=wp
                ur[i,2]+=vz
    return ur, Xr, idx


# -------------------------- 重构A*子进程：独立初始化+队列通信 --------------------------
def astar_process_func(position_queue, path_queue, num_drones):
    """
    A*路径规划子进程函数（独立初始化，无序列化问题）
    :param queue: 进程通信队列，用于传递 (Path, flag)
    :param num_drones: 无人机数量（3架跟踪机）
    """
    # 子进程内独立初始化路径规划器（不继承主进程对象）
    app = PathPlanningApp()
    
    while True:
        try:
            # 子进程无法直接访问主进程的MavList，需通过队列接收无人机位置？
            # ❌ 错误：之前的逻辑是子进程直接读MavList，多进程内存不共享，读不到最新值！
            # ✅ 修正：主进程将无人机位置发送给子进程，子进程计算路径后返回
            # （这里先调整为「子进程等待主进程发送位置」，避免内存不共享问题）
            try:
                drone_positions = position_queue.get(timeout=1.0)  # 1秒超时，避免无限等待
            except:
                continue  # 队列空时，继续循环等待  # 格式：[(uav1_pos), (uav2_pos), (uav3_pos)]

            # if drone_positions is None:  # 主进程发送None表示退出
            #     break
            
            # 转换为像素坐标
            start = [world2pixel(pos) for pos in drone_positions]
            goal = [
                world2pixel([24, 0, -1]),  # 无人机1的目标
                start[0],                  # 无人机2的目标（跟踪无人机1）
                start[0]                   # 无人机3的目标（跟踪无人机1）
            ]
            print("A*子进程：start=", start, "\ngoal=", goal)
            
            Path = []
            flag = 0  # 0=路径有效，1=无路径，2=路径过短
            for i in range(num_drones):
                path = app.run(start[i], goal[i])
                if not path:
                    flag = 1
                    print(f"{i+1}号无人机的路径不存在")
                elif len(path) < 6:
                    flag = 2
                    print(f"{i+1}号无人机的路径长度过短")
                Path.append(pixel2world(path))
            
            # 将结果发送回主进程（队列是进程安全的）
            path_queue.put( (Path, flag) )
            time.sleep(2)  # 每6秒重新计算一次路径
        
        except Exception as e:
            print('[A* 子进程] 异常:', e)
            path_queue.put( (None, 2) )  # 发送异常信号
            # break


# -------------------------- 主进程逻辑（必须包裹在if __name__ == "__main__"中）--------------------------
if __name__ == "__main__":
    # 1. 主进程初始化第三方API和无人机实例（仅主进程操作）
    vis = VisionCaptureApi.VisionCaptureApi()
    ue = UE4CtrlAPI.UE4CtrlAPI()
    vis.jsonLoad()
    
    MavList = []
    vehicleNum = 4  # 1架目标机 + 3架跟踪机
    for i in range(vehicleNum):
        MavList.append(PX4MavCtrl.PX4MavCtrler(1+i))  # 修正：避免列表拼接的低效写法
    
    # 2. 初始化取图和UE4设置
    isSuss = vis.sendReqToUE4()
    if not isSuss:
        sys.exit(0)
    vis.startImgCap()
    time.sleep(2)
    
    ue.sendUE4Cmd('RflyChangeViewKeyCmd S')
    # ue.sendUE4Cmd('RflyChangeViewKeyCmd T')
    ue.sendUE4Cmd('t.MaxFPS 30', 0)
    time.sleep(2)
    
    # 3. 初始化无人机（解锁、起飞）
    for i in range(vehicleNum):
        MavList[i].InitMavLoop(False)
        MavList[i].InitTrueDataLoop()
    for i in range(vehicleNum):
        MavList[i].initOffboard()
        MavList[i].SendMavArm(True)
    time.sleep(2)
    
    # 4. 发送初始位置（起飞到z=-1m）
    MavList[0].SendPosNED(0, 0, -1)  # 目标机
    MavList[1].SendPosNED(0, 0, -1)  # 跟踪机1
    MavList[2].SendPosNED(0, 0, -1)  # 跟踪机2
    MavList[3].SendPosNED(0, 0, -1)  # 跟踪机3
    time.sleep(10)  # 等待无人机稳定
    
    # 5. 初始化MPC（主进程内创建，仅主进程使用）
    uav1_mpc = uav_mpc()
    
    # 6. 创建进程通信队列，启动A*子进程
    num = 3  # 跟踪机数量
    position_queue = Queue()  # 主→子：位置队列
    path_queue = Queue()      # 子→主：路径队列
    sub_process = Process(
        target=astar_process_func,
        args=(position_queue, path_queue, num)  # 仅传递队列和数字（可序列化参数）
    )
    sub_process.start()
    time.sleep(0.5)
    print("A*子进程是否存活:", sub_process.is_alive())
    
    # 7. 主循环：控制无人机+推送位置给A*子进程+接收路径
    lastTime = time.time()
    timeInterval = 1/30.0  # 10Hz控制频率
    A = np.array([ [1, 0, 0, timeInterval, 0, 0],
                [0, 1, 0, 0, timeInterval, 0], 
                [0, 0, 1, 0, 0, timeInterval],   
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ])
    P = np.eye(6)*0.1
    H = np.eye(6)
    Q = np.eye(6)*0.01    # 过程噪声
    Rk = np.eye(6)*0.1
    def kalman(x, P, z):
        # 预测
        x_pred = A @ x
        P_pred = A @ P @ A.T + Q
        # 更新
        S = H @ P_pred @ H.T + Rk
        K = P_pred @ H.T @ np.linalg.inv(S)
        y = z - H @ x_pred          # 新息
        x_new = x_pred + K @ y
        P_new = (np.eye(6) - K @ H) @ P_pred @ (np.eye(6) - K @ H).T + K @ Rk @ K.T
        return x_new, P_new
    current_path = None
    current_flag = 0
    max_queue_size = 3  # 队列最大长度阈值（可根据需求调整）
    
    while True:
        # 控制循环频率
        lastTime += timeInterval
        sleepTime = lastTime - time.time()
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            lastTime = time.time()
        
        # -------------------------- 推送无人机位置给A*子进程 --------------------------
        # 读取3架跟踪机的当前位置
        drone_positions = [
            MavList[1].truePosNED,
            MavList[2].truePosNED,
            MavList[3].truePosNED
        ]
        # 关键：清空队列旧数据，避免堆积
        # if position_queue.qsize() >= max_queue_size:
        while not position_queue.empty():
            try:
                position_queue.get_nowait()
            except:
                break
            # print(f"队列堆积超过{max_queue_size}条，已清空旧数据")
        
        # 推送最新位置（此时队列仅含最新数据，无冗余）
        position_queue.put(drone_positions)
        
        # -------------------------- 接收A*子进程的路径结果 --------------------------
        if not path_queue.empty():
            result = path_queue.get()
            # if isinstance(result, tuple) and len(result) == 2:
            current_path, current_flag = result
        
        # -------------------------- MPC控制无人机 --------------------------
        # if current_flag == 2:
        #     print("路径过短，退出控制循环")
        #     break
        
        for i in range(num):
            uav_pos = MavList[i+1].truePosNED
            uav_ang = MavList[i+1].trueAngEular
            pos = np.array([uav_pos[0], uav_pos[1], uav_pos[2], uav_ang[2]])
            
            if current_path:
                if current_path[i]:
                    ur, Xr, idx = mpc_info(pos, current_path[i])
                else:
                    MavList[i+1].SendVelFRD(-0.1, 0, 0, 0)
                    print(f"无人机{i+1}：无有效路径")
                    continue
                # print(f"无人机{i+1}：MPC路径有效")
            else:
                ur, Xr, idx = mpc_info(pos, None)
                # print(f"无人机{i+1}：无有效路径")
            
            # 调用MPC求解控制指令
            U = uav1_mpc.step(ur, Xr, pos, 5)
            # print(f"无人机{i+1}：控制指令U=", U)
            MavList[i+1].SendVelFRD(U[0], 0, U[2], U[1])
        
        # -------------------------- UE4绘制路径 --------------------------
        if current_path and current_flag == 0:
            # 绘制无人机1的路径（ID 6~105）
            for i in range(min(len(current_path[0]), 100)):
                ue.sendUE4PosScale(i+6, 100000060, 0, 
                                  [current_path[0][i][0], current_path[0][i][1], current_path[0][i][2]], 
                                  [0, 0, 0], [0.05, 0.05, 0.05])
            # 绘制无人机2的路径（ID 116~215）
            for i in range(min(len(current_path[1]), 100)):
                ue.sendUE4PosScale(i+116, 101000060, 0, 
                                  [current_path[1][i][0], current_path[1][i][1], current_path[1][i][2]], 
                                  [0, 0, 0], [0.05, 0.05, 0.05])
            # 绘制无人机3的路径（ID 216~315）
            for i in range(min(len(current_path[2]), 100)):
                ue.sendUE4PosScale(i+216, 102000060, 0, 
                                  [current_path[2][i][0], current_path[2][i][1], current_path[2][i][2]], 
                                  [0, 0, 0], [0.05, 0.05, 0.05])
    
    # -------------------------- 退出清理 --------------------------
    print("退出程序，清理资源...")
    path_queue.put(None)  # 通知A*子进程退出
    sub_process.join()    # 等待子进程结束
    position_queue.close() # 关闭队列
    path_queue.close()    
    
    # 停止无人机
    for i in range(vehicleNum):
        MavList[i].SendVelFRD(0, 0, 0, 0)
        # MavList[i].SendMavArm(False)