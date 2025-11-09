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
import queue


vis = VisionCaptureApi.VisionCaptureApi()
ue = UE4CtrlAPI.UE4CtrlAPI()
vis.jsonLoad()
MavList = []
# Create MAV instance
# 创建两架飞机，一架作为目标，另一架作为跟踪飞机
vehicleNum = 4
for i in range(vehicleNum):
    MavList = MavList+[PX4MavCtrl.PX4MavCtrler(1+i)]
isSuss = vis.sendReqToUE4()  # 向RflySim3D发送取图请求，并验证
if not isSuss:  # 如果请求取图失败，则退出
    sys.exit(0)
vis.startImgCap()  # 开启取图，并启用共享内存图像转发，转发到填写的目录
time.sleep(2)
# Start MAV loop with UDP mode: MAVLINK_FULL

ue.sendUE4Cmd('RflyChangeViewKeyCmd S')
ue.sendUE4Cmd('RflyChangeViewKeyCmd T')

# ue.sendUE4Cmd('r.setres 720x405w', 0)
ue.sendUE4Cmd('t.MaxFPS 30', 0)  # 设置UE4最大刷新频率，同时也是取图频率
time.sleep(2)

for i in range(vehicleNum):
    MavList[i].InitMavLoop(False)
    MavList[i].InitTrueDataLoop()
for i in range(vehicleNum):
    MavList[i].initOffboard()
    MavList[i].SendMavArm(True)
time.sleep(2)

uav1_pos = MavList[1].truePosNED
uav2_pos = MavList[2].truePosNED
uav3_pos = MavList[3].truePosNED

print(uav2_pos)

MavList[0].SendPosNED(0, 0, -1)
MavList[1].SendPosNED(0, 0, -1)  # bat脚本里面会默认对copter_id = 2 的飞机y方向增加2m
MavList[2].SendPosNED(0, 0, -1)
MavList[3].SendPosNED(0, 0, -1)

uav1_mpc = uav_mpc()

time.sleep(10)
print(uav2_pos)
uav1_pos = MavList[1].truePosNED
uav2_pos = MavList[2].truePosNED
uav3_pos = MavList[3].truePosNED


def pixel2world(path_pixels):
    resolution = 0.05
    world_path = []
    for pixel_point in path_pixels:
        world_x = pixel_point[1] * resolution + 22.45  # 列坐标 -> x
        world_y = pixel_point[0] * resolution - 4  # 行坐标 -> y  
        world_z = -pixel_point[2] * resolution  # 高度坐标 -> z
        world_path.append([world_x, world_y, world_z])
    return world_path

def world2pixel(point):
    resolution = 0.05
    pixel_x = (point[0] - 22.45) / resolution  # 列坐标 -> x
    pixel_y = (point[1] + 4) / resolution  # 行坐标 -> y  
    pixel_z = -point[2] / resolution  # 高度坐标 -> z
    pixel = [int(pixel_y), int(pixel_x), int(pixel_z)]
    return pixel
    

flag = False
total_step = 300

num = 3
max_speed = 1
start = [world2pixel(uav1_pos),
         world2pixel(uav2_pos),
         world2pixel(uav3_pos)]
goal = [world2pixel([24, 0, -1]),
        start[0],
        start[0]]
timeInterval = 1/10.0

class Astar:
    def __init__(self):
        self.flag = False
        self.lock = threading.Lock()
        self.Path = None
        self.num = 3
        self.app = PathPlanningApp()

    def A_star_task(self):
        while True:
            try:
                self.flag = 0
                uav1_pos = MavList[1].truePosNED
                uav2_pos = MavList[2].truePosNED
                uav3_pos = MavList[3].truePosNED

                start = [world2pixel(uav1_pos),
                        world2pixel(uav2_pos),
                        world2pixel(uav3_pos)]
                goal = [world2pixel([24, 0, -1]),
                        start[0],
                        start[0]]
                print("start:",start)
                print("\n goal:",goal)
                with self.lock:
                    Path = []
                    for i in range(self.num):
                        path = self.app.run(start[i], goal[i])
                        if not path:
                            # self.Path = None
                            self.flag = 1
                            break
                        if len(path) < 6:
                            self.flag = 2
                        Path.append(pixel2world(path))
                    if not self.flag:
                        self.Path = Path
                        print("paths exist!")
                    else:
                        print("NO path exist!")
                time.sleep(6)
            except Exception as e:
                print('[A* THREAD] exception:', e)
                break
    
    def get_result(self):
        """主线程调用：获取子线程结果"""
        with self.lock:
            return self.Path, self.flag

# 2. 启动子线程
astar = Astar()
sub_thread = threading.Thread(target=astar.A_star_task)
sub_thread.start()
time.sleep(0.5)
print(sub_thread.is_alive())
time.sleep(0.5)

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

def calc_yaw_from_two_points(P1, P2, degrees=False):
    """
    由两个世界坐标计算yaw角（偏航角）
    参数：
        P1: 起点坐标，2D格式 (x1,y1) 或 3D格式 (x1,y1,z1)（列表/元组/NumPy数组）
        P2: 终点坐标，格式与P1一致
        degrees: 是否返回角度制（默认False，返回弧度制）
    返回：
        yaw: 偏航角（范围：弧度制 [-π, π]，角度制 [-180°, 180°]）
    """
    # 转换为NumPy数组，方便向量运算
    P1 = np.array(P1, dtype=np.float64)
    P2 = np.array(P2, dtype=np.float64)
    
    # 1. 计算方向向量（P2 - P1）
    vec = P2 - P1
    
    # 2. 取x-y平面投影（忽略z轴，若为2D则无影响）
    x = vec[0]
    y = vec[1]
    
    # 3. 处理特殊情况：两点重合（向量为零），yaw角无意义（返回0或抛出异常）
    if np.isclose(x, 0) and np.isclose(y, 0):
        raise ValueError("两点重合，无法计算yaw角")
    
    # 4. 计算yaw角（arctan2(y, x)：y是对边，x是邻边）
    yaw_rad = np.arctan2(y, x)
    
    # # 5. 可选：转换为角度制
    # if degrees:
    #     yaw = np.rad2deg(yaw_rad)
    # else:
    #     yaw = yaw_rad
    
    return yaw_rad

def calc_angular_velocity_2d(P1, P2, P3):
    # 转换为 NumPy 数组，方便向量运算
    P1 = np.array(P1, dtype=np.float64)
    P2 = np.array(P2, dtype=np.float64)
    P3 = np.array(P3, dtype=np.float64)
    
    # 1. 计算方向向量（以 P2 为中间点）
    v1 = P2 - P1  # 从 P1 到 P2 的向量
    v2 = P3 - P2  # 从 P2 到 P3 的向量
    v1_xy = v1[:2].flatten()  # 确保是 1D 数组 [x, y]
    v2_xy = v2[:2].flatten()
    # 2. 计算向量的模长（避免除以零）
    # velocity1 = np.linalg.norm(v1[:2])
    norm_v1 = np.linalg.norm(v1[:2])
    norm_v2 = np.linalg.norm(v2[:2])
    if norm_v1 < 1e-8 or norm_v2 < 1e-8:
        return 0.0, 0.0, 0.0  # 向量长度为0（点重合），角速度为0
    
    # 3. 点积求夹角余弦值（clip 避免数值误差导致 cosθ 超出 [-1,1]）
    dot_product = np.dot(v1, v2)
    cos_theta = np.clip(dot_product / (norm_v1 * norm_v2), -1.0, 1.0)
    theta = np.arccos(cos_theta)  # 夹角（0~π rad）
    
    # 4. 叉积判断旋转方向（2D 叉积的 z 分量）
    cross_product = np.cross(v1, v2)  # 结果为标量（z 分量）
    cross_z = v1_xy[0] * v2_xy[1] - v2_xy[0] * v1_xy[1]
    direction = 1 if cross_z > 1e-8 else -1  # 正=逆时针，负=顺时针
    delta_theta = direction * theta  # 带方向的角位移
    
    # 5. 计算时间间隔（取 P2-P3 的时间差，或平均时间差，这里用平均更稳定）
    delta_t = timeInterval  # 总时间差的一半，对应角位移 delta_theta 的时间
    if delta_t < 1e-8:
        raise ValueError("时间间隔过小，无法计算角速度")
    
    # 6. 角速度 = 角位移 / 时间间隔
    velocity = norm_v1 / delta_t
    omega = delta_theta / delta_t
    vz = v1[2] / delta_t
    return velocity, omega, vz

lastTime = time.time()

while True:
    lastTime = lastTime + timeInterval
    sleepTime = lastTime - time.time()
    if sleepTime > 0:
        time.sleep(sleepTime)  # sleep until the desired clock
    else:
        lastTime = time.time()
    Path, flag1 = astar.get_result()
    if flag1 == 2:
        break
    for i in range(num):
        uav_pos = MavList[i+1].truePosNED
        uav_ang = MavList[i+1].trueAngEular
        pos = np.array([uav_pos[0], uav_pos[1], uav_pos[2], uav_ang[2]])
        if Path:
            ur, Xr, idx = mpc_info(pos, Path[i])
            print("mpc paths exist!")
        else:
            ur, Xr, idx = mpc_info(pos, None)
            print("NO mpc path exist!")
        U = uav1_mpc.step(ur, Xr, pos, 5)
        print("U:",U)
        MavList[i+1].SendVelFRD(U[0], 0, U[2], U[1])
        # MavList[i+1].SendVelFRD(0, 0, 0, 0)
    if Path:
        for i in range(min(len(Path[0]), 100)):
            ue.sendUE4PosScale(i+6, 100000060, 0, [Path[0][i][0], Path[0][i][1], Path[0][i][2]], [0, 0, 0], [0.05, 0.05, 0.05])
        for i in range(min(len(Path[1]), 100)):
            ue.sendUE4PosScale(i+116, 101000060, 0, [Path[1][i][0], Path[1][i][1], Path[1][i][2]], [0, 0, 0], [0.05, 0.05, 0.05])
        for i in range(min(len(Path[2]), 100)):
            ue.sendUE4PosScale(i+216, 102000060, 0, [Path[2][i][0], Path[2][i][1], Path[2][i][2]], [0, 0, 0], [0.05, 0.05, 0.05])

        