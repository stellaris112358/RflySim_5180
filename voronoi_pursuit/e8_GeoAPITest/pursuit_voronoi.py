import time
import math
import sys
import os
import PX4MavCtrlV4 as PX4MavCtrl
import numpy as np

from circle_draw import calculate_drone_positions
from voronoi import voronoi_shrink
from evader_control import get_evader_velocity

VehilceNum = 4
MavList=[]
# 创建4个无人机控制实例
for i in range(VehilceNum):
    MavList=MavList+[PX4MavCtrl.PX4MavCtrler(1+i)]

time.sleep(2)

GpsOrigin=[28.142958,112.983711,50.9] # 长沙地区地图的原点经纬高，以此坐标作为所有飞机的原点

# 使用 MAVLink_Full 模式来监听数据
for i in range(VehilceNum):
    MavList[i].InitMavLoop() # 启用监听循环
    MavList[i].setGPSOriLLA(GpsOrigin) # 设置所有飞机的共同GPS原点

# 让各个飞机进入Offboard模式，开始控制飞机的任务
time.sleep(2)
for i in range(VehilceNum):
    MavList[i].initOffboard() # 开启Offboard控制

# time.sleep(2)
# for i in range(VehilceNum):
#     print('Copter #',MavList[i].CopterID,', UE pos: ',MavList[i].uavGlobalPos)
    
time.sleep(2)
for i in range(VehilceNum):
    mav=MavList[i]
    posGPs = mav.uavPosGPSHome
    xyz = mav.geo.lla2ned(posGPs,GpsOrigin)
    print('Copter #',mav.CopterID,', XyzNedPos: ',xyz)
    mav.SendMavArm(True)
    
time.sleep(2)
for i in range(VehilceNum):
    mav=MavList[i]
    mav.SendPosNED(0, 0, -5)  # 让四架飞机起飞到10米高

time.sleep(5)

# 环境边界
X_MAX, X_MIN = 10,-10
Y_MAX, Y_MIN = 10,-10

idx = 4  # 追踪目标无人机的索引，这里是第四架无人机

start_time = time.time()
while time.time() - start_time < 50:
    
    P_matlab = np.zeros((VehilceNum, 3))
    for i in range(VehilceNum):
        mav = MavList[i]
        P_matlab[i, :] = np.array(mav.truePosNED)

    evader_pos = P_matlab[idx-1][:2]
    pursuer_pos = np.delete(P_matlab, idx-1, axis=0)
    pursuer_pos = pursuer_pos[:, :2]
    # print("Evader Position:", evader_pos)
    # print("Pursuer Positions:", pursuer_pos)
    p_velocities = voronoi_shrink(pursuer_pos, evader_pos, X_MAX, X_MIN, Y_MAX, Y_MIN)
    for i in range(VehilceNum - 1):
        mav = MavList[i]
        vx_p, vy_p = p_velocities[i] * 2.3
        mav.SendVelNED(vx_p, vy_p, 0, 0)

    e_velocity = get_evader_velocity(pursuer_pos, evader_pos, X_MAX, X_MIN, Y_MAX, Y_MIN, output_format='xy')
    vx_e, vy_e = e_velocity * 2.0
    MavList[idx-1].SendVelNED(vx_e,vy_e,0,0)

    time.sleep(1/50)


# 结束仿真
for i in range(VehilceNum):
    mav=MavList[i]
    mav.endOffboard()
    time.sleep(0.5)
    mav.endMavLoop()