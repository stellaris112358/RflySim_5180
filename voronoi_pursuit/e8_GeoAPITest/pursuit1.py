import time
import math
import sys
import os
import PX4MavCtrlV4 as PX4MavCtrl
import numpy as np

from circle_draw import calculate_drone_positions

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

poslist = [[0, 0, -2], [4, 0, -2], [0, 4, -2], [4, 4, -2]]

start_time = time.time()
while time.time() - start_time < 50:
    
    for j in range(4): 

        # MavList[3].SendPosNED(poslist[j][0], poslist[j][1], -2)  # 让第四架飞机起飞到2米高
        MavList[3].SendVelFRD(2,0,0,1)

        start_time2 = time.time()
        while time.time() - start_time2 < 3:
            target_uav = MavList[3].geo.lla2ned(MavList[3].uavPosGPS, GpsOrigin)  # 第四架飞机的世界系位置
            # print("error",abs(MavList[3].truePosGPS[0]-MavList[3].uavPosGPS[0]))
            # target_uav[2] = target_uav[2] - 4
            # print(f"target_uav_z",target_uav)  
            drone_positions = calculate_drone_positions(target_uav, r=2.5, n=VehilceNum - 1)
            for i in range(VehilceNum - 1):
                mav = MavList[i]
                targetPos = drone_positions[i]  

                targetPosLocal = np.array(targetPos) - np.array(mav.GpsOriOffset)

                targetPos = targetPosLocal.tolist()
                # print(f"targetPosLocal{i+1}: {targetPosLocal}")
                # print(f"pos{i+1}: {mav.pos}")
                print(f"uavPosNED{i+1}: {mav.uavPosNED}")
                print(f"truePosNED{i+1}: {mav.truePosNED}")
                print(f"uavGlobalPos{i+1}: {mav.uavGlobalPos}")
                # print(f"error{i+1}: {np.array(mav.truePosNED)-np.array(mav.pos)}")
                mav.SendPosNED(targetPosLocal[0], targetPosLocal[1], -5)
    
            time.sleep(1/15)
        

# 结束仿真
for i in range(VehilceNum):
    mav=MavList[i]
    mav.endOffboard()
    time.sleep(0.5)
    mav.endMavLoop()