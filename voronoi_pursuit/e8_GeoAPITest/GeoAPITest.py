import time
import math
import sys
import os
import PX4MavCtrlV4 as PX4MavCtrl
import numpy as np

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

time.sleep(2)
for i in range(VehilceNum):
    print('Copter #',MavList[i].CopterID,', UE pos: ',MavList[i].uavGlobalPos)
    
time.sleep(2)
for i in range(VehilceNum):
    mav=MavList[i]
    posGPs = mav.uavPosGPSHome
    xyz = mav.geo.lla2ned(posGPs,GpsOrigin)
    print('Copter #',mav.CopterID,', XyzNedPos: ',xyz)
    
time.sleep(2)
for i in range(VehilceNum):
    mav=MavList[i]
    targetPos = [0,0,-40] # 起飞到40米高
    # 将targetPos从本机坐标系，转换到UE坐标系，使多个飞机坐标系统一
    
    targetPosLocal = np.array(targetPos) -np.array(mav.GpsOriOffset)
    mav.SendMavArm(True)
    # 发送命令，让飞机飞到地图[0,0,-2]位置
    targetPos=targetPosLocal.tolist()
    print(targetPos,targetPosLocal,mav.GpsOriOffset)
    mav.SendPosNED(targetPosLocal[0],targetPosLocal[1],targetPosLocal[2])
    print('Send to takeoff.')
    