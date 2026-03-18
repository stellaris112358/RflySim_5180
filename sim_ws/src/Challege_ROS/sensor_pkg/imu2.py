# import required libraries
# pip3 install pymavlink pyserial

import cv2
import numpy as np
import time
import VisionCaptureApi
import math
import ReqCopterSim
import RflyRosStart
import sys

# 启用ROS发布模式
VisionCaptureApi.isEnableRosTrans = True

req = ReqCopterSim.ReqCopterSim()  # 获取局域网内所有CopterSim程序的电脑IP列表
StartCopterID = 2
TargetIP = req.getSimIpID(
    StartCopterID
)  # 获取CopterSim的1号程序所在电脑的IP，作为目标IP
# 注意：如果是本电脑运行的话，那TargetIP是127.0.0.1的本机地址；如果是远程访问，则是192打头的局域网地址。
# 因此本程序能同时在本机运行，也能在其他电脑运行。
# TargetIP = "172.27.144.1"
print(TargetIP)
print("Request CopterSim Send data.")
req.sendReSimIP(StartCopterID)  # 请求回传数据到本电脑
vis = VisionCaptureApi.VisionCaptureApi(TargetIP)
vis.sendImuReqCopterSim(
    StartCopterID, TargetIP
)  # 发送请求，从目标飞机CopterSim读取IMU数据,回传地址为127.0.0.1，默认频率为200Hz
