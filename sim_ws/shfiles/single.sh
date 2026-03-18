#!/bin/bash

WS=/root/sim_ws


xterm -hold -e "cd /root/sim_ws/src/Challege_ROS/sensor_pkg && python3 main.py" &
# 使用 sleep 命令等待1.1秒
sleep 8

xterm -hold -e "cd /root/sim_ws && source devel/setup.bash && roslaunch ego_planner rviz.launch" &
sleep 5

# # 在新终端窗口中运行第一个命令
# xterm -hold -e "cd /root/sim_ws && source devel/setup.bash && roslaunch offboard_py rflysim_px4_udp.launch" &

# # 等待1秒
# sleep 5

# # 在新终端窗口中运行第一个命令
# xterm -hold -e "cd /root/sim_ws/src/Challege_ROS/object_det/scripts && python3 det.py" &

# # 等待1秒
# sleep 1

# # 在新终端窗口中运行第二个命令
# xterm -hold -e "cd /root/sim_ws/src/Challege_ROS/recognize_aruco && python3 image.py" &

# # 等待4秒
# sleep 5

# 在新终端窗口中运行第三个命令
# xterm -hold -e "roslaunch faster_lio rflysim.launch" &
xterm -hold -e "cd /root/sim_ws && source devel/setup.bash && roslaunch faster_lio rflysim_ori.launch" &


# 等待3秒
sleep 5

# 在新终端窗口中运行第四个命令
xterm -hold -e "cd /root/sim_ws && source devel/setup.bash && roslaunch ego_planner rflysim_ori.launch" &

# 等待10秒
sleep 6

xterm -hold -e "cd /root/sim_ws && source devel/setup.bash && roslaunch mission_pkg send_goal.launch" &

