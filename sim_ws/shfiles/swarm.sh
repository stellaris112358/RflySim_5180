#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS="$SCRIPT_DIR/.."





echo "启动 激光雷达 接口"
xterm -hold -e "cd $PWD/../src/Challege_ROS/sensor_pkg; python3 lidar.py"  &
sleep 5
echo "启动 IMU 接口"
xterm -hold -e "cd $PWD/../src/Challege_ROS/sensor_pkg; ROS_NAMESPACE=uav1 python3 imu1.py"  &
sleep 3
xterm -hold -e "cd $PWD/../src/Challege_ROS/sensor_pkg; ROS_NAMESPACE=uav2 python3 imu2.py"  &



echo "mavros"
xterm -hold -e "roslaunch mavros px4.launch __ns:=uav1 fcu_url:="udp://:20101@127.0.0.1:20100" tgt_system:=1" &
# xterm -hold -e "cd /root/sim_ws && source devel/setup.bash && roslaunch offboard_py rflysim_px4_udp.launch" &
sleep 3
xterm -hold -e "roslaunch mavros px4.launch __ns:=uav2 fcu_url:="udp://:20103@127.0.0.1:20102" tgt_system:=2" &
sleep 3

### 启动定位模块
echo "[UAV 1] 启动 Faster-LIO..."
xterm -hold -e "source $PWD/../devel/setup.bash ; roslaunch faster_lio rflysim.launch \
    uav_id:=uav1 \
    lid_topic:="/rflysim/sensor0/mid360_lidar" \
    imu_topic:="/uav1/rflysim/imu" \
    x_offset:=0.0 \
    y_offset:=0.0 \
    rviz:=false" &
echo "[UAV 2] 启动 Faster-LIO..."
xterm -hold -e "source $PWD/../devel/setup.bash ; roslaunch faster_lio rflysim.launch \
    uav_id:=uav2 \
    lid_topic:=/rflysim/sensor2/mid360_lidar \
    imu_topic:=/uav2/rflysim/imu \
    x_offset:=2.0 \
    y_offset:=0.0 \
    rviz:=false " &
sleep 5

# # 在新终端窗口中运行第四个命令
# xterm -hold -e "cd /root/sim_ws && source devel/setup.bash && roslaunch ego_planner rflysim.launch" &
# # 等待3秒
# sleep 5




#开启rviz 可视化
echo "启动 rviz..."
xterm -hold -e "rosrun rviz rviz -d $PWD/../src/faster_lio/rviz_cfg/rflysim.rviz" &
sleep 5

# ### 启动 TF 树可视化工具
xterm -hold -e "rosrun rqt_tf_tree rqt_tf_tree" &
sleep 2

# xterm -hold -e "cd /root/sim_ws && source devel/setup.bash && roslaunch mission_pkg test_cross.launch" &




