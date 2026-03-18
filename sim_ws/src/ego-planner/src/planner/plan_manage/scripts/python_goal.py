#!/usr/bin/env python3
# coding: utf-8
import rospy
from geometry_msgs.msg import PoseStamped

def publish_waypoints(waypoints, delay=3.0):
    """
    连续发布多个目标点（航点）
    :param waypoints: 航点列表，每个元素是 (x, y, z, w) → w 是机头朝向的四元数 w 分量
    :param delay: 两个航点之间的时间间隔（秒），确保无人机完成前一段轨迹
    """
    rospy.init_node('python_goal', anonymous=True)
    goal_pub0 = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    # goal_pub1 = rospy.Publisher('/drone_1/move_base_simple/goal', PoseStamped, queue_size=10)
    # goal_pub2 = rospy.Publisher('/drone_2/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1.0)
    
    for i, (x, y, z, w) in enumerate(waypoints):
        goal_msg0 = PoseStamped()
        goal_msg0.header.frame_id = "world"
        goal_msg0.header.stamp = rospy.Time.now()
        
        # 位置
        goal_msg0.pose.position.x = x
        goal_msg0.pose.position.y = y
        goal_msg0.pose.position.z = z
        
        # 朝向（默认机头朝向 x 轴，w=1.0；若需调整朝向，修改 z 和 w 分量）
        goal_msg0.pose.orientation.x = 0.0
        goal_msg0.pose.orientation.y = 0.0
        goal_msg0.pose.orientation.z = 0.0
        goal_msg0.pose.orientation.w = w

        # goal_msg1 = PoseStamped()
        # goal_msg1.header.frame_id = "world"
        # goal_msg1.header.stamp = rospy.Time.now()
        
        # # 位置
        # goal_msg1.pose.position.x = x
        # goal_msg1.pose.position.y = y + 2
        # goal_msg1.pose.position.z = z
        
        # # 朝向（默认机头朝向 x 轴，w=1.0；若需调整朝向，修改 z 和 w 分量）
        # goal_msg1.pose.orientation.x = 0.0
        # goal_msg1.pose.orientation.y = 0.0
        # goal_msg1.pose.orientation.z = 0.0
        # goal_msg1.pose.orientation.w = w

        # goal_msg2 = PoseStamped()
        # goal_msg2.header.frame_id = "world"
        # goal_msg2.header.stamp = rospy.Time.now()
        
        # # 位置
        # goal_msg2.pose.position.x = x
        # goal_msg2.pose.position.y = y + 4
        # goal_msg2.pose.position.z = z
        
        # # 朝向（默认机头朝向 x 轴，w=1.0；若需调整朝向，修改 z 和 w 分量）
        # goal_msg2.pose.orientation.x = 0.0
        # goal_msg2.pose.orientation.y = 0.0
        # goal_msg2.pose.orientation.z = 0.0
        # goal_msg2.pose.orientation.w = w
        
        # 发布当前航点
        for _ in range(100):
            goal_pub0.publish(goal_msg0)
            # goal_pub1.publish(goal_msg1)
            # goal_pub2.publish(goal_msg2)
            # print("start to publish points!!!!!!!!!!!!!!!!!!")
            rospy.sleep(0.1)
        
        # rospy.loginfo("第 %d 个航点发布成功：(%.2f, %.2f, %.2f)", 
        #               i+1, x, y, z)
        
        # 等待无人机飞到当前航点（或按固定延迟发布下一个）
        # rospy.sleep(delay)
    while rospy.is_shutdown():
        goal_pub0.publish(goal_msg0)
        # goal_pub1.publish(goal_msg1)
        # goal_pub2.publish(goal_msg2)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        # 定义航点序列：[(x1,y1,z1,w1), (x2,y2,z2,w2), ...]
        waypoints = [
            (0.0, 0.0, 1.0, 1.0),    # 航点1：起点（假设无人机初始位置）
            (5, 1, 1.0, 1.0)    # 航点2：沿 x 轴飞行
        ]
        
        # 连续发布航点，间隔 5 秒（可根据无人机速度调整）
        publish_waypoints(waypoints, delay=50.0)
    except rospy.ROSInterruptException:
        rospy.logerr("节点被中断！")
        pass
