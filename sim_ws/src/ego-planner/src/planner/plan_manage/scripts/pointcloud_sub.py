#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class GridMapSubscriber:
    def __init__(self):
        rospy.init_node('grid_map_subscriber', anonymous=True)
        
        # 存储障碍物信息
        self.obstacle_world = None
        self.obs_world = {}
        
        # 统计信息
        self.callback_count = 0
        
        # 订阅点云地图
        self.grid_map_sub = rospy.Subscriber(
            '/rflysim/sensor0/mid360_lidar',  # /grid_map/occupancy
            PointCloud2,
            self.grid_map_callback,
            queue_size=10
        )
        
        rospy.loginfo("Point cloud subscriber initialized")
    
    def grid_map_callback(self, msg):
        """
        处理 PointCloud2 消息
        Args:
            msg: PointCloud2 消息
        """
        self.callback_count += 1
        
        # 取出点云里所有点的 x,y,z
        pts = np.array([p[:3] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])
        
        if pts.size == 0:
            # 空点云
            self.obstacle_world = None
            rospy.logwarn(f"[Callback #{self.callback_count}] Empty point cloud received")
            return
        
        # 存储障碍物点云
        self.obstacle_world = pts
        
        # 详细统计信息
        num_points = len(pts)
        if num_points < 1000:
            magnitude = f"{num_points} points (< 1K)"
        elif num_points < 10000:
            magnitude = f"{num_points} points (~{num_points/1000:.1f}K)"
        elif num_points < 100000:
            magnitude = f"{num_points} points (~{num_points/1000:.0f}K)"
        else:
            magnitude = f"{num_points} points (~{num_points/1000000:.1f}M)"
        
        # 计算点云边界
        x_min, y_min, z_min = pts.min(axis=0)
        x_max, y_max, z_max = pts.max(axis=0)
        
        rospy.loginfo(f"[Callback #{self.callback_count}] Point cloud statistics:")
        rospy.loginfo(f"  - Total points: {magnitude}")
        rospy.loginfo(f"  - Bounding box: X[{x_min:.2f}, {x_max:.2f}], Y[{y_min:.2f}, {y_max:.2f}], Z[{z_min:.2f}, {z_max:.2f}]")
        rospy.loginfo(f"  - Size: {x_max-x_min:.2f}m x {y_max-y_min:.2f}m x {z_max-z_min:.2f}m")
        rospy.loginfo(f"  - Frame: {msg.header.frame_id}, Timestamp: {msg.header.stamp.to_sec():.3f}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        subscriber = GridMapSubscriber()
        subscriber.run()
    except rospy.ROSInterruptException:
        pass