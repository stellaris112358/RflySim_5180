#! /usr/bin/env python

import rospy #导入ROS的Python库
from geometry_msgs.msg import PoseStamped #从ROS中的geometry_msgs包中导入PoseStamped消息类型，用于发布飞行器的位置
from mavros_msgs.msg import State #从ROS中的mavros_msgs包中导入State消息类型，用于获取飞行器的状态信息
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest #从ROS的mavros_msgs包中导入四个服务类型，用于控制飞行器的解锁、上锁和切换飞行模式

current_state = State() #创建一个名为current_state的State类型对象，用于存储当前的飞行器状态信息

def state_cb(msg): #定义一个名为state_cb的回调函数，用于接收来自飞行器状态话题的消息
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    # 方式二：根据当前命名空间自适应话题/服务路径
    ns = rospy.get_namespace().strip('/')  # e.g. "uav1" 或空字符串

    def rel(path):
        """使用相对名称，交由 ROS 根据当前节点命名空间自动解析。
        注意：如果节点已在 <group ns="uavX"> 启动，这里不再手动加前缀，避免出现 /uavX/uavX/... 的双重命名空间。
        """
        return path

    def abs_srv(path):
        """将相对服务名解析为绝对路径，随节点命名空间自动变化。
        例如：在 ns=uav0 下，resolve("mavros/cmd/arming") -> "/uav0/mavros/cmd/arming"；无命名空间时 -> "/mavros/cmd/arming"。
        """
        return rospy.resolve_name(path)

    # 订阅状态与发布位置（相对话题名，随命名空间变化）
    state_sub = rospy.Subscriber(rel("mavros/state"), State, callback=state_cb)
    local_pos_pub = rospy.Publisher(rel("mavros/setpoint_position/local"), PoseStamped, queue_size=10)

    # 等待并连接服务（使用绝对路径，避免解析歧义）
    rospy.wait_for_service(abs_srv("mavros/cmd/arming"))
    arming_client = rospy.ServiceProxy(rel("mavros/cmd/arming"), CommandBool)

    rospy.wait_for_service(abs_srv("mavros/set_mode"))
    set_mode_client = rospy.ServiceProxy(rel("mavros/set_mode"), SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()
