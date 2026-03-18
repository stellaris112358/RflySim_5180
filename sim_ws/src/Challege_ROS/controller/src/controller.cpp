#include "controller.h"

Controller::Controller(ros::NodeHandle &nh) : nh_(nh)
{

  Init();
  std::string depth_topic = "/camera/depth/image_raw";
  nh_.param("depth_topic", depth_topic, std::string("/camera/depth/image_raw"));
  nh_.param("cameras_param/rgb_K", cam.rgb_K, std::vector<double>());
  nh_.param("cameras_param/depth_K", cam.depth_K, std::vector<double>());
  nh_.param("cameras_param/rgb2depth_R", cam.rgb2depth_R,
            std::vector<double>());
  nh_.param("cameras_param/rgb2depth_T", cam.rgb2depth_T,
            std::vector<double>());
  nh_.param("rflysim/enable", rflysim_p.enable, true);
  nh_.param("rflysim/f_rgb", rflysim_p.f_rgb, 320.);
  nh_.param("rflysim/f_depth", rflysim_p.f_depth, 320.);
  nh_.param("rflysim/rgb_image_w", rflysim_p.rgb_image_width, 640);
  nh_.param("rflysim/rbg_image_h", rflysim_p.rgb_image_height, 480);
  nh_.param("rflysim/depth_image_w", rflysim_p.depth_image_width, 640);
  nh_.param("rflysim/depth_image_h", rflysim_p.depth_image_height, 480);
  nh_.param<std::vector<double>>(
    "rflysim/cam2body_R", rflysim_p.depth_cam2body_R, std::vector<double>());
  nh_.param<std::vector<double>>(
    "rflysim/cam2body_T", rflysim_p.depth_cam2body_T, std::vector<double>());
  nh_.param("rflysim/depth_down_sample", rflysim_p.depth_down_sample, 5);
  nh_.param("auto_arming", auto_arming, false);
  nh.param("takeoff_h", takeoff_h, 0.5);
  nh.param("takeoff_yaw", takeoff_yaw, 0.);
  nh.param("kx", kx, 0.0);
  nh.param("ky", ky, 0.0);
  nh.param("vx_max", vx_max, 0.0);
  nh.param("vy_max", vy_max, 0.0);
  nh.param("/rflysim/goal_x_t", rflysim_p.goal_x_t, 0.);
  nh.param("hight_max", rflysim_p.hight_max, 3.0);
  nh.param("rflysim/rgb_ppx", rflysim_p.rgb_cnt.x,
           rflysim_p.rgb_image_width / 2.);
  nh.param("rflysim/rgb_ppy", rflysim_p.rgb_cnt.y,
           rflysim_p.rgb_image_height / 2.);
  nh.param("rflysim/depth_ppx", rflysim_p.depth_cnt.x,
           rflysim_p.depth_image_width / 2.);
  nh.param("rflysim/depth_ppy", rflysim_p.depth_cnt.y,
           rflysim_p.rgb_image_height / 2.);
  nh.param("rflysim/min_score", rflysim_p.min_score, 0.7);
  nh.param("rflysim/rgb_fov_h", rflysim_p.rgb_fov_h, 90.);
  nh.param("rflysim/rgb_fov_v", rflysim_p.rgb_fov_v, 90.);
  nh.param("rflysim/is_sim", rflysim_p.is_sim, true);
  nh.param("is_S", rflysim_p.is_S, false);

  rflysim_p.rgb_fov_h *= (M_PI / 180);
  rflysim_p.rgb_fov_v *= (M_PI / 180);

  //  rflysim_p.rgb_cnt.x   = int(rflysim_p.rgb_image_width / 2);
  //  rflysim_p.rgb_cnt.y   = int(rflysim_p.rgb_image_height / 2);
  //  rflysim_p.depth_cnt.x = int(rflysim_p.depth_image_width / 2);
  //  rflysim_p.depth_cnt.y = int(rflysim_p.depth_image_height / 2);

  depth_img_sub = nh.subscribe<sensor_msgs::Image>(
    depth_topic, 10,
    std::bind(&Controller::DepthImgCB, this, std::placeholders::_1));

  fcu_state_sub = nh.subscribe<mavros_msgs::State>(
    "/mavros/state", 10,
    std::bind(&Controller::RecvFcuState, this, std::placeholders::_1));
  fcu_pose_sub = nh.subscribe<nav_msgs::Odometry>(
    "/mavros/local_position/odom", 10,
    std::bind(&Controller::RecvPose, this, std::placeholders::_1));
  tra_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
    "/planning/pos_cmd", 10,
    std::bind(&Controller::RecvTra, this, std::placeholders::_1));
  obj_sub = nh.subscribe<common_msgs::Objects>(
    "/objects", 10,
    std::bind(&Controller::RecvAiCB, this, std::placeholders::_1));
  odom_sub = nh.subscribe<nav_msgs::Odometry>(
    "/Odometry", 10,
    std::bind(&Controller::RecvLIO, this, std::placeholders::_1));
  aruco_sub = nh.subscribe<common_msgs::Aruco>(
    "/Aruco", 10,
    std::bind(&Controller::RecvAruco, this, std::placeholders::_1));

  pose_pub =
    nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

  ctrl_cmd_pub =
    nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
  goal_pub =
    nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  vis_pub         = nh.advertise<sensor_msgs::PointCloud2>("/test", 10);
  full_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/full_points", 10);
  task_state_pub  = nh.advertise<common_msgs::MissionState>("/task_state", 10);

  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  arming_client =
    nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  run_timer =
      nh_.createTimer(ros::Duration(0.03),
                     std::bind(&Controller::Run, this,
                      std::placeholders::_1));
  ROS_INFO("construct  finished");
}

bool Controller::Init()
{
  // ctrl_source = CtrlSource::Servo;
  // //使用伺服控制，但前提是相机的视场角要够大，下视相机视场角够大，所以在降落的时候可以使用伺服控制
  is_recv_pose   = false;
  is_recv_state  = false;
  is_recv_tra    = 0;
  is_aruco       = false;
  is_H           = false;
  goal_flag      = 0;
  send_goal_flag = 0;
  ctrl_source    = CtrlSource::Trajectroy;
  mission        = Mission::takeoff;  //初始化完成可以起飞
  auto_arming    = false;
  is_rec_frame1  = false;
  is_rec_frame2  = false;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  land.request.custom_mode          = "AUTO.LAND";
  arm_cmd.request.value             = true;
  //  mission_points;
  cmd.coordinate_frame = Command::FRAME_LOCAL_NED;
  cmd.type_mask =
    ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
  cmd.type_mask &= (~uint16_t(Command::FORCE));  //把 FORCE 屏蔽掉
  cmd.header.frame_id = "odom";
  kx                  = 0.;
  ky                  = 0.;
  ROS_INFO("finished init");
  return true;
}

void Controller::Run(const ros::TimerEvent &e)
{ 
  //如果有定位数据
  if(!is_recv_pose || !is_recv_state)
    return;
  if(!fcu_state.connected)
    return;
  //  PrintMission();

  StateChange();
  switch(mission)
  {
    case Mission::takeoff:
      if(fcu_state.mode != "OFFBOARD")
      {  //还没及切换模式，先发目标值给非控制切换模式
        cmd.type_mask &=
          ~(Command::IGNORE_PX | Command::IGNORE_PY | Command::IGNORE_PZ
            | Command::IGNORE_YAW);  // 使用位置控制起飞
        for(int i = 0; i < 10; ++i)
        {
          cmd.position.x = 0;
          cmd.position.y = 0;
          cmd.position.z = takeoff_h;
          cmd.yaw        = takeoff_yaw;
          ctrl_cmd_pub.publish(cmd);
          ros::spinOnce();
          sleep(0.01);
        }
        if( auto_arming && set_mode_client.call(offb_set_mode)
           && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
      }
      if(fcu_state.mode == "OFFBOARD" && auto_arming && !fcu_state.armed)
      {
        if(arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
      }
      if(std::abs(fcu_pose.pose.position.z - takeoff_h) < 0.1)
      {  // 飞到指定高度，切换模式
        mission = Mission::cross_frame1;
        // mission = Mission::recognize_aruco; //调试，直接进入识别模式
      }
      ctrl_cmd_pub.publish(cmd);
      break;
    case Mission::land:
      //降落模式 切换到伺服控制
      // cmd.type_mask |= Command::IGNORE_PX | Command::IGNORE_PY
      //                  | Command::IGNORE_PZ;
      // cmd.type_mask &=
      //   (Command::IGNORE_VX | Command::IGNORE_VY | Command::IGNORE_VZ);
      cmd.coordinate_frame =
        Command::FRAME_BODY_NED;  // 速度控制基于body坐标系控制
      if(ros::Time::now().toSec() - cmd.header.stamp.toSec() > 10)
      {  //如果超过两秒中没有更新指令，直接降落
        set_mode_client.call(land);
      }
      ctrl_cmd_pub.publish(cmd);

      break;
    case Mission::end:
      //发生异常情况，直接降落
      break;
    case Mission::cross_frame1:
    case Mission::cross_frame2:

      ROS_WARN("send cmd to contrl drone");
      //发送控制命令前，得先发目标点给规划节点

      ctrl_cmd_pub.publish(cmd);
      break;
    case Mission::recognize_aruco:
      //如果到识别二维码的状态，可能二维码没在下视相机视场角内，这个时候应该飞高一点，使二维码或者降落标志在视场内
      cmd.coordinate_frame = Command::FRAME_LOCAL_NED;
      cmd.type_mask =
        ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
      cmd.type_mask &= (~uint16_t(Command::FORCE));  //把 FORCE 屏蔽掉
      cmd.type_mask &= ~(Command::IGNORE_VX | Command::IGNORE_VY
                         | Command::IGNORE_VZ);  // 使用速度控制；
      cmd.header.frame_id = "base_link";
      if(!is_aruco)
      {  //此时二维码没在视场角内，应该飞高一点，但是如果识别到了降落标志，还没识别到二维码，说明图像清晰度不够，应超降落标志飞去，逐渐降低高度
        cmd.velocity.x = 0.01;
        cmd.velocity.y = 0.2;  //二维码趋势在此时机体坐标系的y方向
        if(rflysim_p.is_S)
        {
          cmd.velocity.y *= -1;
        }
        cmd.velocity.z = 0.05;  //速度不能太快，当然高度需要加上一个约束值，与此同时需要考虑气流的影响；
        if(fcu_pose.pose.position.z > rflysim_p.hight_max)
        {  //此时高度，不能再高了,此时降落表示应该识别了
          ROS_ERROR("vichle'z is too height");
          cmd.velocity.y = 0;
          cmd.velocity.z = 0;
        }
        if(is_H)
        {  //发现降落标识，没识别出二维码
          //视觉伺服控制，x,y 往降落标志点飞行
          auto dx =
            (obj.left_top_x + obj.right_bottom_x) / 2. - rflysim_p.rgb_cnt.x;
          auto dy =
            (obj.left_top_y + obj.right_bottom_y) / 2. - rflysim_p.rgb_cnt.y;
          cmd.velocity.y =
            -dx * kx;  //至于方向，需要确定下视单目图像坐标系与机体坐标系的关系
          cmd.velocity.x = dy * ky;
          if(fcu_pose.pose.position.z < rflysim_p.hight_max)
            cmd.velocity.z = 0.1;  //继续上升以便识别二维码
          else
          {  //这种情况放弃识别二维码
            mission = Mission::land;
          }
          cmd.header.stamp = ros::Time::now();
        }
      }
      ctrl_cmd_pub.publish(cmd);
      ros::spinOnce();
      cmd.velocity.y = 0;
      cmd.velocity.z = 0;
    
      break;
    case Mission::recognize_H:
      //如果识别出了二维码，没扫描到降落表示，可能降落标志没在视场内，此时应飞高，并往识别了二维码的位置飞去
      // cmd.coordinate_frame = Command::FRAME_LOCAL_NED;
      cmd.type_mask =
        ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
      cmd.type_mask &= (~uint16_t(Command::FORCE));  //把 FORCE 屏蔽掉
      cmd.type_mask &= ~(Command::IGNORE_VX | Command::IGNORE_VY
                         | Command::IGNORE_VZ);  // 使用速度控制；
      cmd.coordinate_frame = Command::FRAME_BODY_NED;
      if(!is_H)
      {  //没识别到降落标志
        static  bool is_cnt_aurco = false;
        std::cout << " aruco: " << aruco.cnt_x << ", " << aruco.cnt_y
                  << std::endl;
        std::cout << " rgb_cnt: " << rflysim_p.rgb_cnt.x << ", "
                  << rflysim_p.rgb_cnt.y << std::endl;
        int    dx = int(aruco.cnt_x - rflysim_p.rgb_cnt.x);
        int    dy = int(aruco.cnt_y - rflysim_p.rgb_cnt.y);
        double vx = -dy * kx;
        double vy = -dx * ky;
        if(std::abs(vx) > vx_max)
          vx = vx / std::abs(vx) * vx_max;
        if(std::abs(vy) > vy_max)
          vy = vy / std::abs(vy) * vy_max;
        std::cout << "dx: " << dx << ", dy: " << dy << std::endl;
        std::cout << "vx: " << cmd.velocity.x << ", vy: " << cmd.velocity.y
                  << std::endl;
        cmd.velocity.x   = vx;
        cmd.velocity.y   = vy;
        cmd.velocity.z   = 0.05;
  
        if(fcu_pose.pose.position.z > rflysim_p.hight_max)
        {  //此时高度，不能再高了,此时降落表示应该识别了
          ROS_ERROR("vichle'z is too height");
          cmd.velocity.z = 0;
        }

        if(std::abs(dx) < 10 && std::abs(dy) < 10)
        {
          is_cnt_aurco = true;
        }
        if(is_cnt_aurco)
        {
          cmd.velocity.x = 0.05;
        }
        cmd.header.stamp = ros::Time::now();
        ctrl_cmd_pub.publish(cmd);
      }
      is_H = false;
      break;
  };
  ros::spinOnce();
}

void Controller::DepthImgCB(const sensor_msgs::Image::ConstPtr &depth)
{
  if(!rflysim_p.is_sim)
  {
    return;
  }
  std::unique_lock<std::mutex> lock(depth_mtx);
  depth_queue.push(*depth);
  while(true)
  {
    auto front = depth_queue.front();
    if(ros::Time::now() - front.header.stamp > ros::Duration(1))
    {  //把超过一秒外的数据丢弃
      depth_queue.pop();
    }
    else
    {
      return;
    }
  }
}

void Controller::RecvPose(const nav_msgs::Odometry::ConstPtr &pose)
{
  fcu_pose.pose   = pose->pose.pose;
  fcu_pose.header = pose->header;
  is_recv_pose    = true;
}

void Controller::RecvFcuState(const mavros_msgs::State::ConstPtr &state)
{
  fcu_state     = *state;
  is_recv_state = true;
  ;
}

void Controller::RecvAiCB(const common_msgs::Objects::ConstPtr &objs)
{  //假设已经收到了这样的结构体std::vector<frame>，frame:left_top_x left_top_y
   //,right_button_x, right_button_y;
  if(objs->objects.empty())
    return;
  if(mission == Mission::recognize_aruco)
  {
    if(objs->objects.size() == 1 && objs->objects[0].score > rflysim_p.min_score
       && objs->objects[0].class_name == "land" && objs->sensor_id == 2)
    {
      is_H = true;
      obj  = objs->objects[0];
    }
  }
  if(mission == Mission::recognize_H || mission == Mission::land)
  {  //进入降落模式,使用伺服控制，计算vx,vy， 基于body 坐标系
    cmd.velocity.x = 0;
    cmd.velocity.y = 0;
    cmd.velocity.z = 0;
    for(int i = 0; i < objs->objects.size(); ++i)
    {
      if(objs->objects[i].class_name == "land"
         && objs->objects[i].score > rflysim_p.min_score && objs->sensor_id == 2)
      { //还得知道数据来自哪路图像
        is_H     = true;
        auto obj = objs->objects[i];
        auto dx =
          (obj.left_top_x + obj.right_bottom_x) / 2. - rflysim_p.rgb_cnt.x;
        auto dy =
          (obj.left_top_y + obj.right_bottom_y) / 2. - rflysim_p.rgb_cnt.y;
        std::cout << "land dx:  " << dx << " dy: " << dy << std::endl;
        double vx = -dy * kx;
        double vy = -dx * ky;
        if(std::abs(vx) > vx_max)
          vx = vx / std::abs(vx) * vx_max;
        if(std::abs(vy) > vy_max)
          vy = vy / std::abs(vy) * vy_max;
        cmd.velocity.x = vx;
        cmd.velocity.y = vy;
        std::cout << "vx: " << cmd.velocity.x << ", vy: " << cmd.velocity.y
                  << std::endl;
        if(std::abs(dx) < 10 && std::abs(dy) < 10)
        {
          //cmd.velocity.z = -0.5;  // 符合降落条件了
          set_mode_client.call(land);
        }
        cmd.header.stamp = ros::Time::now();
      }
    };
    return;
  }
  //  if((mission != Mission::cross_frame1 || mission != Mission::cross_frame2)
  //     && mission_points.find(mission) != mission_points.end())
  //  {  //如果不是穿框任务，或者已经找到目标框的中心位置；
  //    return;
  //  }

  //@back
  if(mission != Mission::cross_frame1 && mission != Mission::cross_frame2)
  {  //如果不是穿框或者穿环,下面的流程不用走了。
   return;
  }

  if(!rflysim_p.is_sim)
  {  //如果是真机模式不需要使用深度读取
    pcl::PointXYZ cnt;
    ObjConterRGB(objs, &cnt);
    ROS_INFO("camera corr p: x:%f, y:%f, z:%f",cnt.x,cnt.y,cnt.z);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(
      new pcl::PointCloud<pcl::PointXYZ>());
    //因为直接使用的检测框作为目标框，得加上一个往前的偏移量
    cnt.z += rflysim_p.goal_x_t;
    ROS_INFO("cnt.z:%f goal_x_t:%f",cnt.z,rflysim_p.goal_x_t);
    tmp->points.push_back(cnt);
    CoordinateTrans(tmp);
    VisionPointCloud(tmp,vis_pub);
    goal_point.pose.position.x = tmp->points[0].x;
    goal_point.pose.position.y = tmp->points[0].y;
    goal_point.pose.position.z = tmp->points[0].z;
    ROS_INFO("goal: x:%f,y:%f,z:%f",goal_point.pose.position.x,goal_point.pose.position.y,goal_point.pose.position.z);
    if(mission == Mission::cross_frame1)
    {
      goal_flag = 1;
      if(send_goal_flag == 0)
      {
        goal_1        = goal_point;
        is_rec_frame1 = true;
        goal_pub.publish(goal_1);
        ROS_INFO("goal1: x:%f,y:%f,z:%f",goal_1.pose.position.x,goal_1.pose.position.y,goal_1.pose.position.z);
        ros::spinOnce();
        send_goal_flag = 1;
      }
    }
    else if(mission == Mission::cross_frame2)
    {
      if(send_goal_flag == 1)
      {
        double dx = goal_1.pose.position.x - goal_point.pose.position.x;
        double dy = goal_1.pose.position.y - goal_point.pose.position.y;
        double dz = goal_1.pose.position.z - goal_point.pose.position.z;
        if(std::sqrt(dx * dx + dy * dy) > 1)
        {  //表示以及识别出第二个框
          goal_2        = goal_point;
          goal_2.pose.position.x += 0.5;
          is_rec_frame2 = true;

          ROS_INFO("goal2: x:%f,y:%f,z:%f",goal_2.pose.position.x,goal_2.pose.position.y,goal_2.pose.position.z);
          ROS_ERROR("REPEAT SEND POSE");
          ROS_WARN("goal_x_t: %f", rflysim_p.goal_x_t);
          goal_pub.publish(goal_2);
          ros::spinOnce();
          send_goal_flag = 2;
          goal_flag      = 2;
        }
      }
    }
    return;
  }

  sensor_msgs::Image depth;
  //这里需要做坐标转换，从RGB像素到深度图像里面的像素
  {  //需要这么几个矩阵，两个相机的畸变矫正矩阵，RGB相机到深度相机的变换矩阵，因为仿真里面没有畸变，也不产生旋转与平移，那么可以直接计算。
    if(depth_queue.empty())
    {
      ROS_WARN("current depth queue is empty!");
      return;
    }
    //找与图像目标检测时间最近的深度图
    bool                         is_find_data = false;
    std::unique_lock<std::mutex> lock(depth_mtx);
    while(depth_queue.size() > 0)
    {
      auto   front = depth_queue.front();
      double dt    = front.header.stamp.toSec() - objs->header.stamp.toSec();
      if(std::abs(dt) < 0.1)
      {
        depth = front;
        depth_queue.pop();
        is_find_data = true;
        break;
      }
      else
      {
        ROS_WARN("depth_queue size: %d, %f ", depth_queue.size(), dt);
        depth_queue.pop();
      }
      if(depth_queue.empty())
      {

        // ROS_WARN("not find the stamp recentest depth image");
        // sleep(0.1); // 睡眠一个时间，让深度图赋值
        return;
      }
    }
    lock.unlock();
    if(is_find_data == false)
    {
      return;
    }
  }
  if(mission == Mission::cross_frame1 && is_rec_frame1)
    return;
  if(mission == Mission::cross_frame2 && is_rec_frame2)
    return;
  std::cout << " find recent depth image: " << depth.width << ", "
            << depth.height << std::endl;

  if(rflysim_p.enable)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
    static auto scale = rflysim_p.f_depth / rflysim_p.f_rgb;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr =
        cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch(cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat depth_img = cv_ptr->image;
    //    cv::imshow("depth", depth_img);
    //    cv::waitKey(0);
    for(size_t i = 0; i < objs->objects.size(); ++i)
    {
      if(objs->objects[i].score < rflysim_p.min_score)
      {  //置信度低的不做考虑，按原指令执行
        continue;
      }
      //截取深度图部分点云
      std::cout << "det left_top: " << objs->objects[i].left_top_x << ", "
                << objs->objects[i].left_top_y << std::endl;
      std::cout << "det right_bottom: " << objs->objects[i].right_bottom_x
                << ", " << objs->objects[i].right_bottom_y << std::endl;
      std::cout << "rgb_cnt: " << rflysim_p.rgb_cnt.x << ","
                << rflysim_p.rgb_cnt.y << std::endl;
      std::cout << "depth_cnt: " << rflysim_p.depth_cnt.x << ","
                << rflysim_p.depth_cnt.y << std::endl;

      int dx_left  = objs->objects[i].left_top_x - rflysim_p.rgb_cnt.x;
      int dy_left  = objs->objects[i].left_top_y - rflysim_p.rgb_cnt.y;
      int dx_right = objs->objects[i].right_bottom_x - rflysim_p.rgb_cnt.x;
      int dy_right = objs->objects[i].right_bottom_y - rflysim_p.rgb_cnt.y;

      cv::Point2i left;
      cv::Point2i right;
      left.x  = int(scale * dx_left + rflysim_p.depth_cnt.x);
      left.y  = int(scale * dy_left + rflysim_p.depth_cnt.y);
      right.x = int(scale * dx_right + rflysim_p.depth_cnt.x);
      right.y = int(scale * dy_right + rflysim_p.depth_cnt.y);
      std::cout << " lddeft: " << left << " , right: " << right << std::endl;
      if(left.x < 0 || left.x >= depth_img.cols || left.y < 0
         || left.y >= depth_img.rows || right.x < 0 || right.x >= depth_img.cols
         || right.y < 0 || right.y >= depth_img.cols)
      {
        ROS_ERROR("please check params is correct");
        return;
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(
        new pcl::PointCloud<pcl::PointXYZ>());

      cv::Mat frame = depth_img(cv::Rect(left, right)).clone();
      DepthImgToCloud(tmp, &frame, left, right);
      cv::Point3d frame_cnt;
      if(Cluster(tmp, &frame_cnt))
      {  //当前仅仅是传感器坐标系的坐标标，应该转换成世界坐标系下的坐标，然后往规划器里面发送
        //具体操作如下，1.把传感器坐标系的坐标转到机体坐标系,2.通过飞机位姿获得旋转与平移,将机体坐标系下的目标点位置转换成世界坐标系的位置
        //对这个点进行坐标转；
        //获取飞机的在全局坐标系下的位姿
        pcl::PointXYZ cnt;
        cnt.x = frame_cnt.x;
        cnt.y = frame_cnt.y;
        cnt.z = frame_cnt.z;
        cnt.z += rflysim_p.goal_x_t;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(
        new pcl::PointCloud<pcl::PointXYZ>());
      //因为直接使用的检测框作为目标框，得加上一个往前的偏移量
        ROS_INFO("cnt.z:%f goal_x_t:%f",cnt.z,rflysim_p.goal_x_t);
        tmp->points.push_back(cnt);
        CoordinateTrans(tmp);

        goal_point.pose.position.x = tmp->points[0].x;
        goal_point.pose.position.y = tmp->points[0].y;
        goal_point.pose.position.z = tmp->points[0].z;
        if(mission == Mission::cross_frame1)
        {
          goal_flag = 1;
          if(send_goal_flag == 0)
          {
            goal_1        = goal_point;
            is_rec_frame1 = true;
            goal_pub.publish(goal_1);
            ros::spinOnce();
            send_goal_flag = 1;
          }
        }
        else if(mission == Mission::cross_frame2)
        {
          if(send_goal_flag == 1)
          {
            double dx = goal_1.pose.position.x - goal_point.pose.position.x;
            double dy = goal_1.pose.position.y - goal_point.pose.position.y;
            double dz = goal_1.pose.position.z - goal_point.pose.position.z;
            if(std::sqrt(dx * dx + dy * dy) > 1)
            {  //表示以及识别出第二个框
              goal_2        = goal_point;
              is_rec_frame2 = true;
              goal_pub.publish(goal_2);
              ros::spinOnce();
              send_goal_flag = 2;
              goal_flag      = 2;
            }
          }
        }
      }
      else
      {
        ROS_ERROR("cluser fail");
      }
    }
  }
  std::cout << ">>>>>>>>frame position in world: " << goal_point.pose.position.x
            << "," << goal_point.pose.position.y << ","
            << goal_point.pose.position.z << std::endl;
}

void Controller::ObjConterRGB(const common_msgs::Objects::ConstPtr objs,
                              pcl::PointXYZ *                      cnt)
{  //该接口目标中心计算比较粗糙，直接使用目标检测框（这就要求标注的时候尽可能的贴着边缘标定），严格上来讲，应该对目标款内图像做角点检测，然后筛选出目标的实际目标点位置

  static double scale_ =
    rflysim_p.rgb_image_width / rflysim_p.depth_image_height;

  if(objs->objects.empty())
    return;

  for(int i = 0; i < objs->objects.size(); ++i)
  {  //可能会检测出多个目标，刷选的方方法有的有很多，例如通过score
     //过滤,，然后通过框的相对位置，比如第一个框一定在第二个框的右边等等
    auto &obj = objs->objects[i];
    if(obj.score < rflysim_p.min_score)
      continue;

    auto width  = obj.right_bottom_x - obj.left_top_x;
    auto height = obj.right_bottom_y - obj.left_top_y;

    auto ret = width / height;
    if(ret - scale_ > 0.5)
    {
      //如果比大很多，那可以判断这个目标不是我们需要的目标；
      continue;
    }

    //计算传感器的宽度和高度，unit:mm
    static double s_w = 2 * rflysim_p.f_rgb * std::tan(rflysim_p.rgb_fov_h / 2);
    static double s_h = 2 * rflysim_p.f_rgb * std::tan(rflysim_p.rgb_fov_v / 2);
    static double pw  = s_w / rflysim_p.rgb_image_width;   //像素的宽度
    static double ph  = s_h / rflysim_p.rgb_image_height;  //像素的高度
    static double wf = 1.3 * rflysim_p.f_rgb;

    //目标宽，与高比值1：1，1.3m, 默认目标框的高度是没有遮挡的，
    //先还原如果目标框没有被遮挡，应该在图像上的什么位置；
    int left_or_right = (obj.left_top_x + obj.right_bottom_x)/2 - rflysim_p.rgb_cnt.x;
    int det_x = 0;
    if(ret < scale_)
    {
      det_x = int(height * scale_ - width);
      // if(obj.left_top_x - det_x < 0)
      // {  // 目标根据场景布置，目标只有可能被左边的柱子遮挡，如果场景移动了，那就是右边,
      //   //所以这种情况是月边界不再图像内
      //   continue;
      // }
       
      if(rflysim_p.is_S)
        ROS_INFO("l_or_r: %d",left_or_right);
      if(!rflysim_p.is_S && mission == Mission::cross_frame1 && left_or_right > 0 ||
        !rflysim_p.is_S && mission == Mission::cross_frame2 && left_or_right < 0 || 
        rflysim_p.is_S && mission == Mission::cross_frame1 && left_or_right < 0 || 
        rflysim_p.is_S && mission == Mission::cross_frame2 && left_or_right > 0
      )
      { // 通过当前飞机正在执行的任务与飞机的轨迹形状，再接目标检测的结果判断，该目标是否有效
        continue;  
      }
        width +=  det_x;

    }
    //根据焦距计算相机坐标系的宽中心点位置（x,y,z）
    double        z  = wf / width;
    
    //需要考虑遮挡情况
    double cx = (obj.right_bottom_x + (obj.left_top_x - det_x)) / 2;
    double cy = (obj.right_bottom_y + obj.left_top_y) / 2;
    if(!rflysim_p.is_S && mission == Mission::cross_frame2 || rflysim_p.is_S && mission == Mission::cross_frame1)
    {
      cx = ((obj.right_bottom_x + det_x) + obj.left_top_x) / 2;
    }
     
    double xc = (cx - rflysim_p.rgb_cnt.x) * pw;
    double yc = (cy - rflysim_p.rgb_cnt.y) * ph;
    double x  = xc * z / rflysim_p.f_rgb;
    double y  = yc * z / rflysim_p.f_rgb;
    //至此，一致计算得到目标宽中心位置在相机坐标系里的位置了；

    cnt->x = x;
    cnt->y = y;
    cnt->z = z;

    //    pcl::PointXYZ p;
    //    p.x = x;
    //    p.y = y;
    //    p.z = z;
    //    VisionPointCloud(&p, vis_pub);  // 结合深度图像看看位置是否准确
  }
}

void Controller::RecvTra(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  //使用直接赋值，默认认为给飞控的odom数据与给ego-planner的数据是同一坐标系的

  if(send_goal_flag == 1 && mission == Mission::cross_frame1)
  {
    is_recv_tra = 1;
  }
  if(send_goal_flag == 2 && mission == Mission::cross_frame2)
  {
    is_recv_tra = 2;
  }
  cmd.position.x   = msg->position.x;
  cmd.position.y   = msg->position.y;
  cmd.position.z   = msg->position.z;
  cmd.yaw          = takeoff_yaw;
  cmd.header.stamp = msg->header.stamp;
}

void Controller::RecvLIO(const nav_msgs::Odometry::ConstPtr &odom)
{
  send_local_pose.pose.position.x = odom->pose.pose.position.y;
  send_local_pose.pose.position.y = -odom->pose.pose.position.x;
  send_local_pose.pose.position.z = odom->pose.pose.position.z;

  tf2::Quaternion q;
  tf2::fromMsg(odom->pose.pose.orientation, q);
  tf2::Matrix3x3 att(q);
  double         roll, pitch, yaw;
  att.getRPY(roll, pitch, yaw);
  yaw += 1.5707;
  tf2::Quaternion q_;
  q_.setRPY(roll, pitch, yaw);

  // send_local_pose.pose.orientation = odom->pose.pose.orientation;
  send_local_pose.pose.orientation = tf2::toMsg(q_);

  send_local_pose.header.frame_id = "map";
  send_local_pose.header.stamp    = ros::Time::now();

  pose_pub.publish(send_local_pose);
  ros::spinOnce();
  ;
}

void Controller::RecvAruco(const common_msgs::Aruco::ConstPtr &msg)
{
  if(mission == Mission::recognize_aruco)
    is_aruco = true;
  aruco = *msg;
}

void Controller::StateChange()
{

  // ROS_INFO("fcu_p
  // %f,%f,%f",fcu_pose.pose.position.x,fcu_pose.pose.position.x,fcu_pose.pose.position.x);
  if(mission == Mission::cross_frame1)
  {  //收到到fram1的轨迹，且离目标点很近了
    double dx = fcu_pose.pose.position.x - goal_1.pose.position.x;
    double dy = fcu_pose.pose.position.y - goal_1.pose.position.y;
    double dz = fcu_pose.pose.position.z - goal_1.pose.position.z;
    if(std::abs(dx) < 0.4 && std::abs(dy) < 0.4 && std::abs(dz) < 0.4)
    {
      ROS_INFO("to cross fram2");
      mission = Mission::cross_frame2;
    }
    ROS_WARN("distance :%f, %f, %f", dx, dy, dz);
    ROS_INFO("fcu_p %f,%f,%f", fcu_pose.pose.position.x,
             fcu_pose.pose.position.y, fcu_pose.pose.position.z);
    ROS_INFO("goal_1  %f,%f,%f", goal_1.pose.position.x, goal_1.pose.position.y,
             goal_1.pose.position.y);
  }
  else if(mission == Mission::cross_frame2)
  {  //收到到fram2的轨迹，且离目标点很近了
    double dx = fcu_pose.pose.position.x - goal_2.pose.position.x;
    double dy = fcu_pose.pose.position.y - goal_2.pose.position.y;
    double dz = fcu_pose.pose.position.z - goal_2.pose.position.z;
    ROS_WARN("distance :%f, %f, %f", dx, dy, dz);
    ROS_INFO("fcu_p %f,%f,%f", fcu_pose.pose.position.x,
             fcu_pose.pose.position.y, fcu_pose.pose.position.z);
    ROS_INFO("goal_2 %f,%f,%f", goal_2.pose.position.x, goal_2.pose.position.y,
             goal_2.pose.position.z);
    if(std::abs(dx) < 0.2 && std::abs(dy) < 0.2 && std::abs(dz) < 0.2)
    {
      ROS_INFO("to recognize_aruco");
      mission = Mission::recognize_aruco;
    }
  }
  else if(mission == Mission::recognize_aruco && is_aruco)
  {
    mission = Mission::recognize_H;
    //进入视觉伺服控制
  }
  else if(mission == Mission::recognize_H && is_H)
  {
    mission = Mission::land;
  }

  common_msgs::MissionState state;
  state.header.stamp = ros::Time::now();
  state.task         = uint8_t(mission);
  task_state_pub.publish(state);
  ros::spinOnce();

  PrintMission();
}

void Controller::DepthImgToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 const cv::Mat *img, const cv::Point2i &left,
                                 const cv::Point2i &right)
{
  //我们需要获得相机坐标系下的点云坐标，等计算中心点，只需要转换一个点坐标即可
  cloud->is_dense = false;
  //  cloud->points.resize(cloud->width * cloud->height);
  //  cv::imshow("depth_frame", *img);
  //  cv::waitKey(0);
  int           dx = left.x - rflysim_p.depth_cnt.x;
  int           dy = left.y - rflysim_p.depth_cnt.y;
  pcl::PointXYZ point;
  for(int row = 0; row < img->rows; row += rflysim_p.depth_down_sample)
  {
    for(int col = 0; col < img->cols; col += rflysim_p.depth_down_sample)
    {
      float depth =
        img->at<uint16_t>(row, col) * 0.001f;  // rflysim 精度为0.001
      if(depth > 0 && depth < 7)  //目标不能在7米外，减少计算量
      {
        point.z = depth;
        point.x = (col + dx) * depth / rflysim_p.f_depth;
        point.y = (row + dy) * depth / rflysim_p.f_depth;

        cloud->points.push_back(point);
      }
    };
  }
}

bool Controller::Cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                         cv::Point3d *                       cnt)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
    new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices>                 cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.4);  // 设置聚类的欧几里得距离阈值为 50cm
  ec.setMinClusterSize(10);     // 设置一个聚类需要的最小点数
  ec.setMaxClusterSize(10000);  // 设置一个聚类需要的最大点数
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  // 提取聚类的索引（以点云簇的形式返回）
  ec.extract(cluster_indices);
  double obs_dist = FLT_MAX;
  std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;
  // 输出每个聚类的点数和中心点
  pcl::PointCloud<pcl::PointXYZ>::Ptr ret_pc(
    new pcl::PointCloud<pcl::PointXYZ>);
  for(std::vector<pcl::PointIndices>::const_iterator it =
        cluster_indices.begin();
      it != cluster_indices.end(); ++it)
  {  //我们需要对目标应该是点最多的，还可以进一步求出聚类点角点,然后求里飞机最近的目标点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(
      new pcl::PointCloud<pcl::PointXYZ>);
    auto min_x = FLT_MAX;
    auto max_x = FLT_MIN;
    auto min_y = FLT_MAX;
    auto max_y = FLT_MIN;
    for(std::vector<int>::const_iterator pit = it->indices.begin();
        pit != it->indices.end(); ++pit)
    {
      if(min_x > cloud->points[*pit].x)
        min_x = cloud->points[*pit].x;
      if(max_x < cloud->points[*pit].x)
        max_x = cloud->points[*pit].x;
      if(min_y > cloud->points[*pit].y)
        min_y = cloud->points[*pit].y;
      if(max_y < cloud->points[*pit].y)
        max_y = cloud->points[*pit].y;
      cluster->points.push_back(
        cloud->points[*pit]);  // 将聚类中的点添加到点云中
    }

    auto width  = max_x - min_x;
    auto height = max_y - min_y;
    //    std::cout << " cloud w: " << width << " , h:" << height << std::endl;
    //    if(abs(width - 1.3) > 0.5 || abs(height - 1.3) > 0.5)
    //    {  //计算框的宽和高，以此过滤不符合要求的目标
    //      continue;
    //    }

    cluster->width    = cluster->points.size() + 1;
    cluster->height   = 1;
    cluster->is_dense = true;

    std::cout << "Cluster size: " << cluster->size() << std::endl;
    //    std::cout << "Cluster center: " << std::endl;
    Eigen::Vector4f centroid;
    auto            ret = pcl::compute3DCentroid(*cluster, centroid);
    if(ret == 0)
    {
      continue;
    }
    //    std::cout << "x: " << centroid[0] << ", y: " << centroid[1]
    //              << ", z: " << centroid[2] << std::endl;

    if(centroid[2] < obs_dist)
    {  //最后，距离最近的目标将被选出
      obs_dist = centroid[2];
      cnt->x   = centroid[0];
      cnt->y   = centroid[1];
      cnt->z   = centroid[2];
      pcl::PointXYZ p;
      p.x = cnt->x;
      p.y = cnt->y;
      p.z = cnt->z;
      cluster->points.push_back(p);
      ret_pc->points.swap(cluster->points);
    }
    //    VisionPointCloud(cluster);
    // ret_pc->points.insert(ret_pc->points.begin() +
    // ret_pc->points.size(),cluster->points.begin(),cluster->points.end());
  }
  //  VisionPointCloud(ret_pc);
  cloud->points.swap(ret_pc->points);
  if(obs_dist > 9)
  {
    ROS_ERROR("not cluster objects");
    return false;
  }
  return true;
}

void Controller::CloudKeyPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // 估计法向量
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
    new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
    new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.03);
  ne.compute(*cloud_normals);

  // 计算 ISS 角点
  pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
  iss.setInputCloud(cloud);
  iss.setNormals(cloud_normals);
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(
    new pcl::PointCloud<pcl::PointXYZ>);
  iss.setSalientRadius(0.05);  // 设置关键点的最小和最大尺度
  iss.setNonMaxRadius(0.05);   // 设置非极大值抑制的搜索半径
  iss.compute(*keypoints);

  // 输出角点数量
  std::cout << "Number of keypoints: " << keypoints->size() << std::endl;
  ;
}

void Controller::PrintMission()
{
  switch(mission)
  {
    case Mission::init:
      ROS_INFO("init");
      break;
    case Mission::takeoff:
      ROS_INFO("takeoff");
      break;
    case Mission::cross_corridor:
      ROS_INFO("cross_corridor");
      break;
    case Mission::cross_frame1:
      ROS_INFO("cross_frame1");
      break;
    case Mission::cross_frame2:
      ROS_INFO("cross_frame2");
      break;
    case Mission::recognize_aruco:
      ROS_INFO("recognize_aruco");
      break;
    case Mission::recognize_H:
      ROS_INFO("recognize_H");
      break;
    case Mission::land:
      ROS_INFO("land");
      break;
    case Mission::end:
      ROS_INFO("end");
      break;
  }
}

void Controller::VisionPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                  const ros::Publisher &              pub)
{
  sensor_msgs::PointCloud2 data;
  pcl::toROSMsg(*cloud, data);
  data.header.frame_id = "map";
  pub.publish(data);
  ros::spinOnce();
}
void Controller::VisionPointCloud(const pcl::PointXYZ * p,
                                  const ros::Publisher &pub)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZ>());
  cloud->points.push_back(*p);
  VisionPointCloud(cloud, pub);
}

bool Controller::RePlanReq()
{
  //有时候因为遮挡的问题，随着飞机靠近，遮挡解除，目标点中心位置也会发送变化，如果当前目标点无法通过，这时候需要重新发送目标点，

  if(mission == Mission::cross_frame1)
  {
    double dx = goal_1.pose.position.x - goal_point.pose.position.x;
    double dy = goal_1.pose.position.y - goal_point.pose.position.y;
    double dz = goal_1.pose.position.z - goal_point.pose.position.z;
    if(std::abs(dx) > 0.5 || std::abs(dy) > 0.5 || std::abs(dz) > 0.5)
    {  //此时需要重新规划路径
      goal_1 = goal_point;
      goal_pub.publish(goal_1);
    }
  }
  else if(mission == Mission::cross_frame2)
  {
    double dx = goal_2.pose.position.x - goal_point.pose.position.x;
    double dy = goal_2.pose.position.y - goal_point.pose.position.y;
    double dz = goal_2.pose.position.z - goal_point.pose.position.z;
    if(std::abs(dx) > 0.5 || std::abs(dy) > 0.5 || std::abs(dz) > 0.5)
    {  //此时需要重新规划路径

      goal_2 = goal_point;
      goal_pub.publish(goal_2);
    }
  }
  ros::spinOnce();
  return true;
}

void Controller::CoordinateTrans(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  std::vector<double> R = rflysim_p.depth_cam2body_R;
  std::vector<double> T = rflysim_p.depth_cam2body_T;
  //        tf2::Vector3        bc(x, y, z);
  tf2::Quaternion q;
  tf2::convert(fcu_pose.pose.orientation, q);
  tf2::Vector3 t;
  tf2::convert(fcu_pose.pose.position, t);

  tf2::Transform trans;
  trans.setOrigin(t);
  trans.setRotation(q);

  for(size_t i = 0; i < cloud->points.size(); ++i)
  {
    auto         p = cloud->points[i];
    double       x = p.x * R[0] + p.y * R[1] + p.z * R[2] + T[0];
    double       y = p.x * R[3] + p.y * R[4] + p.z * R[5] + T[1];
    double       z = p.x * R[6] + p.y * R[7] + p.z * R[8] + T[2];
    tf2::Vector3 b_p(x, y, z);

    tf2::Vector3 w_p = trans * b_p;
    cloud->points[i].x = w_p.getX();
    cloud->points[i].y = w_p.getY();
    cloud->points[i].z = w_p.getZ();
  }
  //  std::cout << "xxxxxx: " << tmp->points.size() << std::endl;
  ;
}
