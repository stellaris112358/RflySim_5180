#include "ros/ros.h"
#include "std_msgs/String.h"
#include "controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "contorller");
  ros::NodeHandle nh;
  auto            ctrl = new Controller(nh);
  ros::spin();

  delete ctrl;
  return 0;
}
