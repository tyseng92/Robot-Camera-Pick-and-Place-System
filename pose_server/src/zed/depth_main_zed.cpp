
#include "pose_server/zed/depth_srv_zed.h"

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  ROS_INFO("Shutting down...");
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_server");
  ros::NodeHandle nh;

  depth_server::ObjectDepth od(&nh);
  signal(SIGINT, mySigintHandler);

  ROS_INFO("Ready to find object depth.");
  ros::spin();

  return 0;
}
