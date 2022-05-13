#include "pose_server/zed/pose_srv_zed.h"

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
  ros::init(argc, argv, "pose_server");
  ros::NodeHandle nh;

  pose_server::ObjectPose op(&nh);
  signal(SIGINT, mySigintHandler);

  ROS_INFO("Ready to find object pose.");
  ros::spin();

  return 0;
}
