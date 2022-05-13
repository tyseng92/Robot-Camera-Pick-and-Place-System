
#include "pose_server/zivid/depth_srv_zivid.h"

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
  ros::init(argc, argv, "depth_server_zivid");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  depth_server::ObjectDepth od(&nh);
  signal(SIGINT, mySigintHandler);

  ROS_INFO("Ready to find object depth.");
  ros::waitForShutdown();
  //ros::spin();

  return 0;
}