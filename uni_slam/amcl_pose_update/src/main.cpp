#include "amcl_pose_update/pose_update.h"

boost::shared_ptr<PoseUpdate> pose_update_ptr;

void sigintHandler(int sig)
{
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl_pose_update");
  ros::NodeHandle nh;

  // Override default sigint handler
  signal(SIGINT, sigintHandler);

  // Make our node available to sigintHandler
  pose_update_ptr.reset(new PoseUpdate());

  if (argc == 1)
  {
    // run using ROS input
    ros::spin();
  }

  // Without this, our boost locks are not shut down nicely
  pose_update_ptr.reset();

  // To quote Morgan, Hooray!
  return(0);
}