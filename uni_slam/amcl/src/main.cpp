#include <amcl.hpp>

boost::shared_ptr<AmclNode> amcl_node_ptr;

void sigintHandler(int sig)
{
  // Save latest pose as we're shutting down.
  amcl_node_ptr->savePoseToServer();
  ros::shutdown();
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl");
  ros::NodeHandle nh;

  // Override default sigint handler
  signal(SIGINT, sigintHandler);

  // Make our node available to sigintHandler
  amcl_node_ptr.reset(new AmclNode());

  if (argc == 1)
  {
    // run using ROS input
    ros::spin();
  }
  else if ((argc >= 3) && (std::string(argv[1]) == "--run-from-bag"))
  {
    if (argc == 3)
    {
      amcl_node_ptr->runFromBag(argv[2]);
    }
    else if ((argc == 4) && (std::string(argv[3]) == "--global-localization"))
    {
      amcl_node_ptr->runFromBag(argv[2], true);
    }
  }

  // Without this, our boost locks are not shut down nicely
  amcl_node_ptr.reset();

  // To quote Morgan, Hooray!
  return(0);
}