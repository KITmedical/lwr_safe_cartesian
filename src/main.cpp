#include "LwrSafeCartesian.h"
#include <getopt.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "lwr_safe_cartesian");

  std::string robotName = "lwr";
  std::string setJointTopic = "lwr/set_joint";
  std::string getJointTopic = "lwr/get_joint";
  std::string setCartesianTopic = "lwr/set_cartesian";
  std::string getCartesianTopic = "lwr/get_cartesian";
  std::string stateTopic = "lwr/state";

  std::string directSetJointTopic = "lwr/direct/set_joint";
  std::string directGetJointTopic = "lwr/direct/get_joint";
  std::string directStateTopic = "lwr/direct/state";

  const char optstring[] = "";
  struct option longopts[] = {
    { "robotname", required_argument, NULL, 0 },
  };
  int opt;
  int optindex;
  while ((opt = getopt_long(argc, argv, optstring, longopts, &optindex)) != -1) {
    switch (opt) {
    case 0:
      if (strcmp(longopts[optindex].name, "robotname") == 0) {
        robotName = optarg;
      }
      break;
    }
  }

  LwrSafeCartesian lwrSafeCartesian(robotName);

  std::cout << "Spinning" << std::endl;
  ros::spin();

  return 0;
}
