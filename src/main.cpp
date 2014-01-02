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
    { "setjointtopic", required_argument, NULL, 0 },
    { "getjointtopic", required_argument, NULL, 0 },
    { "setcartesiantopic", required_argument, NULL, 0 },
    { "getcartesiantopic", required_argument, NULL, 0 },
    { "statetopic", required_argument, NULL, 0 },
    { "directsetjointtopic", required_argument, NULL, 0 },
    { "directgetjointtopic", required_argument, NULL, 0 },
    { "directstatetopic", required_argument, NULL, 0 },
  };
  int opt;
  int optindex;
  while ((opt = getopt_long(argc, argv, optstring, longopts, &optindex)) != -1) {
    switch (opt) {
    case 0:
      if (strcmp(longopts[optindex].name, "robotname") == 0) {
        robotName = optarg;
      } else if (strcmp(longopts[optindex].name, "setjointtopic") == 0) {
        setJointTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "getjointtopic") == 0) {
        getJointTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "setcartesiantopic") == 0) {
        setCartesianTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "getcartesiantopic") == 0) {
        getCartesianTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "statetopic") == 0) {
        stateTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "directsetjointtopic") == 0) {
        directSetJointTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "directgetjointtopic") == 0) {
        getJointTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "directstatetopic") == 0) {
        directStateTopic = optarg;
      }
      break;
    }
  }

  LwrSafeCartesian lwrSafeCartesian(robotName, setJointTopic, getJointTopic, setCartesianTopic, getCartesianTopic, stateTopic, directSetJointTopic, directGetJointTopic, directStateTopic);

  std::cout << "Spinning" << std::endl;
  ros::spin();

  return 0;
}
