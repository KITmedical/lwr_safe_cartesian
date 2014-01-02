#include "LwrSafeCartesian.h"
#include <getopt.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "lwr_safe_cartesian");

  std::string setJointTopic = "/robots/lwr/set_joint";
  std::string getJointTopic = "/robots/lwr/get_joint";
  std::string setCartesianTopic = "/robots/lwr/set_cartesian";
  std::string getCartesianTopic = "/robots/lwr/get_cartesian";
  std::string stateTopic = "/robots/lwr/state";

  const char optstring[] = "";
  struct option longopts[] = {
    { "setjointtopic", required_argument, NULL, 0 },
    { "getjointtopic", required_argument, NULL, 0 },
    { "setcartesiantopic", required_argument, NULL, 0 },
    { "getcartesiantopic", required_argument, NULL, 0 },
    { "statetopic", required_argument, NULL, 0 },
  };
  int opt;
  int optindex;
  while ((opt = getopt_long(argc, argv, optstring, longopts, &optindex)) != -1) {
    switch (opt) {
    case 0:
      if (strcmp(longopts[optindex].name, "setjointtopic") == 0) {
        setJointTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "getjointtopic") == 0) {
        getJointTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "setcartesiantopic") == 0) {
        setCartesianTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "getcartesiantopic") == 0) {
        getCartesianTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "statetopic") == 0) {
        stateTopic = optarg;
      }
      break;
    }
  }

  LwrSafeCartesian lwrSafeCartesian(setJointTopic, getJointTopic, setCartesianTopic, getCartesianTopic, stateTopic);

  std::cout << "Spinning" << std::endl;
  ros::spin();

  return 0;
}
