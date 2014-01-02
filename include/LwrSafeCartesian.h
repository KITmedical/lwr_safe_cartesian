#ifndef _LWR_SAFE_CARTESIAN_H_
#define _LWR_SAFE_CARTESIAN_H_

// system includes

// library includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

// custom includes

// forward declarations


class LwrSafeCartesian
{
  public:
    // enums

    // typedefs

    // const static member variables
 
    // static utility functions


    // constructors
    LwrSafeCartesian(const std::string& p_setJointTopic, const std::string& p_getJointTopic, const std::string& p_setCartesianTopic, const std::string& p_getCartesianTopic, const std::string& p_stateTopic);

    // overwritten methods

    // methods

    // variables


  private:
    // methods
    void setJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg);
    void setCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg);
    void publish();

    // variables
    std::string m_setJointTopic;
    std::string m_getJointTopic;
    std::string m_setCartesianTopic;
    std::string m_getCartesianTopic;
    std::string m_stateTopic;

    ros::NodeHandle m_node;
    ros::Subscriber m_setJointTopicSub;
    ros::Publisher m_getJointTopicPub;
    ros::Subscriber m_setCartesianTopicSub;
    ros::Publisher m_getCartesianTopicPub;
    ros::Publisher m_stateTopicPub;
    sensor_msgs::JointState m_currentJointState;
    geometry_msgs::Pose m_currentCartesianState;
};

#endif // _LWR_SAFE_CARTESIAN_H_
