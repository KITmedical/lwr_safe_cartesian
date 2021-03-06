#ifndef _LWR_SAFE_CARTESIAN_H_
#define _LWR_SAFE_CARTESIAN_H_

// system includes

// library includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

// custom includes
#include <GeneralPurposeInterpolator.hpp>
#include <safe_cartesian_msgs/ElbowState.h>
#include <safe_cartesian_msgs/SetElbow.h>

// forward declarations
class CollisionCheckMoveIt;


class LwrSafeCartesian
{
  public:
    // enums

    // typedefs

    // const static member variables
    static const double path_collision_check_dist_threshold = 0.1;
    static const double m_velMax = 0.8;
    static const double m_accelMax = 10.0;
 
    // static utility functions


    // constructors
    LwrSafeCartesian(const std::string& p_robotName);

    // overwritten methods

    // methods

    // variables


  private:
    // methods
    void setJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg);
    void doCartesian(const geometry_msgs::Pose::ConstPtr& poseMsg, bool collision_checking = true);
    void setCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg);
    void setUnsafeCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg);
    void directGetJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg);
    void directStateCallback(const std_msgs::String::ConstPtr& stateMsg);
    bool setElbowCallback(safe_cartesian_msgs::SetElbow::Request& request, safe_cartesian_msgs::SetElbow::Response& response);
    void publishToHardware();
    void publishToApplication();
    void publishToTF();
    bool pathHasCollision(const sensor_msgs::JointState& targetJointState);

    // variables
    std::string m_robotName;

    std::vector<std::string> m_jointNames;

    ros::NodeHandle m_node;
    ros::Subscriber m_setJointTopicSub;
    ros::Publisher m_getJointTopicPub;
    ros::Subscriber m_setCartesianTopicSub;
    ros::Subscriber m_setUnsafeCartesianTopicSub;
    ros::Publisher m_getCartesianTopicPub;
    ros::Publisher m_stateTopicPub;
    ros::Subscriber m_directGetJointTopicSub;
    ros::Publisher m_directSetJointTopicPub;
    ros::Subscriber m_directStateTopicSub;
    ros::ServiceServer m_setElbowService;
    sensor_msgs::JointState m_lastJointState;
    sensor_msgs::JointState m_targetJointState;
    geometry_msgs::Pose m_lastCartesianPose;
    geometry_msgs::Pose m_targetCartesianPose;
    std_msgs::String m_currentState;
    double m_current_nsparam;
    bool m_fixed_nsparam;

    CollisionCheckMoveIt* m_collision_check;
    GeneralPurposeInterpolator m_gpi;
    std::vector<double> m_gpiPosCurrentBuffer;
    std::vector<double> m_gpiPosTargetBuffer;
    std::vector<double> m_gpiPosMinBuffer;
    std::vector<double> m_gpiPosMaxBuffer;
    std::vector<double> m_gpiVelCurrentBuffer;
    std::vector<double> m_gpiVelMaxBuffer;
    std::vector<double> m_gpiAccelMaxBuffer;
};

#endif // _LWR_SAFE_CARTESIAN_H_
