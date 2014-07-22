#include "LwrSafeCartesian.h"

// system includes

// library includes
#include <tf/transform_datatypes.h>

// custom includes
#include <ahbstring.h>
#include <safe_cartesian_moveit/CollisionCheckMoveIt.hpp>
#include <LwrLibrary.hpp>


/*---------------------------------- public: -----------------------------{{{-*/
LwrSafeCartesian::LwrSafeCartesian(const std::string& p_robotName, const std::string& p_setJointTopic, const std::string& p_getJointTopic, const std::string& p_setCartesianTopic, const std::string& p_getCartesianTopic, const std::string& p_stateTopic, const std::string& p_directSetJointTopic, const std::string& p_directGetJointTopic, const std::string& p_directStateTopic)
  :m_robotName(p_robotName),
   m_setJointTopic(p_setJointTopic),
   m_getJointTopic(p_getJointTopic),
   m_setCartesianTopic(p_setCartesianTopic),
   m_getCartesianTopic(p_getCartesianTopic),
   m_stateTopic(p_stateTopic),
   m_directSetJointTopic(p_directSetJointTopic),
   m_directGetJointTopic(p_directGetJointTopic),
   m_directStateTopic(p_directStateTopic)
{
  // ros
  m_lastJointState.position.resize(LWR_JOINTS, 0);
  m_lastJointState.velocity.resize(LWR_JOINTS, 0);
  m_lastJointState.effort.resize(LWR_JOINTS, 0);
  m_targetJointState.position.resize(LWR_JOINTS, 0);
  m_targetJointState.velocity.resize(LWR_JOINTS, 0);
  m_targetJointState.effort.resize(LWR_JOINTS, 0);

  m_jointNames.resize(LWR_JOINTS);
  for (size_t jointIdx = 0; jointIdx < LWR_JOINTS; jointIdx++) {
    m_jointNames[jointIdx] = m_robotName + "_" + Lwr::jointNames[jointIdx] + "_joint";
  }

  m_setJointTopicSub = m_node.subscribe<sensor_msgs::JointState>(m_setJointTopic, 1, &LwrSafeCartesian::setJointCallback, this);
  m_getJointTopicPub = m_node.advertise<sensor_msgs::JointState>(m_getJointTopic, 1);
  m_setCartesianTopicSub = m_node.subscribe<geometry_msgs::Pose>(m_setCartesianTopic, 1, &LwrSafeCartesian::setCartesianCallback, this);
  m_getCartesianTopicPub = m_node.advertise<geometry_msgs::Pose>(m_getCartesianTopic, 1);
  m_stateTopicPub = m_node.advertise<std_msgs::String>(m_stateTopic, 1);

  m_directGetJointTopicSub = m_node.subscribe<sensor_msgs::JointState>(m_directGetJointTopic, 1, &LwrSafeCartesian::directGetJointCallback, this);
  m_directSetJointTopicPub = m_node.advertise<sensor_msgs::JointState>(m_directSetJointTopic, 1);
  m_directStateTopicSub = m_node.subscribe<std_msgs::String>(m_directStateTopic, 1, &LwrSafeCartesian::directStateCallback, this);

  m_collision_check = new CollisionCheckMoveIt();
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
LwrSafeCartesian::setJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg)
{
  //std::cout << "setJointCallback: jointsMsg=\n" << *jointsMsg << std::endl;

  if (jointsMsg->position.size() != LWR_JOINTS) {
    ROS_FATAL_STREAM("Wrong number of joints. Will not move robot at all.\n");
    m_currentState.data = "SAFE_LWR_ERROR|SAFE_LWR_WRONG_NUMBER_OF_JOINTS";
    m_stateTopicPub.publish(m_currentState);
    return;
  }

  for (size_t jointIdx = 0; jointIdx < LWR_JOINTS; jointIdx++) {
    if (std::abs(jointsMsg->position[jointIdx]) > Lwr::jointLimits.j[jointIdx]) {
      ROS_FATAL_STREAM("Joint" << jointIdx << " beyond joint limit (" << Lwr::jointLimits.j[jointIdx] << "). Will not move robot at all.\n");
      m_currentState.data = "SAFE_LWR_ERROR|SAFE_LWR_JOINT_LIMIT_EXCEEDED";
      m_stateTopicPub.publish(m_currentState);
      return;
    }
  }

  if (m_collision_check->hasCollision(*jointsMsg)) {
    std::cout << "------------------> COLLISION <---------------" << std::endl;
    return;
  }

  m_targetJointState = *jointsMsg;
  publishToHardware();
}

void
LwrSafeCartesian::setCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg)
{
  //std::cout << "setCartesianCallback: poseMsg=\n" << *poseMsg << std::endl;

  tf::Pose tfpose;
  tf::poseMsgToTF(*poseMsg, tfpose);
  tf::Vector3 tforigin = tfpose.getOrigin();
  tf::Quaternion tforientation = tfpose.getRotation();
  LwrXCart cartXPose;
  cartXPose.nsparam = 0;
  cartXPose.config = 2;
  cartXPose.pose.setPos(tforigin.x(), tforigin.y(), tforigin.z());
  double x, y, z, w;
  w = tforientation.w();
  x = tforientation.x();
  y = tforientation.y();
  z = tforientation.z();
  cartXPose.pose.setQuat(w, x, y, z);
  LwrJoints joints;
  LwrErrorMsg kinematicReturn;
  kinematicReturn = Lwr::inverseKinematics(joints, cartXPose);

  m_currentState.data = Lwr::errorToString(kinematicReturn);
  m_stateTopicPub.publish(m_currentState);
  
  //printf("kinematicReturn=0x%02X=%s\n", kinematicReturn, Lwr::errorToString(kinematicReturn).c_str());
  if (kinematicReturn != LWR_OK && kinematicReturn != (LWR_WARNING | LWR_CLOSE_TO_SINGULARITY)) {
    ROS_WARN_STREAM("Lwr::inverseKinematics() failed: " << kinematicReturn);
    return;
  }

  sensor_msgs::JointState unsafeTargetJointState(m_targetJointState);
  for (size_t jointIdx = 0; jointIdx < m_targetJointState.position.size(); jointIdx++) {
    unsafeTargetJointState.position[jointIdx] = joints.j[jointIdx];
  }
  //std::cout << "m_targetJointState: " << m_targetJointState << std::endl;

  if (m_collision_check->hasCollision(unsafeTargetJointState)) {
    std::cout << "------------------> COLLISION <---------------" << std::endl;
    return;
  }

  m_targetJointState = unsafeTargetJointState;
  publishToHardware();
}

void
LwrSafeCartesian::directGetJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg)
{
  //std::cout << "directGetJointCallback: jointsMsg=\n" << *jointsMsg << std::endl;
 
  m_lastJointState = *jointsMsg;
  m_lastJointState.name = m_jointNames;

  LwrXCart cartXPose;
  LwrJoints joints;
  joints.setJoints(m_lastJointState.position);
  LwrErrorMsg kinematicReturn;
  kinematicReturn = Lwr::forwardKinematics(cartXPose, joints);
  if (kinematicReturn != LWR_OK && kinematicReturn != (LWR_WARNING | LWR_SINGULARITY)) {
    ROS_WARN_STREAM("Lwr::forwardKinematics() failed: " << kinematicReturn);
    return;
  }
  cartXPose.pose.getPos(m_lastCartesianPose.position.x, m_lastCartesianPose.position.y, m_lastCartesianPose.position.z);
  cartXPose.pose.getQuat(m_lastCartesianPose.orientation.w, m_lastCartesianPose.orientation.x, m_lastCartesianPose.orientation.y, m_lastCartesianPose.orientation.z);

  publishToApplication();
  publishToTF();
}

void
LwrSafeCartesian::directStateCallback(const std_msgs::String::ConstPtr& stateMsg)
{
  std::cout << "directStateCallback: state=\n" << *stateMsg << std::endl;
}

void
LwrSafeCartesian::publishToHardware()
{
  m_targetJointState.header.stamp = ros::Time::now();
  m_directSetJointTopicPub.publish(m_targetJointState);
}

void
LwrSafeCartesian::publishToApplication()
{
  m_lastJointState.header.stamp = ros::Time::now();
  m_getJointTopicPub.publish(m_lastJointState);
  m_getCartesianTopicPub.publish(m_lastCartesianPose);
}

void
LwrSafeCartesian::publishToTF()
{
}
/*------------------------------------------------------------------------}}}-*/
