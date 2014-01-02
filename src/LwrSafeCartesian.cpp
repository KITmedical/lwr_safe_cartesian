#include "LwrSafeCartesian.h"

// system includes

// library includes
#include <tf/transform_datatypes.h>

// custom includes
#include <ahbstring.h>
#include <lwr/LwrLibrary.hpp>


/*---------------------------------- public: -----------------------------{{{-*/
LwrSafeCartesian::LwrSafeCartesian(const std::string& p_setJointTopic, const std::string& p_getJointTopic, const std::string& p_setCartesianTopic, const std::string& p_getCartesianTopic, const std::string& p_stateTopic, const std::string& p_directSetJointTopic, const std::string& p_directGetJointTopic, const std::string& p_directStateTopic)
  :m_setJointTopic(p_setJointTopic),
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

  m_setJointTopicSub = m_node.subscribe<sensor_msgs::JointState>(m_setJointTopic, 1, &LwrSafeCartesian::setJointCallback, this);
  m_getJointTopicPub = m_node.advertise<sensor_msgs::JointState>(m_getJointTopic, 1);
  m_setCartesianTopicSub = m_node.subscribe<geometry_msgs::Pose>(m_setCartesianTopic, 1, &LwrSafeCartesian::setCartesianCallback, this);
  m_getCartesianTopicPub = m_node.advertise<geometry_msgs::Pose>(m_getCartesianTopic, 1);
  m_stateTopicPub = m_node.advertise<std_msgs::String>(m_stateTopic, 1);

  m_directGetJointTopicSub = m_node.subscribe<sensor_msgs::JointState>(m_directGetJointTopic, 1, &LwrSafeCartesian::directGetJointCallback, this);
  m_directSetJointTopicPub = m_node.advertise<sensor_msgs::JointState>(m_directSetJointTopic, 1);
  m_directStateTopicSub = m_node.subscribe<std_msgs::String>(m_directStateTopic, 1, &LwrSafeCartesian::directStateCallback, this);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
LwrSafeCartesian::setJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg)
{
  //std::cout << "setJointCallback: jointsMsg=\n" << *jointsMsg << std::endl;

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
  if (kinematicReturn != LWR_OK && kinematicReturn != (LWR_WARNING | LWR_CLOSE_TO_SINGULARITY)) {
    ROS_WARN_STREAM("Lwr::inverseKinematics() failed: " << kinematicReturn);
    return;
  }
  for (size_t jointIdx = 0; jointIdx < m_targetJointState.position.size(); jointIdx++) {
    m_targetJointState.position[jointIdx] = joints.j[jointIdx];
  }

  publishToHardware();
}

void
LwrSafeCartesian::directGetJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg)
{
  //std::cout << "directGetJointCallback: jointsMsg=\n" << *jointsMsg << std::endl;
 
  m_lastJointState = *jointsMsg;

  LwrXCart cartXPose;
  LwrJoints joints;
  joints.setJoints(m_lastJointState.position);
  LwrErrorMsg kinematicReturn;
  kinematicReturn = Lwr::forwardKinematics(cartXPose, joints);
  if (kinematicReturn != LWR_OK && kinematicReturn != (LWR_WARNING | LWR_SINGULARITY)) {
    ROS_WARN_STREAM("Lwr::forwardKinematics() failed: " << kinematicReturn);
    return;
  }
  cartXPose.pose.getPos(m_lastCartesianState.position.x, m_lastCartesianState.position.y, m_lastCartesianState.position.z);
  cartXPose.pose.getQuat(m_lastCartesianState.orientation.w, m_lastCartesianState.orientation.x, m_lastCartesianState.orientation.y, m_lastCartesianState.orientation.z);

  publishToApplication();
}

void
LwrSafeCartesian::directStateCallback(const std_msgs::String::ConstPtr& stateMsg)
{
  std::cout << "directStateCallback: state=\n" << *stateMsg << std::endl;
}

void
LwrSafeCartesian::publishToHardware()
{
  m_directSetJointTopicPub.publish(m_targetJointState);
}

void
LwrSafeCartesian::publishToApplication()
{
  m_getJointTopicPub.publish(m_lastJointState);
  m_getCartesianTopicPub.publish(m_lastCartesianState);
}
/*------------------------------------------------------------------------}}}-*/
