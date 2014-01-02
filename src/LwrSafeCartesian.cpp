#include "LwrSafeCartesian.h"

// system includes

// library includes

// custom includes
#include <ahbstring.h>
#include <lwr/LwrLibrary.hpp>


/*---------------------------------- public: -----------------------------{{{-*/
LwrSafeCartesian::LwrSafeCartesian(const std::string& p_setJointTopic, const std::string& p_getJointTopic, const std::string& p_setCartesianTopic, const std::string& p_getCartesianTopic, const std::string& p_stateTopic)
  :m_setJointTopic(p_setJointTopic),
   m_getJointTopic(p_getJointTopic),
   m_setCartesianTopic(p_setCartesianTopic),
   m_getCartesianTopic(p_getCartesianTopic),
   m_stateTopic(p_stateTopic)
{
  // ros
  m_currentJointState.position.resize(LWR_JOINTS, 0);
  m_currentJointState.velocity.resize(LWR_JOINTS, 0);
  m_currentJointState.effort.resize(LWR_JOINTS, 0);

  m_setJointTopicSub = m_node.subscribe<sensor_msgs::JointState>(m_setJointTopic, 1, &LwrSafeCartesian::setJointCallback, this);
  m_getJointTopicPub = m_node.advertise<sensor_msgs::JointState>(m_getJointTopic, 1);
  m_setCartesianTopicSub = m_node.subscribe<geometry_msgs::Pose>(m_setCartesianTopic, 1, &LwrSafeCartesian::setCartesianCallback, this);
  m_getCartesianTopicPub = m_node.advertise<geometry_msgs::Pose>(m_getCartesianTopic, 1);
  m_stateTopicPub = m_node.advertise<std_msgs::String>(m_stateTopic, 1);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
LwrSafeCartesian::setJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg)
{
  std::cout << "setJointCallback: jointsMsg=" << *jointsMsg << std::endl;
  /*
  for (size_t jointIdx = 0; jointIdx < LWR_JOINTS; jointIdx++) {
    m_gpiPosTargetBuffer[jointIdx] = jointsMsg->position[jointIdx];
  }
  */
}

void
LwrSafeCartesian::setCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg)
{
  std::cout << "setCartesianCallback: poseMsg=" << *poseMsg << std::endl;
}

void
LwrSafeCartesian::publish()
{
  //m_getJointTopicPub.publish(m_currentJointState);
}
/*------------------------------------------------------------------------}}}-*/
