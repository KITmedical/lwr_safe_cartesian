#include "LwrSafeCartesian.h"

// system includes

// library includes
#include <tf/transform_datatypes.h>

// custom includes
#include <ahbstring.h>
#include <safe_cartesian_moveit/CollisionCheckMoveIt.hpp>
#include <LwrLibrary.hpp>


#define LBR_MNJ 7

double
joint_dist(const sensor_msgs::JointState& j1, const sensor_msgs::JointState& j2)
{
  if (j1.position.size() != j2.position.size()) {
    ROS_ERROR("joint_dist: Unequal number of joints");
    return INFINITY;
  }

  double dist = 0;
  for (size_t joint_idx = 0; joint_idx < j1.position.size(); joint_idx++) {
    double d = j1.position[joint_idx] - j2.position[joint_idx];
    dist += d * d;
  }
  dist = sqrt(dist);

  return dist;
}

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
   m_directStateTopic(p_directStateTopic),
   m_gpi(LBR_MNJ),
   m_gpiPosCurrentBuffer(LBR_MNJ, 0),
   m_gpiPosTargetBuffer(LBR_MNJ, 0),
   m_gpiPosMinBuffer(LBR_MNJ, 0),
   m_gpiPosMaxBuffer(LBR_MNJ, 0),
   m_gpiVelCurrentBuffer(LBR_MNJ, 0),
   m_gpiVelMaxBuffer(LBR_MNJ, m_velMax),
   m_gpiAccelMaxBuffer(LBR_MNJ, m_accelMax)
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
  // gpi
  for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
    m_gpiPosMinBuffer[jointIdx] = -1 * Lwr::jointLimits.j[jointIdx];
    m_gpiPosMaxBuffer[jointIdx] = Lwr::jointLimits.j[jointIdx];
  }
  m_gpi.setXTarget(m_gpiPosTargetBuffer);
  m_gpi.setXLast(m_gpiPosCurrentBuffer);
  m_gpi.setVLast(m_gpiVelCurrentBuffer);
  m_gpi.setXMin(m_gpiPosMinBuffer);
  m_gpi.setXMax(m_gpiPosMaxBuffer);
  m_gpi.setVMax(m_gpiVelMaxBuffer);
  m_gpi.setAMax(m_gpiAccelMaxBuffer);
  m_gpi.setDt(0.1);
  m_gpi.setMode(1);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
bool
LwrSafeCartesian::pathHasCollision(const sensor_msgs::JointState& targetJointState)
{
  if (joint_dist(m_lastJointState, targetJointState) < path_collision_check_dist_threshold) {
    return m_collision_check->hasCollision(targetJointState);
  } else {
    ROS_INFO("> path_collision_check_dist_threshold");
    for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
      m_gpiPosTargetBuffer[jointIdx] = targetJointState.position[jointIdx];
    }
    m_gpi.setXTarget(m_gpiPosTargetBuffer);

    std::vector<sensor_msgs::JointState> targetJointStateVec;
    while (true) {
      m_gpi.interpolate();
      m_gpi.getXNow(m_gpiPosCurrentBuffer);

      // target not reachable
      if (targetJointStateVec.size() != 0 && std::equal(m_gpiPosCurrentBuffer.begin(), m_gpiPosCurrentBuffer.end(), targetJointStateVec[targetJointStateVec.size() - 1].position.begin())) {
        ROS_INFO("Target not reachable, counting this as collision");
        return true;
      }

      sensor_msgs::JointState intermediate_state(targetJointState);
      intermediate_state.position = m_gpiPosCurrentBuffer;
      targetJointStateVec.push_back(intermediate_state);

      // target reached
      if (std::equal(m_gpiPosCurrentBuffer.begin(), m_gpiPosCurrentBuffer.end(), targetJointState.position.begin())) {
        break;
      }
    }

    return m_collision_check->hasPathCollision(targetJointStateVec);
  }
}

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

  if (pathHasCollision(*jointsMsg)) {
    std::cout << "------------------> COLLISION <---------------" << std::endl;
    m_currentState.data = "SAFE_LWR_ERROR|SAFE_LWR_COLLISION";
    m_stateTopicPub.publish(m_currentState);
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

  if (pathHasCollision(unsafeTargetJointState)) {
    std::cout << "------------------> COLLISION <---------------" << std::endl;
    m_currentState.data = "SAFE_LWR_ERROR|SAFE_LWR_COLLISION";
    m_stateTopicPub.publish(m_currentState);
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
