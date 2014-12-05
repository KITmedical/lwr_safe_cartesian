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
LwrSafeCartesian::LwrSafeCartesian(const std::string& p_robotName)
  :m_robotName(p_robotName),
   m_current_nsparam(0),
   m_fixed_nsparam(true),
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
  m_targetJointState.name = m_jointNames;

  // must be available in order for CollisionCheckMoveIt to initialize (if joint_topic_merger.py is used)
  m_getJointTopicPub = m_node.advertise<sensor_msgs::JointState>("get_joint", 1);
  m_getCartesianTopicPub = m_node.advertise<geometry_msgs::Pose>("get_cartesian", 1);
  m_directGetJointTopicSub = m_node.subscribe<sensor_msgs::JointState>("direct/get_joint", 1, &LwrSafeCartesian::directGetJointCallback, this);

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
  m_gpi.setDt(0.05);
  m_gpi.setMode(1);

  m_setJointTopicSub = m_node.subscribe<sensor_msgs::JointState>("set_joint", 1, &LwrSafeCartesian::setJointCallback, this);
  m_setCartesianTopicSub = m_node.subscribe<geometry_msgs::Pose>("set_cartesian", 1, &LwrSafeCartesian::setCartesianCallback, this);
  m_setUnsafeCartesianTopicSub = m_node.subscribe<geometry_msgs::Pose>("unsafe/set_cartesian", 1, &LwrSafeCartesian::setUnsafeCartesianCallback, this);
  m_stateTopicPub = m_node.advertise<std_msgs::String>("state", 1);

  m_directSetJointTopicPub = m_node.advertise<sensor_msgs::JointState>("direct/set_joint", 1);
  m_directStateTopicSub = m_node.subscribe<std_msgs::String>("direct/state", 1, &LwrSafeCartesian::directStateCallback, this);

  m_setElbowService = m_node.advertiseService("set_elbow", &LwrSafeCartesian::setElbowCallback, this);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
bool
LwrSafeCartesian::pathHasCollision(const sensor_msgs::JointState& targetJointState)
{
  if (joint_dist(m_lastJointState, targetJointState) < path_collision_check_dist_threshold) {
    return m_collision_check->hasCollision(targetJointState);
  } else {
    ROS_INFO("Checking PATH for collisions");
    for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
      m_gpiPosCurrentBuffer[jointIdx] = m_lastJointState.position[jointIdx];
    }
    m_gpi.setXLast(m_gpiPosCurrentBuffer);
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
    //ROS_INFO("targetJointStateVec.size()=%zd", targetJointStateVec.size());

    return m_collision_check->hasPathCollision(targetJointStateVec);
  }
}

void
LwrSafeCartesian::setJointCallback(const sensor_msgs::JointState::ConstPtr& p_jointsMsg)
{
  sensor_msgs::JointState jointsMsg(*p_jointsMsg);
  //std::cout << "setJointCallback: jointsMsg=\n" << jointsMsg << std::endl;

  if (jointsMsg.position.size() != LWR_JOINTS) {
    ROS_FATAL_STREAM("Wrong number of joints. Will not move robot at all.\n");
    m_currentState.data = "SAFE_LWR_ERROR|SAFE_LWR_WRONG_NUMBER_OF_JOINTS";
    m_stateTopicPub.publish(m_currentState);
    return;
  }

  // use default names
  if (jointsMsg.name.size() == 0) {
    jointsMsg.name = m_jointNames;
  }

  for (size_t jointIdx = 0; jointIdx < LWR_JOINTS; jointIdx++) {
    if (std::abs(jointsMsg.position[jointIdx]) > Lwr::jointLimits.j[jointIdx]) {
      ROS_FATAL_STREAM("Joint" << jointIdx << " beyond joint limit (is=" << jointsMsg.position[jointIdx] << " limit=" << Lwr::jointLimits.j[jointIdx] << "). Will not move robot at all.");
      m_currentState.data = "SAFE_LWR_ERROR|SAFE_LWR_JOINT_LIMIT_EXCEEDED";
      m_stateTopicPub.publish(m_currentState);
      return;
    }
  }

  if (pathHasCollision(jointsMsg)) {
    std::cout << "------------------> COLLISION <---------------" << std::endl;
    m_currentState.data = "SAFE_LWR_ERROR|SAFE_LWR_COLLISION";
    m_stateTopicPub.publish(m_currentState);
    return;
  }

  m_targetJointState = jointsMsg;
  publishToHardware();
}

void
LwrSafeCartesian::doCartesian(const geometry_msgs::Pose::ConstPtr& poseMsg, bool collision_checking)
{
  tf::Pose tfpose;
  tf::poseMsgToTF(*poseMsg, tfpose);
  tf::Vector3 tforigin = tfpose.getOrigin();
  tf::Quaternion tforientation = tfpose.getRotation();
  LwrXCart cartXPose;
  cartXPose.nsparam = m_current_nsparam;
  cartXPose.config = 2;
  cartXPose.pose.setPos(tforigin.x(), tforigin.y(), tforigin.z());
  double x, y, z, w;
  w = tforientation.w();
  x = tforientation.x();
  y = tforientation.y();
  z = tforientation.z();
  cartXPose.pose.setQuat(w, x, y, z);
  LwrJoints joints;
  double best_nsparam = 0; // best = closest to zero
  if (!m_fixed_nsparam) {
    LwrElbowInterval currentMargin;
    LwrElbowInterval reachable[11];
    LwrElbowInterval blocked[11];
    LwrElbowInterval blockedPerJoint[7][3];
    LwrErrorMsg elbowIntervalReturn;
    elbowIntervalReturn = Lwr::elbowIntervals(currentMargin, reachable, blocked, blockedPerJoint, cartXPose);
    if (elbowIntervalReturn != LWR_OK) {
      ROS_WARN_STREAM("Lwr::elbowIntervals() failed: " << elbowIntervalReturn);
      m_currentState.data = "SAFE_LWR_ERROR|SAFE_LWR_ELBOW_INTERVAL";
      m_stateTopicPub.publish(m_currentState);
      return;
    }
    best_nsparam = 1e6;
    for (size_t ival_idx = 0; ival_idx < 11; ival_idx++) {
      if (!reachable[ival_idx].valid) {
        continue;
      }

      //std::cout << "reachable[" << ival_idx << "]=" << reachable[ival_idx] << std::endl;
      if (reachable[ival_idx].lower <= 0 and reachable[ival_idx].upper >= 0) {
        best_nsparam = 0;
        break;
      } else {
        double local_best_nsparam;
        if (std::abs(reachable[ival_idx].lower) < std::abs(reachable[ival_idx].upper)) {
          local_best_nsparam = reachable[ival_idx].lower;
        } else {
          local_best_nsparam = reachable[ival_idx].upper;
        }
        if (std::abs(local_best_nsparam) < std::abs(best_nsparam)) {
          best_nsparam = local_best_nsparam;
        }
      }
    }
  } else {
    best_nsparam = m_current_nsparam;
  }
  //std::cout << "best_nsparam=" << best_nsparam << std::endl;
  cartXPose.nsparam = best_nsparam;
  LwrErrorMsg kinematicReturn;
  kinematicReturn = Lwr::inverseKinematics(joints, cartXPose);

  //printf("kinematicReturn=0x%02X=%s\n", kinematicReturn, Lwr::errorToString(kinematicReturn).c_str());
  if (kinematicReturn != LWR_OK && kinematicReturn != (LWR_WARNING | LWR_CLOSE_TO_SINGULARITY)) {
    ROS_WARN_STREAM("Lwr::inverseKinematics() failed: " << kinematicReturn);
    m_currentState.data = "SAFE_LWR_ERROR|SAFE_LWR_INVERSE_KINEMATICS";
    m_stateTopicPub.publish(m_currentState);
    return;
  }

  sensor_msgs::JointState unsafeTargetJointState(m_targetJointState);
  for (size_t jointIdx = 0; jointIdx < m_targetJointState.position.size(); jointIdx++) {
    unsafeTargetJointState.position[jointIdx] = joints.j[jointIdx];
  }
  //std::cout << "unsafeTargetJointState: " << unsafeTargetJointState << std::endl;

  if (collision_checking && pathHasCollision(unsafeTargetJointState)) {
    std::cout << "------------------> COLLISION <---------------" << std::endl;
    m_currentState.data = "SAFE_LWR_ERROR|SAFE_LWR_COLLISION";
    m_stateTopicPub.publish(m_currentState);
    return;
  }

  m_current_nsparam = cartXPose.nsparam;
  m_targetJointState = unsafeTargetJointState;
  publishToHardware();
  m_currentState.data = Lwr::errorToString(kinematicReturn);
  m_stateTopicPub.publish(m_currentState);
}

void
LwrSafeCartesian::setCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg)
{
  //std::cout << "setCartesianCallback: poseMsg=\n" << *poseMsg << std::endl;
  doCartesian(poseMsg);
}

void
LwrSafeCartesian::setUnsafeCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg)
{
  //std::cout << "setUnsafeCartesianCallback: poseMsg=\n" << *poseMsg << std::endl;
  doCartesian(poseMsg, false);
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

bool
LwrSafeCartesian::setElbowCallback(safe_cartesian_msgs::SetElbow::Request& request, safe_cartesian_msgs::SetElbow::Response& response)
{
  if (request.elbow.fixed) {
    m_fixed_nsparam = true;
    m_current_nsparam = request.elbow.nsparam;
  } else {
    m_fixed_nsparam = false;
  }

  return true;
}
/*------------------------------------------------------------------------}}}-*/
