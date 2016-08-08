#include "contact_direction.h"

ContactDirection::ContactDirection() :
  isInitialized(false),
  direction(Contact::DIM_X),
  velocityMax(0.0),
  forceTorqueMax(0.0),
  displacementMax(0.0),
  endOnDiff(false),
  ftDiffMax(0.0),
  springConstant(0.0),
  dampingCoeff(0.0),
  movementType(Contact::DIRECTION),
  isReady(false),
  controlFrame(""),
  worldFrame("")
{
  // Do nothing
}

ContactDirection::~ContactDirection()
{
  delete listener;
}

void ContactDirection::initialize(Contact::Dimension dim, std::string wf, std::string cf, tf::TransformListener* list)
{
  direction = dim;
  worldFrame = wf;
  controlFrame = cf;
  listener = list;
  isInitialized = true;
}

void ContactDirection::setMovement(double vMax, double ftMax, double dMax, geometry_msgs::PoseStamped pose)
{
  movementType = Contact::DIRECTION;
  velocityMax = vMax;
  forceTorqueMax = ftMax;
  displacementMax = dMax;
  setStart(pose);
  if(fabs(velocityMax) <= 1e-3)
  {
    ROS_ERROR("Max velocity must be non zero");
    isReady = false;
  }
  else if(fabs(forceTorqueMax) <= 1e-3)
  {
    ROS_ERROR("Force Torque max must be non zero");
    isReady = false;
  }
  else
  { 
    springConstant = forceTorqueMax/velocityMax;
    isReady = true;
  }
}

void ContactDirection::setSpring(double k, double b, double ftMax, geometry_msgs::PoseStamped pose)
{
  movementType = Contact::SPRING;
  setStart(pose);
  if(fabs(k) <= 1e-3 || fabs(b) <= 1e-3)
  {
    ROS_ERROR("Spring and damping constants must be non zero");
    isReady = false;
  }
  else
  {
    springConstant = k;
    dampingCoeff = b;
    isReady = true;
  }
  forceTorqueMax = ftMax;
}

void ContactDirection::setFollower(double b, double ftMax, geometry_msgs::PoseStamped pose)
{
  movementType = Contact::FOLLOWER;
  setStart(pose);
  if(fabs(b) <= 1e-3)
  {
    ROS_ERROR("Damping constant must be non zero");
    isReady = false;
  }
  else
  {
    dampingCoeff = b;
    isReady = true;
  }
  forceTorqueMax = ftMax;
}

double ContactDirection::getVelocity(double ft, geometry_msgs::PoseStamped pose)
{
  if(isReady)
  {
    if(Contact::SPRING == movementType)
    {
      return (springConstant*-getTravel(pose) + (1/dampingCoeff)*ft);
    }
    else if(Contact::DIRECTION == movementType)
    {
      return checkSpeed((velocityMax - (ft/springConstant)), pose);
    }
    else if(Contact::FOLLOWER == movementType)
    {
      return (1/dampingCoeff)*ft;
    }
    else
    {
      ROS_ERROR("Movement type is incorrect.");
      return 0.0;
    }
  }
  else
  {
    return 0.0;
  }
  if(endOnDiff)
  {
    writeFT(ft);
  }
}

void ContactDirection::reset()
{
  isReady = false;
  clearFT();
}

bool ContactDirection::getCondition(double ft, geometry_msgs::PoseStamped pose)
{
  bool endCondition = false;

  if(isReady)
  {
    if(fabs(ft)>=forceTorqueMax)
    {
      endCondition = true;
    }
    if(Contact::DIRECTION == movementType)
    {
      if(fabs(getTravel(pose))>=displacementMax || checkDiff())
      {
        endCondition = true;
      }
    }
  }
  //ROS_INFO_STREAM("Dimension: " << direction << " FT: " << ft << " Travel: " << fabs(getTravel(pose)) << " Max: " << displacementMax);
  return endCondition;
}

void ContactDirection::writeFT(double ft)
{
  ftHistory.add(ros::Time::now().toSec(), ft);
}

void ContactDirection::clearFT()
{
  ftHistory.clear();
}

void ContactDirection::setMaxDiff(double diffMax, double time)
{
  if(diffMax != 0.0)
  {
    endOnDiff = true;
    ftDiffMax = diffMax;
    ftHistory.initialize(time);
  }
  else
  {
    endOnDiff = false;
  }
}

bool ContactDirection::checkDiff()
{
  if(endOnDiff)
    return ftHistory.getDiff() > ftDiffMax;
  return false;
}

void ContactDirection::setStart(geometry_msgs::PoseStamped pose)
{
  startPose = pose;
}

double ContactDirection::getTravel(geometry_msgs::PoseStamped pose)
{
  double travel = 0.0;
  geometry_msgs::Vector3Stamped diff;
  ros::Time time = ros::Time::now();
  switch(direction)
  {
    case Contact::DIM_X:
    case Contact::DIM_Y:
    case Contact::DIM_Z:
    {
      diff.header.stamp = time;
      diff.header.frame_id = worldFrame;
      diff.vector.x = pose.pose.position.x - startPose.pose.position.x;
      diff.vector.y = pose.pose.position.y - startPose.pose.position.y;
      diff.vector.z = pose.pose.position.z - startPose.pose.position.z;
      if(listener->waitForTransform(controlFrame, time, worldFrame, time, worldFrame, ros::Duration(1.0)))
      {
        listener->transformVector(controlFrame, time, diff, worldFrame, diff);
      }
      else
      {
        ROS_ERROR("Could not get travel. Wait for transform timed out.");
      }
      switch(direction)
      {
        case Contact::DIM_X:
          travel = diff.vector.x;
          break;
        case Contact::DIM_Y:
          travel = diff.vector.y;
          break;
        case Contact::DIM_Z:
          travel = diff.vector.z;
          break;
        default:
          break;
      }
    }
      break;
    case Contact::DIM_RX:
    case Contact::DIM_RY:
    case Contact::DIM_RZ:
    {
      tf::Stamped<tf::Pose> currentPose;
      tf::Stamped<tf::Pose> initPose;
      tf::poseStampedMsgToTF(pose, currentPose);
      tf::poseStampedMsgToTF(startPose, initPose);
      tf::Quaternion currentQuat;
      tf::Quaternion startQuat;
      currentQuat = currentPose.getRotation();
      startQuat = initPose.getRotation();
      travel = startQuat.angleShortestPath(currentQuat);
    }
      break;
    default:
      ROS_ERROR("Direction class initialized with incorrect direction variable");
      break;
  }
  return travel;
}

double ContactDirection::checkSpeed(double velocity, geometry_msgs::PoseStamped pose)
{
  double remainingTravel = displacementMax - fabs(getTravel(pose));
  int sign = 1;
  if (velocity < 0)
    sign = -1;
  if(remainingTravel <= 0.02 && fabs(velocity) >= 0.3)
  {
    return sign*0.3;
  }
  else if (remainingTravel <= 0.01 && fabs(velocity) >=0.1)
  {
    return sign*0.1;
  }
  else
  {
    return velocity;
  }
}
