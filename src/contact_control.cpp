#include "contact_control.h"

ContactControl::ContactControl(ros::NodeHandle* nh) :
fixedFrame(""),
velFrame(""),
ftFrame(""),
controlFrame(""),
ftAddress(""),
ftTopic(""),
velTopic("jog_command"),
controlRate(500),
n(nh),
endCondition(Contact::NO_CONDITION),
isMoving(false),
gravBalance(false),
leverDim(Contact::NUM_DIMS),
forceDim(Contact::NUM_DIMS),
leverWorld(false),
gravLever(0.0),
monitorFT(false),
isInit(false),
netftCancel(false)
{
  mi = new MoveInterface();
}

ContactControl::~ContactControl()
{
  delete n;
  delete spinner;
  delete fti;
  delete listener;
  delete mi;
}

void ContactControl::initialize(std::string mg, std::string ff, std::string vf, std::string ftf, std::string cf)
{
  // Start spinner
  spinner = new ros::AsyncSpinner(3);
  spinner->start();

  // Create force/torque interface
  fti = new NetftUtilsLean(n);
  if (!ftTopic.empty())
  {
    fti->setFTTopic(ftTopic);
  }
  else if (!ftAddress.empty())
  {
    fti->setFTAddress(ftAddress);
  }
  else
  {
    ROS_ERROR("Must set ft topic or address before initializing ContactControl.");
    return;
  }
  
  if(!fti->initialize(controlRate, ff, ftf))
  {
    ROS_ERROR("Failed to initialize force torque interface");
    return;
  }
  // Set max force and torque (used for threshold to cancel moves)
  fti->setMax(80.0, 8.0, 60.0, 6.0);
  
  // Initialize variables
  if(ff.compare("") == 0 || vf.compare("") == 0 || ftf.compare("") == 0 || cf.compare("") == 0)
  {
    ROS_ERROR("All frames must be non-empty. Cannot initialize Contact Control.");
    return;
  }
  fixedFrame = ff;
  velFrame = vf;
  ftFrame = ftf;
  controlFrame = cf;
  
  // Initiallize the move interface
  mi->initialize(mg);

  // Initialize transform listener (give it a buffer of 300 seconds)
  listener = new tf::TransformListener(ros::Duration(300));

  // Initialize ROS publishers
  delta_pub = n->advertise<geometry_msgs::TwistStamped>(velTopic, 1);
  data_pub = n->advertise<std_msgs::Float64>("/position_delta", 100);

  // Initialize ROS subscribers
  netft_cancel_sub = n->subscribe("/netft/cancel", 1, &ContactControl::cancelCallback, this);

  // Initialize contact directions
  direction[Contact::DIM_X].initialize(Contact::DIM_X, velFrame, controlFrame, listener);
  direction[Contact::DIM_RX].initialize(Contact::DIM_RX, velFrame, controlFrame, listener);
  direction[Contact::DIM_Y].initialize(Contact::DIM_Y, velFrame, controlFrame, listener);
  direction[Contact::DIM_RY].initialize(Contact::DIM_RY, velFrame, controlFrame, listener);
  direction[Contact::DIM_Z].initialize(Contact::DIM_Z, velFrame, controlFrame, listener);
  direction[Contact::DIM_RZ].initialize(Contact::DIM_RZ, velFrame, controlFrame, listener);
  
  isInit = true;
}

std::future<Contact::EndCondition> ContactControl::moveAsync(double fMax, double tMax, double vMax)
{
  return std::async(std::launch::async, &ContactControl::move, this, fMax, tMax, vMax);
}

Contact::EndCondition ContactControl::move(double fMax, double tMax, double vMax)
{
  // Do not do anything if Contact Control did not initialize properly
  if(!isInit)
  {
    ROS_ERROR_STREAM("Cannot perform move. Contact control was not initialized properly.");
    return Contact::NOT_INITIALIZED;
  }
  
  std::future<bool> ftThread;
  bool startedFT = false;
  if(!fti->isRunning())
  {
    ROS_INFO("Starting FT");
    ftThread = std::async(std::launch::async, &NetftUtilsLean::run, fti);
    startedFT = true;
  }
  // Cancel move if netft is not available
  if(!fti->waitForData(10.0))
  {
    ROS_ERROR("Cannot perform move. Netft is not receiving data.");
    gravBalance = false;
    return Contact::NO_FT_DATA;
  }
  // Turn on ft monitoring
  monitorFT = true;
  std::future<bool> monitorThread = std::async(std::launch::async, &ContactControl::ftMonitor, this);
  
  // Cancel move if there is already a move happenning
  if(isMoving)
  {
    ROS_ERROR("Cannot perform move while another move is executing. Please use stopMove() to end the previous move first.");
    gravBalance = false;
    monitorFT = false;
    monitorThread.get();
    return Contact::IN_MOTION;
  }
  else
  {
    isMoving = true;
    ROS_INFO("Starting move.");
  }
  
  // Record start position
  startPose = mi->getCurrentPose();

  // Setup jogging variables and initialize jog command to zero
  std::vector<double> deltas;
  deltas.resize(6);
  for(int i = 0; i<deltas.size(); i++)
  {
    deltas[i] = 0.0;
  }

  endCondition = Contact::NO_CONDITION;
  // Set loop rate
  ros::Rate loopRate(controlRate);
  // Start moving according to the control law
  while(!netftCancel && Contact::NO_CONDITION == endCondition && ros::ok())
  {
    // Publish a velocity command and check end conditions
    int startInfo = -1;
    for(int i = 0; i<Contact::NUM_DIMS; i++)
    {
      if(direction[i].hasStartInfo())
      {
        startInfo = i;
      }
    }
    if(-1 == startInfo)
    {
      ROS_ERROR("Cannot perform move. You have not initialized any control directions");
      isMoving = false;
      gravBalance = false;
      monitorFT = false;
      monitorThread.get();
      return Contact::NO_LAWS;
    }
    geometry_msgs::PoseStamped currentPose = mi->getCurrentPose();
    direction[startInfo].toStartFrame(currentPose);
    ros::Time time;
    double ft;
    std::vector<double> torque;
    torque.resize(6);
    for(int k = 0; k<torque.size(); k++)
      torque.at(k) = 0.0;
    // If trying to balance, find torque cause by contact with surface
    if(gravBalance)
    {
      tf::Vector3 torqueTF;
      geometry_msgs::WrenchStamped tempFtData;
      if(leverWorld)
      {
        tempFtData.header = ftDataWorld.header;
        tempFtData.wrench = ftDataWorld.wrench;
      }
      else
      {
        tempFtData.header = ftData.header;
        tempFtData.wrench = ftData.wrench;
      }
      switch(leverDim)
      {
        case Contact::DIM_X:
          if(Contact::DIM_Y == forceDim)
            torqueTF.setX(tempFtData.wrench.force.z * gravLever);
          if(Contact::DIM_Z == forceDim)
            torqueTF.setX(tempFtData.wrench.force.y * gravLever);
          break;
        case Contact::DIM_Y:
          if(Contact::DIM_X == forceDim)
            torqueTF.setX(tempFtData.wrench.force.z * gravLever);
          if(Contact::DIM_Z == forceDim)
            torqueTF.setX(tempFtData.wrench.force.x * gravLever);
          break;
        case Contact::DIM_Z:
          if(Contact::DIM_Y == forceDim)
            torqueTF.setX(tempFtData.wrench.force.x * gravLever);
          if(Contact::DIM_X == forceDim)
            torqueTF.setX(tempFtData.wrench.force.y * gravLever);
          break;
      }
      if(velFrame.compare(controlFrame) != 0 && leverWorld)
      {
        tf::StampedTransform tempTransform;
        try
        {
          listener->waitForTransform(controlFrame, velFrame, ros::Time(0), ros::Duration(1.0));
          listener->lookupTransform(controlFrame, velFrame, ros::Time(0), tempTransform);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("Error in balance tf transform. Stopping move.");
          stopMove();
        }
        tempTransform.setOrigin(tf::Vector3(0.0,0.0,0.0));
        torqueTF = tempTransform * torqueTF;
      }
      
      torque.at(Contact::DIM_RX) = torqueTF.getX();
      torque.at(Contact::DIM_RY) = torqueTF.getY();
      torque.at(Contact::DIM_RZ) = torqueTF.getZ();
    }
    
    for(int i = 0; i<Contact::NUM_DIMS; i++)
    {
      getFT(static_cast<Contact::Dimension>(i), ft, time);
      if(gravBalance)
      {
        ft -= torque.at(i);
      }
      deltas[i] = direction[i].getVelocity(ft, currentPose);
      Contact::EndCondition dirCondition = direction[i].getCondition(ft, currentPose);
      if(dirCondition != Contact::NO_CONDITION)
        endCondition = dirCondition;
    }
    if(Contact::NO_CONDITION == endCondition)
    {
      // Convert deltas to world frame to send to controller
      toVelFrame(deltas, time);
      // Limit deltas to be less than vMax
      if(vMax > 1.0 || vMax <=0.0)
      {
        ROS_ERROR("vMax must be between 0 and 1. Setting vMax to 0.5");
        vMax = 0.5;
      }
      else if(vMax == 1.0)
      {
        vMax = 0.99;
      }
      bool sendJog = false;
      for(int i = 0; i<Contact::NUM_DIMS; i++)
      {
        if(deltas[i] > vMax)
          deltas[i] = vMax;
        else if(deltas[i] < -vMax)
          deltas[i] = -vMax;
        if(fabs(deltas[i])>1e-5)
          sendJog = true;
      }
      // Send deltas to controller
      geometry_msgs::TwistStamped jogCmd;
      if (sendJog)
      {
        jogCmd.header.frame_id = velFrame;
        jogCmd.header.stamp = ros::Time::now();
        jogCmd.twist.linear.x = deltas[0];
        jogCmd.twist.linear.y = deltas[1];
        jogCmd.twist.linear.z = deltas[2];
        jogCmd.twist.angular.x = deltas[3];
        jogCmd.twist.angular.y = deltas[4];
        jogCmd.twist.angular.z = deltas[5];
        delta_pub.publish(jogCmd);
      }
    }
    else
    {
      ROS_INFO_STREAM("Met end condition. Stopping move."); 
    }
    loopRate.sleep();
    // Publish data to be plotted
    //std_msgs::Float64 temp;
    //temp.data = travel;
    //data_pub.publish(temp);
    ////ROS_INFO_STREAM("Travel: " << travel);
    //ROS_INFO_STREAM("Force or Torque: " << ft);
    ////ROS_INFO_STREAM("Velocity: " << (vMax - (ft/k)));
  } 
  ROS_INFO("Finished move.");
  for(int j = 0; j<Contact::NUM_DIMS; j++)
  {
    direction[j].reset();
  }
  isMoving = false;
  gravBalance = false;
  
  //Turn off FT monitor
  monitorFT = false;
  if(startedFT)
    fti->stop();
  monitorThread.get();
  
  if(!startedFT)
    return endCondition;
    
  ftThread.get();
  return endCondition;
}

Contact::EndCondition ContactControl::balance(Contact::Dimension lDim, Contact::Dimension fDim, bool inWorld, double leverArm, Contact::Dimension mDim, double mSpeed, double fMax, double tMax)
{
  if(fabs(mSpeed) > (1+1e-3))
  {
    ROS_ERROR_STREAM("Speed value must be between -1 and 1. Cannot perform balance procedure.");
    return Contact::INVALID_PARAMS;
  }
  if(fabs(leverArm)<=1e-3)
  {
    ROS_ERROR_STREAM("Lever arm must be non zero. Cannot perform balance procedure.");
    return Contact::INVALID_PARAMS;
  }
  if(lDim == Contact::DIM_RX || lDim == Contact::DIM_RY || lDim == Contact::DIM_RZ || fDim == Contact::DIM_RX || fDim == Contact::DIM_RY || fDim == Contact::DIM_RZ)
  {
    ROS_ERROR_STREAM("Lever dimension and force dimension must be a linear dimension. Cannot perform balance procedure.");
    return Contact::INVALID_PARAMS;
  }
  if(mDim == Contact::DIM_RX || mDim == Contact::DIM_RY || mDim == Contact::DIM_RZ)
  {
    ROS_ERROR_STREAM("Movement dimension must be a linear dimension. Cannot perform balance procedure.");
    return Contact::INVALID_PARAMS;
  }
  if(fDim == lDim)
  {
    ROS_ERROR_STREAM("Lever dimension and force dimension cannot be the same. Cannot perform balance procedure.");
    return Contact::INVALID_PARAMS;
  }
  leverDim = lDim;
  forceDim = fDim;
  leverWorld = inWorld;
  gravLever = leverArm;
  gravBalance = true;
  setMovement(mDim, mSpeed, 4.0, 0.1, 30.0);
  switch(mDim)
  {
    case Contact::DIM_X:
      setFollower(Contact::DIM_RY, 100.0, 20.0);
      setFollower(Contact::DIM_RZ, 100.0, 20.0);
      break;
    case Contact::DIM_Y:
      setFollower(Contact::DIM_RX, 100.0, 20.0);
      setFollower(Contact::DIM_RZ, 100.0, 20.0);
      break;
    case Contact::DIM_Z:
      setFollower(Contact::DIM_RY, 100.0, 20.0);
      setFollower(Contact::DIM_RX, 100.0, 20.0);
      break;
  }
  return move(fMax, tMax);
}

std::future<Contact::EndCondition> ContactControl::balanceAsync(Contact::Dimension lDim, Contact::Dimension fDim, bool inWorld, double leverArm, Contact::Dimension mDim, double mSpeed, double fMax, double tMax)
{
  return std::async(std::launch::async, &ContactControl::balance, this, lDim, fDim, inWorld, leverArm, mDim, mSpeed, fMax, tMax);
}

void ContactControl::toVelFrame(std::vector<double> &cmd, ros::Time time)
{
  if(cmd.size() != 6)
  {
    ROS_ERROR("Could not convert to velocity command frame. Vector should have a size of 6");
    return;
  }
  if(controlFrame.compare(velFrame) != 0)
  {
    geometry_msgs::Vector3Stamped linear;
    geometry_msgs::Vector3Stamped angular;
    linear.header.stamp = time;
    linear.header.frame_id = controlFrame;
    linear.vector.x = cmd[0];
    linear.vector.y = cmd[1];
    linear.vector.z = cmd[2];
    angular.header.stamp = time;
    angular.header.frame_id = controlFrame;
    angular.vector.x = cmd[3];
    angular.vector.y = cmd[4];
    angular.vector.z = cmd[5];
    if(listener->waitForTransform(velFrame, time, controlFrame, time, fixedFrame, ros::Duration(1.0)))
    {
      listener->transformVector(velFrame, time, linear, fixedFrame, linear);
      listener->transformVector(velFrame, time, angular, fixedFrame, angular);
      cmd[0] = linear.vector.x;
      cmd[1] = linear.vector.y;
      cmd[2] = linear.vector.z;
      cmd[3] = angular.vector.x;
      cmd[4] = angular.vector.y;
      cmd[5] = angular.vector.z;
    }
    else
    {
      ROS_ERROR("Could not transform to velocity command frame. Wait for transform timed out.");
    }
  }
}

void ContactControl::toControlFrame(geometry_msgs::WrenchStamped &data)
{
  if(data.header.frame_id.compare(controlFrame)!=0)
  {
    geometry_msgs::Vector3Stamped force;
    geometry_msgs::Vector3Stamped torque;
    force.header.stamp = data.header.stamp;
    force.header.frame_id = data.header.frame_id;
    force.vector.x = data.wrench.force.x;
    force.vector.y = data.wrench.force.y;
    force.vector.z = data.wrench.force.z;
    torque.header.stamp = data.header.stamp;
    torque.header.frame_id = data.header.frame_id;
    torque.vector.x = data.wrench.torque.x;
    torque.vector.y = data.wrench.torque.y;
    torque.vector.z = data.wrench.torque.z;
    if(listener->waitForTransform(controlFrame, data.header.stamp, data.header.frame_id, data.header.stamp, fixedFrame, ros::Duration(1.0)))
    {
      listener->transformVector(controlFrame, data.header.stamp, force, fixedFrame, force);
      listener->transformVector(controlFrame, data.header.stamp, torque, fixedFrame, torque);
      data.wrench.force.x = force.vector.x;
      data.wrench.force.y = force.vector.y;
      data.wrench.force.z = force.vector.z;
      data.wrench.torque.x = torque.vector.x;
      data.wrench.torque.y = torque.vector.y;
      data.wrench.torque.z = torque.vector.z;
      data.header.frame_id = controlFrame;
    }
    else
    {
      ROS_ERROR("Could not transform to control frame. Wait for transform timed out.");
    }
  }
}

void ContactControl::adjustSpringConstant(Contact::Dimension dim, double k)
{
  direction[dim].adjustSpringConstant(k);
}

void ContactControl::setMovement(Contact::Dimension dim, double vMax, double ftStall, double dMax, double ftMax)
{
  direction[dim].setMovement(vMax, ftStall, dMax, ftMax, mi->getCurrentPose());
}

void ContactControl::setSpring(Contact::Dimension dim, double k, double b, double posOffset, double ftMax)
{
  direction[dim].setSpring(k, b, posOffset, ftMax, mi->getCurrentPose());
}

void ContactControl::setSpring(Contact::Dimension dim, double k, double b, double posOffset, double dMax, double ftMax)
{
  direction[dim].setSpring(k, b, posOffset, dMax, ftMax, mi->getCurrentPose());
}

void ContactControl::setFollower(Contact::Dimension dim, double b, double ftMax)
{
  direction[dim].setFollower(b, ftMax, mi->getCurrentPose());
}

bool ContactControl::checkDiff(Contact::Dimension dim)
{
  return direction[dim].checkDiff();
}

// Set a maximum differential force as a stop condition (set to zero to disable feature)
void ContactControl::setMaxDiff(Contact::Dimension dim, double diffMax, double time)
{
  direction[dim].setMaxDiff(diffMax, time);
}

double ContactControl::getTravel(Contact::Dimension dim)
{
  return direction[dim].getTravel(mi->getCurrentPose());
}

double ContactControl::getTravel(Contact::Perspective perspective)
{
  geometry_msgs::PoseStamped pose = mi->getCurrentPose();
  double travel = 0.0;
  switch (perspective)
  {
    case Contact::TRANSLATION:
      travel = pow(pow(pose.pose.position.x - startPose.pose.position.x, 2) +
                   pow(pose.pose.position.y - startPose.pose.position.y, 2) +
                   pow(pose.pose.position.z - startPose.pose.position.z, 2), 0.5);
      break;
    case Contact::ROTATION:
    {
      tf::Stamped<tf::Pose> tfPose, tfStartPose;
      tf::poseStampedMsgToTF(pose, tfPose);
      tf::poseStampedMsgToTF(startPose, tfStartPose);
      tf::Quaternion quat, startQuat;
      quat = tfPose.getRotation();
      startQuat = tfStartPose.getRotation();
      travel = startQuat.angleShortestPath(quat);
    }
      break;
    default:
      ROS_ERROR("Incorrect perspective in getTravel call.");
      break;
  }
  return travel;
}

void ContactControl::getFT(Contact::Dimension dim, double &ft, ros::Time &time)
{
  time = ftData.header.stamp;
  ft = 0.0;
  switch(dim)
  {
    case Contact::DIM_X:
      ft = ftData.wrench.force.x;
      break;
    case Contact::DIM_Y:
      ft = ftData.wrench.force.y;
      break;
    case Contact::DIM_Z:
      ft = ftData.wrench.force.z;
      break;
    case Contact::DIM_RX:
      ft = ftData.wrench.torque.x;
      break;
    case Contact::DIM_RY:
      ft = ftData.wrench.torque.y;
      break;
    case Contact::DIM_RZ:
      ft = ftData.wrench.torque.z;
      break;
    default:
      ROS_ERROR("Incorrect dimension in getFT call.");
      break;
  }
}

void ContactControl::getFT(Contact::Perspective perspective, double &ft, ros::Time &time)
{
  time = ftData.header.stamp;
  ft = 0.0;
  switch(perspective)
  {
    case Contact::TRANSLATION:
      ft = pow(pow(ftData.wrench.force.x,2) + pow(ftData.wrench.force.y,2) + pow(ftData.wrench.force.z,2),0.5);
      break;
    case Contact::ROTATION:
      ft = pow(pow(ftData.wrench.torque.x,2) + pow(ftData.wrench.torque.y,2) + pow(ftData.wrench.torque.z,2),0.5);
      break;
    default:
      ROS_ERROR("Incorrect perspective in getFT call.");
      break;
  }
}

void ContactControl::cancelCallback(const netft_utils::Cancel::ConstPtr& msg)
{
  netftCancel = msg->toCancel;
}

bool ContactControl::ftMonitor()
{
  ros::Rate r(controlRate);
  while(monitorFT)
  {
    //ros::Time start = ros::Time::now();
    geometry_msgs::WrenchStamped temp;
    fti->getWorldData(temp);
    ftDataWorld.header = temp.header;
    ftDataWorld.wrench = temp.wrench;
    toControlFrame(temp);
    ftData.header = temp.header;
    ftData.wrench = temp.wrench;
    ///ROS_INFO_STREAM("Time from sensor: " << ros::Time::now().toSec()-temp.header.stamp.toSec());
    //ROS_INFO_STREAM("Time from sensor: " << ros::Time::now().toSec()-temp.header.stamp.toSec());
    r.sleep();
  }
  return true;
}

MoveInterface* ContactControl::getMI()
{
  return mi;
}

NetftUtilsLean* ContactControl::getFTI()
{
  return fti;
}

void ContactControl::stopMove()
{
  if(Contact::NO_CONDITION == endCondition)
  {
    endCondition = Contact::EXTERNAL;
  }
}

void ContactControl::reset()
{
  for(int i = 0; i<Contact::NUM_DIMS; i++)
    direction[i].reset();
}

void ContactControl::setFTAddress(std::string ftAdd)
{
  ftAddress = ftAdd;
}

void ContactControl::setFTTopic(std::string ftTop)
{
  ftTopic = ftTop;
}

void ContactControl::setVelTopic(std::string velTop)
{
  velTopic = velTop;
}

void ContactControl::setControlRate(double cRate)
{
  ftTopic = cRate;
}