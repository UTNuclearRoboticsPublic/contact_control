#include "contact_control.h"

ContactControl::ContactControl() :
  worldFrame(""),
  ftFrame(""),
  controlFrame(""),
  netftCancel(false)
{
  mi = new MoveInterface();
}

ContactControl::~ContactControl()
{
  delete listener;
  delete mi;
}

void ContactControl::initialize(std::string mg, std::string wf, std::string ftf, std::string cf)
{
  // Start spinner
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Initialize variables
  worldFrame = wf;
  ftFrame = ftf;
  controlFrame = cf;

  // Initiallize the move interface
  mi->initialize(mg);

  // Initialize transform listener
  listener = new tf::TransformListener(ros::Duration(300));

  // Initialize ROS publishers
  delta_pub = n.advertise<motoman_jogger::Deltas>("/cartesian_jogging_deltas", 1);
  data_pub = n.advertise<std_msgs::Float64>("/position_delta", 100);

  // Initialize ROS subscribers
  ft_sub = n.subscribe("/netft/transformed_tool", 1, &ContactControl::ftCallback, this);
  netft_cancel_sub = n.subscribe("/netft/cancel", 1, &ContactControl::cancelCallback, this);

  // Initialize contact directions
  direction[Contact::DIM_X].initialize(Contact::DIM_X, worldFrame, controlFrame, listener);
  direction[Contact::DIM_RX].initialize(Contact::DIM_RX, worldFrame, controlFrame, listener);
  direction[Contact::DIM_Y].initialize(Contact::DIM_Y, worldFrame, controlFrame, listener);
  direction[Contact::DIM_RY].initialize(Contact::DIM_RY, worldFrame, controlFrame, listener);
  direction[Contact::DIM_Z].initialize(Contact::DIM_Z, worldFrame, controlFrame, listener);
  direction[Contact::DIM_RZ].initialize(Contact::DIM_RZ, worldFrame, controlFrame, listener);

}

// One dimensional move
//void ContactControl::move(Contact::Dimension dim, double vMax, double ftMax, double dMax)
//{
//  ROS_INFO_STREAM("vMax: " << vMax);
//  // Record start position
//  startPose = mi->getCurrentPose();
//
//  // Setup jogging variables and initialize jog command to zero
//  motoman_jogger::Deltas jogCmd;
//  jogCmd.deltas.resize(6);
//  for(int i = 0; i<jogCmd.deltas.size(); i++)
//  {
//    jogCmd.deltas[i] = 0.0;
//  }
//
//  // setup useful variables
//  double ft = 0.0;
//  double travel = 0.0;
//  bool diffViolation = false;
//
//  // Get the spring constant from the desired max velocity and force
//  double k = ftMax / vMax;
//
//  // Bias the netft sensor TODO: set better max force and torque
//  netft_utils::SetBias bias_srv;
//  bias_srv.request.toBias = true;
//  bias_srv.request.forceMax = 50.0;
//  bias_srv.request.torqueMax = 5.0;
//  ros::service::call("/netft/bias", bias_srv);
//
//  // Start the simulated force/torque data if available
//  netft_utils::StartSim start_srv;
//  start_srv.request.simDim = dim;
//  start_srv.request.simType = 1;
//  start_srv.request.simSlope = 1.0;
//  if(Contact::DIM_X == dim || Contact::DIM_Y == dim || Contact::DIM_Z == dim)
//  {
//    start_srv.request.maxForce = 15.0;
//  }
//  else
//  {
//    start_srv.request.maxForce = 1.0;
//  }
//  ros::service::call("/netft/start_sim", start_srv);
//
//  // Start moving in the command direction according to the spring law
//  while(fabs(travel) < dMax && fabs(ft) <= ftMax && !diffViolation)
//  {
//    // Publish a velocity command
//    jogCmd.deltas[dim] = vMax - (ft/k);
//    delta_pub.publish(jogCmd);
//    //ros::spinOnce();
//    //ros::Duration(0.05).sleep();
//
//    // Update terminate condition variables
//    diffViolation = checkDiff(dim);
//    travel = getTravel(dim);
//    ft = getFT(dim);
//
//    // Publish data to be plotted
//    std_msgs::Float64 temp;
//    temp.data = travel;
//    data_pub.publish(temp);
//    ROS_INFO_STREAM("Travel: " << travel);
//    ROS_INFO_STREAM("Force or Torque: " << ft);
//    ROS_INFO_STREAM("Velocity: " << (vMax - (ft/k)));
//  }
//
//  // Stop the simulated force torque data if it is running
//  netft_utils::StopSim stop_srv;
//  ros::service::call("/netft/stop_sim", stop_srv);
//
//}

void ContactControl::move(double fMax, double tMax)
{
  // Record start position
  startPose = mi->getCurrentPose();

  // Setup jogging variables and initialize jog command to zero
  motoman_jogger::Deltas jogCmd;
  jogCmd.deltas.resize(6);
  for(int i = 0; i<jogCmd.deltas.size(); i++)
  {
    jogCmd.deltas[i] = 0.0;
  }

  // Bias the netft sensor TODO: set better max force and torque
  //netft_utils::SetBias bias_srv;
  //bias_srv.request.toBias = true;
  //bias_srv.request.forceMax = fMax;
  //bias_srv.request.torqueMax = tMax;
  //ros::service::call("/netft/bias", bias_srv);

  bool endCondition = false;

  // Start moving according to the control law
  while(!netftCancel && !endCondition)
  {
    // Publish a velocity command and check end conditions
    geometry_msgs::PoseStamped currentPose = mi->getCurrentPose();
    ros::Time time;
    double ft;
    for(int i = 0; i<Contact::NUM_DIMS; i++)
    {
      getFT(static_cast<Contact::Dimension>(i), ft, time);
      jogCmd.deltas[i] = direction[i].getVelocity(ft, currentPose);
      endCondition = endCondition || direction[i].getCondition(ft, currentPose);
    }
    if(!endCondition)
    {
      toWorldFrame(jogCmd, time);
      delta_pub.publish(jogCmd);
    }
    else
    {
      for(int j = 0; j<Contact::NUM_DIMS; j++)
      {
        direction[j].reset();
      }
    }
    // Publish data to be plotted
    //std_msgs::Float64 temp;
    //temp.data = travel;
    //data_pub.publish(temp);
    ////ROS_INFO_STREAM("Travel: " << travel);
    //ROS_INFO_STREAM("Force or Torque: " << ft);
    ////ROS_INFO_STREAM("Velocity: " << (vMax - (ft/k)));
  }
}

void ContactControl::toWorldFrame(motoman_jogger::Deltas &cmd, ros::Time time)
{
  if(controlFrame != worldFrame)
  {
    geometry_msgs::Vector3Stamped linear;
    geometry_msgs::Vector3Stamped angular;
    linear.header.stamp = time;
    linear.header.frame_id = controlFrame;
    linear.vector.x = cmd.deltas[0];
    linear.vector.y = cmd.deltas[1];
    linear.vector.z = cmd.deltas[2];
    angular.header.stamp = time;
    angular.header.frame_id = controlFrame;
    angular.vector.x = cmd.deltas[3];
    angular.vector.y = cmd.deltas[4];
    angular.vector.z = cmd.deltas[5];
    if(listener->waitForTransform(worldFrame, time, controlFrame, time, worldFrame, ros::Duration(1.0)))
    {
      listener->transformVector(worldFrame, time, linear, worldFrame, linear);
      listener->transformVector(worldFrame, time, angular, worldFrame, angular);
      cmd.deltas[0] = linear.vector.x;
      cmd.deltas[1] = linear.vector.y;
      cmd.deltas[2] = linear.vector.z;
      cmd.deltas[3] = angular.vector.x;
      cmd.deltas[4] = angular.vector.y;
      cmd.deltas[5] = angular.vector.z;
    }
    else
    {
      ROS_ERROR("Could not transform to world frame. Wait for transform timed out.");
    }
  }
}

void ContactControl::toControlFrame(geometry_msgs::WrenchStamped &data)
{
  if(ftFrame != controlFrame)
  {
    geometry_msgs::Vector3Stamped force;
    geometry_msgs::Vector3Stamped torque;
    force.header.stamp = data.header.stamp;
    force.header.frame_id = ftFrame;
    force.vector.x = data.wrench.force.x;
    force.vector.y = data.wrench.force.y;
    force.vector.z = data.wrench.force.z;
    torque.header.stamp = data.header.stamp;
    torque.header.frame_id = ftFrame;
    torque.vector.x = data.wrench.torque.x;
    torque.vector.y = data.wrench.torque.y;
    torque.vector.z = data.wrench.torque.z;
    if(listener->waitForTransform(controlFrame, data.header.stamp, ftFrame, data.header.stamp, worldFrame, ros::Duration(1.0)))
    {
      listener->transformVector(controlFrame, data.header.stamp, force, worldFrame, force);
      listener->transformVector(controlFrame, data.header.stamp, torque, worldFrame, torque);
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

void ContactControl::setMovement(Contact::Dimension dim, double vMax, double ftMax, double dMax)
{
  direction[dim].setMovement(vMax, ftMax, dMax, mi->getCurrentPose());
}

void ContactControl::setSpring(Contact::Dimension dim, double k, double b, double ftMax)
{
  direction[dim].setSpring(k, b, ftMax, mi->getCurrentPose());
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

void ContactControl::ftCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  ftData.header = msg->header;
  ftData.wrench = msg->wrench;
  toControlFrame(ftData);
}

MoveInterface* ContactControl::getMI()
{
  return mi;
}

void ContactControl::reset()
{
  for(int i = 0; i<Contact::NUM_DIMS; i++)
    direction[i].reset();
}
