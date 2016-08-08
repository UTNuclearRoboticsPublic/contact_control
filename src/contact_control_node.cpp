#include "contact_control_node.h"

int main(int argc, char **argv)
{
  // Initialize the ros contact_control_node
  ROS_INFO("INIT ROS");
  ros::init(argc, argv, "contact_control_node");

  // Instantiate mover class
  ros::NodeHandle n; 
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ContactControlNode controller;
  controller.initialize();
  controller.run();

  return 0;
}

ContactControlNode::ContactControlNode() :
  moveGroup(""),
  worldFrame(""),
  ftFrame(""),
  controlFrame(""),
  currentState(STATE_END)
{
  for(int i = 0; i<initialPos.size(); i++)
  {
    initialPos[i] = 0.0;
  }
}

ContactControlNode::~ContactControlNode()
{
  delete mi;
}

void ContactControlNode::initialize()
{
  // Start spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Load config data
  if(n.getParam("config_data/move_group", moveGroup) &&
     n.getParam("config_data/world_frame", worldFrame) &&
     n.getParam("config_data/ft_frame", ftFrame) &&
     n.getParam("config_data/control_frame", controlFrame) &&
     n.getParam("config_data/init_pos", initialPos))
  {
    // Initialize classes TODO: make move group config variable
    cc.initialize(moveGroup, worldFrame, ftFrame, controlFrame);
    mi = cc.getMI();

    // Initialize state variable
    currentState = STATE_START;

    // Set planner configurations
    mi->setPlannerId("RRTConnectkConfigDefault");
  }
  else
  {
    ROS_ERROR("Unable to get config data from param server");
    currentState = STATE_CLEANUP;
  }
}

void ContactControlNode::run()
{
  while(STATE_END != currentState && ros::ok())
  {
    ROS_INFO_STREAM("Transitioning to next state " << getState(currentState));
    ROS_INFO("Press any key to continue..");
    std::cin.get();
    switch (currentState)
    {
      case STATE_START:
        start();
        break;
      case STATE_APPROACH:
        approachTask();
        break;
      case STATE_MOVE_TO_CONTACT:
        moveToContact();
        break;
      case STATE_CONTACT_TASK:
        contactMove();
        break;
      case STATE_VERIFY:
        verifyTask();
        break;
      case STATE_RETREAT:
        safeRetreat();
        break;
      case STATE_CLEANUP:
        cleanup();
        break;
      default:
        currentState = STATE_END;
        break;
    }
  }
  ros::shutdown();
}

void ContactControlNode::start()
{
  // Add collision scene

  // Update state
  currentState = STATE_APPROACH;
}

void ContactControlNode::approachTask()
{
  // Use move interface to approach the contact task space
  mi->moveJoints(initialPos, 1.0);

  // Start motoman_jogger now that the robot is not in singularity TODO: automate this
  ROS_INFO("Please start motoman_jogger");

  // Update state
  currentState = STATE_MOVE_TO_CONTACT;
}

void ContactControlNode::moveToContact()
{
  // Start the simulated force/torque data if available
  netft_utils::StartSim start_srv;
  start_srv.request.simDim = Contact::DIM_Y;
  start_srv.request.simType = 3;
  start_srv.request.simSlope = 4.0;
  start_srv.request.maxForce = 9.0;
  ros::service::call("/netft/start_sim", start_srv);

  // Use contact control class to move forward until contact
  // set up dimensions
  cc.setMovement(Contact::DIM_Z, -1.0, 20.0, 0.2);
  //cc.setSpring(Contact::DIM_X, k, b, 10.0);
  cc.setSpring(Contact::DIM_Y, 20.0, 9.0, 10.0);
  //cc.setFollower(Contact::DIM_Y, 10.0, 10.0);
  cc.move(200.0, 20.0);
  ros::Duration(1.0).sleep();
  ROS_INFO_STREAM("Final travel distance: " << cc.getTravel(Contact::DIM_Z));
  double ft;
  ros::Time time;
  cc.getFT(Contact::DIM_Z, ft, time);
  ROS_INFO_STREAM("Final force: " << ft);

  // Stop the simulated force torque data if it is running
  netft_utils::StopSim stop_srv;
  ros::service::call("/netft/stop_sim", stop_srv);

  // Update state
  currentState = STATE_CONTACT_TASK;
}

void ContactControlNode::contactMove()
{
  // Start the simulated force/torque data if available
  netft_utils::StartSim start_srv;
  start_srv.request.simDim = Contact::DIM_Z;
  start_srv.request.simType = 1;
  start_srv.request.simSlope = 1.0;
  start_srv.request.maxForce = 6.0;
  ros::service::call("/netft/start_sim", start_srv);

// Use contact control class to execute contact move
  cc.reset();
  cc.setMovement(Contact::DIM_RZ, 1.0, 2.0, 0.3);
  cc.setMovement(Contact::DIM_Z, -1.0, 10.0, 0.1);
  cc.move(20.0, 2.0);
  ros::Duration(1.0).sleep();
  ROS_INFO_STREAM("Final rotation angle: " << cc.getTravel(Contact::DIM_RZ));
  double ft;
  ros::Time time;
  cc.getFT(Contact::DIM_RZ, ft, time);
  ROS_INFO_STREAM("Final torque: " << ft);

  // Stop the simulated force torque data if it is running
  netft_utils::StopSim stop_srv;
  ros::service::call("/netft/stop_sim", stop_srv);

  // Update state
  currentState = STATE_VERIFY;
}

void ContactControlNode::verifyTask()
{
  // Make sure we are in the final position
  // Update state
  currentState = STATE_RETREAT;
}

void ContactControlNode::safeRetreat()
{
  // Retreat from contact paying attention to forces
  // Update state
  currentState = STATE_CLEANUP;
}

void ContactControlNode::cleanup()
{
  // Remove collision scene
  // Update state
  currentState = STATE_END;
}

std::string ContactControlNode::getState(State state)
{
  std::string stateStr;
  switch(state)
  {
    case STATE_START:
       stateStr = "Start State";
       break;
    case STATE_APPROACH:
       stateStr = "Approach State";
       break;
    case STATE_MOVE_TO_CONTACT:
       stateStr = "Move to Contact State";
       break;
    case STATE_CONTACT_TASK:
       stateStr = "Contact Task State";
       break;
    case STATE_VERIFY:
       stateStr = "Verify State";
       break;
    case STATE_RETREAT:
       stateStr = "Retreat State";
       break;
    case STATE_CLEANUP:
       stateStr = "Cleanup State";
       break;
    case STATE_END:
       stateStr = "End State";
       break;
    default:
       stateStr = "Unknown State";
       break;
  }
  return stateStr;
}
