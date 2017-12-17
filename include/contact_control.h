#ifndef CONTACT_CONTROL_H
#define CONTACT_CONTROL_H

/**
 * Generalized contact control class. Allows you to control each dimension with a different control law.
 */

#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>
#include <geometry_msgs/WrenchStamped.h>
#include <move_interface/move_interface.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <netft_utils/SetBias.h>
#include <netft_utils/StartSim.h>
#include <netft_utils/StopSim.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <contact_direction.h>
#include <future>
#include <netft_utils_lean.h>
#include <contact_enums.h>

class ContactControl
{
public:
  /**
   * Constructor.
   */
  ContactControl();

  /**
   * Destructor.
   */
  ~ContactControl();

  /**
   * Initialize this instance of ContactControl.
   * @param mg        The move group of the robot to be controlled.
   * @param ff        The name of the fixed frame.
   * @param vf        The name of the velocity command frame.
   * @param ftf       The name of the force/torque sensor frame.
   * @param cf        The name of the control frame that velocity commands are inputed in.
   */
  void initialize(std::string mg, std::string ff, std::string vf, std::string ftf, std::string cf);

  /**
   * Sets spring constant.
   * @param dim       The dimension of control the spring constant will be readjusted on.
   * @param k         The spring constant.
   */
  void adjustSpringConstant(Contact::Dimension dim, double k);

  /**
   * Sets chosen dimension to move in a direction and act as a spring to applied forces.
   * @param dim     The dimension you are setting the control law for.
   * @param vMax    The maximum velocity to move in this direction (0-1).
   * @param ftStall The force or torque that will cause no motion in this direction.
   * @param dMax    The maximum distance to travel in this direction.
   * @param ftMax   The max force allowed in this direction.
   */
  void setMovement(Contact::Dimension dim, double vMax, double ftStall, double dMax, double ftMax);

  /**
   * Sets chosen dimension to act as a spring
   * @param dim       The dimension you are setting the control law for.
   * @param k         The spring constant.
   * @param b         The damping coefficient.
   * @param posOffset The position offset for the zero position of the virtual spring.
   * @param ftMax     The max force allowed in this direction.
   */
  void setSpring(Contact::Dimension dim, double k, double b, double posOffset, double ftMax);

  /**
   * Sets chosen dimension to act as a spring
   * @param dim       The dimension you are setting the control law for.
   * @param k         The spring constant.
   * @param b         The damping coefficient.
   * @param posOffset The position offset for the zero position of the virtual spring.
   * @param dMax      The maximum distance to travel in this direction.
   * @param ftMax     The max force allowed in this direction.
   */
  void setSpring(Contact::Dimension dim, double k, double b, double posOffset, double dMax, double ftMax);

  /**
   * Sets chosen dimension to act as a force follower
   * @param dim     The dimension you are setting the control law for.
   * @param b       The damping coefficient. Set as 1/(applied force that should yield the max speed).
   * @param ftMax   The max force allowed in this direction.
   */
  void setFollower(Contact::Dimension dim, double b, double ftMax);

  /**
   * Start a move. Dimensions will be controlled and move will be terminated
   * by conditions set in setter methods.
   * @param fMax   The magnitude of maximum combined force allowed
   * @param tMax   The magnitude of maximum combined torque allowed
   * @param vMax   The maximum velocity allowed for any direction of this move
   * @return       The reason that the move was ended
   */
  Contact::EndCondition move(double fMax = 0.0, double tMax = 0.0, double vMax = 1.0);

  /**
   * Start a move async. This allows evaluation of the end condition outside of contact control.
   * @param fMax   The magnitude of maximum combined force allowed
   * @param tMax   The magnitude of maximum combined torque allowed
   * @param vMax   The maximum velocity allowed for any direction of this move
   * @return       Future of the end condition variable
   */
  std::future<Contact::EndCondition> moveAsync(double fMax = 0.0, double tMax = 0.0, double vMax = 1.0);

  /**
   * Balance an object on a surface by moving in the direction of gravity and following in all others.
   * Torque caused by force on surface will be subtracted from follower in that direction.
   * Note: This functionality is untested.
   * @param lDim     A Contact::Direction variable indicating the dimension the lever arm is in
   * @param fDim     A Contact::Direction variable indicating the dimension the contact force is in
   * @param inWorld  True if lever dimension is specified in world frame
   * @param leverArm The distance from the ft_sensor frame to the line of contact (can be negative)
   * @param mDim     A Contact::Direction variable indicating the dimension that movement should take place in
   * @param mSpeed   A -1 to 1 speed of the movement
   * @param fMax     The magnitude of maximum combined force allowed
   * @param tMax     The magnitude of maximum combined torque allowed
   * @return         The reason that the move was ended
   */
  Contact::EndCondition balance(Contact::Dimension lDim, Contact::Dimension fDim, bool inWorld, double leverArm, Contact::Dimension mDim, double mSpeed, double fMax, double tMax);

  /**
   * Balance an object async. This allows evaluation of the end condition outside of contact control.
   * Note: This functionality is untested.
   * @param lDim     A Contact::Direction variable indicating the dimension the lever arm is in
   * @param fDim     A Contact::Direction variable indicating the dimension the contact force is in
   * @param inWorld  True if lever dimension is specified in world frame
   * @param leverArm The distance from the ft_sensor frame to the line of contact (can be negative)
   * @param mDim     A Contact::Direction variable indicating the dimension that movement should take place in
   * @param mSpeed   A -1 to 1 speed of the movement
   * @param fMax     The magnitude of maximum combined force allowed
   * @param tMax     The magnitude of maximum combined torque allowed
   * @return         Future of end condition variable
   */
  std::future<Contact::EndCondition> balanceAsync(Contact::Dimension lDim, Contact::Dimension fDim, bool inWorld, double leverArm, Contact::Dimension mDim, double mSpeed, double fMax, double tMax);

  /**
   * Set the value of force/torque differential that will be considered an end condition.
   * This method is optional. The class defaults to ignoring FT differentials.
   * @param dim       The dimension you are setting the end condition for.
   * @param diffMax   The maximum change in force or torque.
   * @param time      The time span to evaluate the change in.
   */
  void setMaxDiff(Contact::Dimension dim, double diffMax, double time);

  /**
   * Check if the maximum force/torque differential has been violated.
   * Note: This functionality is untested.
   * @param dim    The dimension you are setting the end condition for.
   * @return       True if a violation has occured.
   */
  bool checkDiff(Contact::Dimension dim);

  /**
   * Get the current amount traveled in this dimension since the start of the move.
   * @param dim    The dimension you are getting the travel for.
   * @return       The travel since the start of the move.
   */
  double getTravel(Contact::Dimension dim);

  /**
   * Get the current amount traveled in this perspective (translational or rotational)
   * since the start of the move.
   * @param perspective   The perspective you are getting the travel for.
   * @return              The travel since the start of the move.
   */
  double getTravel(Contact::Perspective perspective);

  /**
   * Get the force or torque reading in the dimension specfied.
   * @param dim    The dimension to get the reading for.
   * @param ft     The force or torque reading.
   * @param time   The time the reading occured.
   */
  void getFT(Contact::Dimension dim, double &ft, ros::Time &time);

  /**
   * Get the force or torque reading in the perspective (translational
   * or rotational) specfied.
   * @param perspective  The perspective to get the reading for.
   * @param ft           The force or torque reading.
   * @param time         The time the reading occured.
   */
   void getFT(Contact::Perspective perspective, double &ft, ros::Time &time);

   /**
    * Stops the move.
    */
   void stopMove();

  /**
   * Reset the control patterns for all dimensions.
   */
  void reset();

  /**
   * Return a pointer to the move interface instance.
   * return    The pointer to the active move interface instance.
   */
  MoveInterface* getMI();

  /**
   * Return a pointer to the force/torque interface instance.
   * return    The pointer to the force/torque interface.
   */
  NetftUtilsLean* getFTI();

  /**
   * Set the topic that force/torque data is read from.
   * @param ftTop      The force/torque data topic.
   */
  void setFTTopic(std::string ftTop);

  /**
   * Set the IP address of a force torque sensor that
   * netFT utils can read data from.
   * @param ftAdd      The force/torque sensor IP address.
   */
  void setFTAddress(std::string ftAdd);

  /**
   * Set the topic to output velocity commands to.
   * @param velTopic    The velocity jog command topic.
   */
  void setVelTopic(std::string velTop);

  /**
   * Set the control rate of the contact control framework.
   * @param cRate       The control rate for the framework.
   */
  void setControlRate(double cRate);

private:
  // Config data
  std::string fixedFrame;                          /*!< The name of the fixed frame */
  std::string velFrame;                            /*!< The name of the velocity command frame */
  std::string ftFrame;                             /*!< The name of the force/torque sensor frame */
  std::string controlFrame;                        /*!< The name of the control frame */
  std::string ftAddress;                           /*!< The address of the force/torque sensor */
  std::string ftTopic;                             /*!< The force/torque data topic */
  std::string velTopic;                            /*!< The velocity jog command topic */
  double controlRate;                              /*!< The control rate of the framework */

  // Objects
  ros::NodeHandle n;                              /*!< The ROS node handle for this instance */
  ros::AsyncSpinner* spinner;                      /*!< The ROS async spinner for this instance */
  tf::TransformListener* listener;                 /*!< Pointer to a transform listener */
  MoveInterface* mi;                               /*!< Pointer to the robot move interface */
  ContactDirection direction[Contact::NUM_DIMS];   /*!< Array of Contact direction objects */
  NetftUtilsLean* fti;                             /*!< Pointer to the force torque interface */

  // Data
  geometry_msgs::PoseStamped startPose;            /*!< The start pose of the current move */
  geometry_msgs::WrenchStamped ftData;             /*!< ftData received from netft_utils in control frame */
  geometry_msgs::WrenchStamped ftDataWorld;        /*!< ftData received from netft_utils in world frame */
  Contact::EndCondition endCondition;              /*!< Enum to tell if and why a move should be stopped */
  bool                  isMoving;                  /*!< Bool to ensure 2 moves are not being executed at once */
  bool                  gravBalance;               /*!< True if we are trying to balance an object */
  Contact::Dimension    leverDim;                  /*!< Dimension that the lever arm of the balance procedure is in */
  Contact::Dimension    forceDim;                  /*!< Dimension that the contact force is acting in if we are balancing */
  bool                  leverWorld;                /*!< True if lever is specified in world frame dimensions */
  double                gravLever;                 /*!< Length from FT sensor to balance contact force in meters */
  bool                  monitorFT;                 /*!< True if we are monitoring the FT sensor */
  bool                  isInit;                    /*!< True if this instance has been initialized correctly */

  // ROS publishers
  ros::Publisher delta_pub;                        /*!< Publisher for commanded velocities */
  ros::Publisher data_pub;                         /*!< Convenience publisher for data to be plotted */

  // ROS subscribers
  ros::Subscriber netft_cancel_sub;                /*!< Subscirber to the netft cancel command */
  bool netftCancel;                                /*!< Boolean to accept netft cancel command */

  // Callback functions
  /**
   * Function that monitors the FT sensor
   */
  bool ftMonitor();

  /**
   * Callback to the netft cancel message
   * @param msg Incoming cancel message.
   */
  void cancelCallback(const netft_utils::Cancel::ConstPtr& msg);

  // Convenience methods
  /**
   * Convert the jogging deltas to the velocity command frame.
   * @param cmd     The velocity command
   * @param time    The time to transform the command
   */
  void toVelFrame(std::vector<double> &cmd, ros::Time time);

  /**
   * Convert the force data to the control frame.
   * @param data    The force data
   */
  void toControlFrame(geometry_msgs::WrenchStamped &data);

};
#endif
