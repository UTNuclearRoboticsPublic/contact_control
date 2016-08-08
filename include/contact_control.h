/**
 * Generalized contact control class. Allows you to control each dimension with a different control law.
 */

#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>
#include <geometry_msgs/WrenchStamped.h>
#include <move_interface.h>
#include <tf/transform_listener.h>
#include <motoman_jogger/Deltas.h>
#include <netft_utils/SetBias.h>
#include <netft_utils/StartSim.h>
#include <netft_utils/StopSim.h>
#include <std_msgs/Float64.h>
#include <contact_direction.h>

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
   * @param mg  The move group of the robot to be controlled.
   * @param wf  The name of the world frame.
   * @param ftf The name of the force/torque sensor frame.
   * @param cf  The name of the control frame that velocity commands are inputed in.
   */
  void initialize(std::string mg, std::string wf, std::string ftf, std::string cf);

  // One dimensional move
  //void move(Contact::Dimension dim, double vMax, double ftMax, double dMax);

  /**
   * Sets chosen dimension to move in a direction and act as a spring to applied forces.
   * @param dim     The dimension you are setting the control law for.
   * @param vMax    The maximum velocity to move in this direction (0-1).
   * @param ftMax   The max force allowed in this direction.
   * @param dMax    The maximum distance to travel in this direction.
   */
  void setMovement(Contact::Dimension dim, double vMax, double ftMax, double dMax);

  /**
   * Sets chosen dimension to act as a spring
   * @param dim     The dimension you are setting the control law for.
   * @param k       The spring constant.
   * @param b       The damping coefficient.
   * @param ftMax   The max force allowed in this direction.
   */
  void setSpring(Contact::Dimension dim, double k, double b, double ftMax);

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
   */
  void move(double fMax, double tMax);

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
   * Reset the control patterns for all dimensions.
   */
  void reset();

  /**
   * Return a pointer to the move interface instance.
   * return    The pointer to the active move interface instance.
   */
  MoveInterface* getMI();

private:
  // Config data
  std::string worldFrame;                          /*!< The name of the world frame */
  std::string ftFrame;                             /*!< The name of the force/torque sensor frame */
  std::string controlFrame;                        /*!< The name of the control frame */

  // Objects
  ros::NodeHandle n;                               /*!< The ROS node handle for this instance */
  tf::TransformListener* listener;                 /*!< Pointer to a transform listener */
  MoveInterface* mi;                               /*!< Pointer to the robot move interface */
  ContactDirection direction[Contact::NUM_DIMS];   /*!< Array of Contact direction objects */

  // Data
  geometry_msgs::PoseStamped startPose;            /*!< The start pose of the current move */
  geometry_msgs::WrenchStamped ftData;             /*!< ftData received from netft_utils in tool frame */

  // ROS publishers
  ros::Publisher delta_pub;                        /*!< Publisher for commanded velocities */
  ros::Publisher data_pub;                         /*!< Convenience publisher for data to be plotted */

  // ROS subscribers
  ros::Subscriber ft_sub;                          /*!< Subscriber to get the netft data */
  ros::Subscriber netft_cancel_sub;                /*!< Subscirber to the netft cancel command */
  bool netftCancel;                                /*!< Boolean to accept netft cancel command */

  // Callback functions
  void ftCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void cancelCallback(const netft_utils::Cancel::ConstPtr& msg);

  // Convenience methods
  /**
   * Convert the jogging deltas to the world frame.
   * @param cmd     The velocity command
   * @param time    The time to transform the command
   */
  void toWorldFrame(motoman_jogger::Deltas &cmd, ros::Time time);

  /**
   * Convert the force data to the control frame.
   * @param data    The force data
   */
  void toControlFrame(geometry_msgs::WrenchStamped &data);

};

