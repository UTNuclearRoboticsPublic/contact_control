/**
 * Generalized contact control class for a single dimension.
 */

#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <contact_enums.h>
#include <history.h>

class ContactDirection
{
public:
  /**
   * Constructor.
   */
  ContactDirection();

  /**
   * Destructor.
   */
  ~ContactDirection();

  /**
   * Initialize this instance of ContactDirection.
   * @param dir  The direction that this instance controls the behavior of.
   * @param wf   The name of the world frame.
   * @param cf   The name of the control frame.
   * @param list The pointer to the transform listener.
   */
  void initialize(Contact::Dimension dir, std::string wf, std::string cf, tf::TransformListener* list);

  /**
   * Sets the dimension to move in a direction and act as a spring to applied forces.
   * @param vMax    The maximum velocity to move in this direction (0-1).
   * @param ftMax   The max force allowed in this direction.
   * @param dMax    The maximum distance to travel in this direction.
   * @param pose    The starting pose used to compute travel distances.
   */
  void setMovement(double vMax, double ftMax, double dMax, geometry_msgs::PoseStamped pose);

  /**
   * Sets the dimension to act as a spring
   * @param k       The spring constant.
   * @param b       The damping coefficient.
   * @param ftMax   The max force allowed in this direction.
   * @param pose    The starting pose used to compute travel distances.
   */
  void setSpring(double k, double b, double ftMax, geometry_msgs::PoseStamped pose);

  /**
   * Sets the dimension to act as a force follower
   * @param b       The damping coefficient. Set as 1/(applied force that should yield the max speed).
   * @param ftMax   The max force allowed in this direction.
   * @param pose    The starting pose used to compute travel distances.
   */
  void setFollower(double b, double ftMax, geometry_msgs::PoseStamped pose);

  /**
   * Gets the velocity to be commanded to the robot in this direction.
   * @param ft      The current force/torque reading in this direction.
   * @param pose    The current pose of the robot.
   * @return        The velocity to be commanded to the robot.
   */
  double getVelocity(double ft, geometry_msgs::PoseStamped pose);

  /**
   * Reset the direction's control behavior.
   */
  void reset();

  /**
   * Check if an end condition has been met.
   * @param ft      The current force/torque reading in this direction.
   * @param pose    The current pose of the robot.
   * @return        True if an end condition has been met.
   */
  bool getCondition(double ft, geometry_msgs::PoseStamped pose);

  /**
   * Get the current amount traveled in this dimension since the start of the move.
   * @param pose    The current pose of the robot.
   * @return        The travel since the start of the move.
   */
  double getTravel(geometry_msgs::PoseStamped);

  /**
   * Check the amount of travel left and modify the velocity if we are close to the end of requested travel.
   * @param velocity The velocity to be commanded to the robot.
   * @param pose     The current pose of the robot.
   * @return         The modified velocity to be commanded to the robot.
   */
  double checkSpeed(double velocity, geometry_msgs::PoseStamped pose);

  /**
   * Set the value of force/torque differential that will be considered an end condition.
   * This method is optional. The class defaults to ignoring FT differentials.
   * @param diffMax   The maximum change in force or torque.
   * @param time      The time span to evaluate the change in.
   */
  void setMaxDiff(double diffMax, double time);

  /**
   * Check if the maximum force/torque differential has been violated.
   * @return True if a violation has occured.
   */
  bool checkDiff();

  /**
   * Write current ft data to the Force/Torque history object.
   * @param ft  The current force/torque data.
   */
  void writeFT(double ft);

  /**
   * Clear the force/torque history that is used to check for a differential violation.
   */
  void clearFT();

private:
  // Init variables
  bool                        isInitialized;    /*!< True if object is initialized */
  Contact::Dimension          direction;        /*!< Direction being controlled */
  geometry_msgs::PoseStamped  startPose;        /*!< Start pose of the current movement */

  // End condition variables
  double      velocityMax;                      /*!< Maximium velocity allowed in this direction */
  double      forceTorqueMax;                   /*!< Maximum force or torque allowed in this direction */
  double      displacementMax;                  /*!< Maximum dispacement allowed in this direction */
  bool        endOnDiff;                        /*!< True if we want to terminate on sharp ft differential */
  double      ftDiffMax;                        /*!< Maximum force/torque differential */
  double      ftDiffTime;                       /*!< Elapsed time to evaluate force/torque differential */
  TimeHistory ftHistory;                        /*!< FT history for differential check */

  // Behavior variables
  double            springConstant;             /*!< Spring constant for impedance control */
  double            dampingCoeff;               /*!< Damping coefficient for impedance control */
  Contact::Movement movementType;               /*!< Tells which behavior law this dimension is acting on */
  bool              isReady;                    /*!< True if a control scheme has been picked and initialized */

  // Listener
  tf::TransformListener* listener;              /*!< Pointer to a transform listener */

  // Frames
  std::string       controlFrame;               /*!< Name of the conrtol frame */
  std::string       worldFrame;                 /*!< Name of the world frame */

  /**
   * Set the start pose that is used to evaluate travel distances.
   * @param pose The start pose.
   */
  void setStart(geometry_msgs::PoseStamped pose);
};

