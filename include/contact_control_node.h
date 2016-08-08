/**
 * The contact control node defines a state machined used to perform contact tasks.
 */
#include <ros/ros.h>
#include <contact_control.h>
#include <netft_utils/SetBias.h>
#include <string>
#include <vector>

class ContactControlNode
{
public:
  /**
   * State enum.
   * The state enum defines the states of the state machine.
   */
  enum State       {STATE_START           = 0,  /**< The start state. */
                    STATE_APPROACH        = 1,  /**< The approach contact state. */
                    STATE_MOVE_TO_CONTACT = 2,  /**< The move into contact state. */
                    STATE_CONTACT_TASK    = 3,  /**< The perform contact task state. */
                    STATE_VERIFY          = 4,  /**< The verify move state. */
                    STATE_RETREAT         = 5,  /**< The retreat state. */
                    STATE_CLEANUP         = 6,  /**< The cleanup state. */
                    STATE_END             = 7}; /**< The end state. */

public:
  /**
   * Constructor.
   */
  ContactControlNode();

  /**
   * Destructor.
   */
  ~ContactControlNode();

  /**
   * Initialize this contact control state machine and contained objects.
   */
  void initialize();

   /**
   * Run the contact control state machine.
   */
  void run();

private:
  // Config data
  std::string moveGroup;                            /*!< The name of the moveit move group */
  std::string worldFrame;                           /*!< The name of the world frame */
  std::string ftFrame;                              /*!< The name of the force/torque sensor frame */
  std::string controlFrame;                         /*!< The name of the control frame */
  std::vector<double> initialPos;                   /*!< The joint positions of the setup move */

  // Objects
  ros::NodeHandle n;                                /*!< ROS node handle */
  ContactControl cc;                                /*!< Class that executes contact task moves */
  MoveInterface* mi;                                /*!< Pointer to the interface to move the robot via moveit */

  // State variables
  State currentState;                               /*!< The current state of the state machine */

  // Actions or states
  void start();                                     /*!< Initialize and start the process */
  void approachTask();                              /*!< Approach the contact surface using moveit to avoid collisions */
  void moveToContact();                             /*!< Cartesian move to contact */
  void contactMove();                               /*!< Execute contact move */
  void verifyTask();                                /*!< Verify that the move was completed successfully */
  void safeRetreat();                               /*!< Retreat safely by monitoring FT */
  void cleanup();                                   /*!< Clean up when done */

  // Convenience methods
  /**
   * Return string value of state enum 
   * @param state The state in enum form.
   * @return      The state in string form.
   */
  std::string getState(State state);
};

