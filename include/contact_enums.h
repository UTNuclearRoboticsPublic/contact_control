#ifndef CONTACT_ENUMS_H_
#define CONTACT_ENUMS_H_
/**
 * Enums and constants for the Contact Control library
 */

namespace Contact
{
  /**
   * Dimension enum.
   * The Dimension enum is used to determine the dimension that is being
   * controlled or requested.
   */
  enum Dimension   {DIM_X      = 0,  /**< The X dimension. */
                    DIM_Y      = 1,  /**< The Y dimension. */
                    DIM_Z      = 2,  /**< The Z dimension. */
                    DIM_RX     = 3,  /**< The rotational X dimension. */
                    DIM_RY     = 4,  /**< The rotational Y dimension. */
                    DIM_RZ     = 5,  /**< The rotational Z dimension. */
                    NUM_DIMS   = 6}; /**< The number of dimensions. */

  /**
   * Perspective enum.
   * The Perspective enum is used to differentiate between translational dimensions
   * and rotational dimensions.
   */
  enum Perspective {TRANSLATION = 0,       /**< The translational perspective or forces. */
                    ROTATION    = 1,       /**< The rotational persepective or torques. */
                    NUM_PERSPECTIVES = 2}; /**< The number of perspectives. */

  /**
   * Movement enum.
   * The movement enum differentiates between the different control laws that are 
   * available. When new control laws are added the the framework, this enum should
   * be updated accordingly.
   */
  enum Movement    {DIRECTION = 0,  /**< The directional control law. */
                    SPRING    = 1,  /**< The static spring based control law. */
                    FOLLOWER  = 2,  /**< The follower control law. */
                    NUM_LAWS  = 3}; /**< The number of laws. */

  /**
   * End condition enum.
   * The end condtion enum tells why the move was ended. As new end conditions are added
   * to the framework, this enum should be updated accordingly.
   */
  enum EndCondition  {NO_CONDITION    = 0,   /**< A move is being executed and there is not yet an end condition. */
                      NO_LAWS         = 1,   /**< No dimensions were initialized with a control law to follow. */
                      FT_VIOLATION    = 2,   /**< Force or torque was read as maximum allowed. */
                      MAX_TRAVEL      = 3,   /**< Maximum allowed travel was reached. */
                      EXTERNAL        = 4,   /**< Stop move function was called. */
                      DIFF_VIOLATION  = 5,   /**< Force torque differential max was reached */
                      NO_FT_DATA      = 6,   /**< Netft is not reading force data */
                      IN_MOTION       = 7,   /**< Another move is executing */
                      INVALID_PARAMS  = 8,   /**< Invalid parameters passed to method */
                      NOT_INITIALIZED = 9,   /**< Invalid parameters passed to method */
                      NUM_CONDITIONS  = 10}; /**< The number of end conditions. */
}
#endif /* CONTACT_ENUMS_H_ */