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
   * available.
   */
  enum Movement    {DIRECTION = 0,  /**< The directional control law. */
                    SPRING    = 1,  /**< The static spring based control law. */
                    FOLLOWER  = 2,  /**< The follower control law. */
                    NUM_LAWS  = 3}; /**< The number of laws. */
}
