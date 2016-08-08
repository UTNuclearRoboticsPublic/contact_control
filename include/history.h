#ifndef TIME_HISTORY_H_
#define TIME_HISTORY_H_
/**
 * The TimeHistory class keeps track of a variable over time.
 */
#include <ros/ros.h>
#include <math.h>
#include <queue>

class TimeHistory
{
public:
  /**
   * Constructor.
   */
  TimeHistory();

  /**
   * Destructor.
   */
  ~TimeHistory();

  /**
   * Initialize this TimeHistory object.
   * @param maxTime The max time to keep data before throwing it away.
   */
  void initialize(double maxTime);

  /**
   * Add the value of a tracked variable to the history and delete 
   * and values that have been kept longer than the maximum time.
   * @param time  The time that the recording of the variable occured at.
   * @param value The value of the variable when it was recorded.
   */
  void add(double time, double value);

  /**
   * Clear all data stored in this time history object
   */
  void clear();

  /**
   * Return the difference in values between the first and last data points.
   * @return The change in the variable.
   */
  double getDiff();

private:
  // Init variables
  bool               isInitialized;             /*!< True if object is initialized */
  double             length;                    /*!< Length of time to keep data */

  // History containers
  std::queue<double> time;                      /*!< Queue holding the time values */
  std::queue<double> value;                     /*!< Queue holding the value at a specific time */
};

#endif /* TIME_HISTORY_H_ */