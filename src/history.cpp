#include "history.h"

TimeHistory::TimeHistory() :
  isInitialized(false),
  length(0.0)
{
  // Do nothing
}

TimeHistory::~TimeHistory()
{
  // Do nothing
}

void TimeHistory::initialize(double maxTime)
{
  length = maxTime;
  isInitialized = true;
}

void TimeHistory::add(double inTime, double inValue)
{
  time.push(inTime);
  value.push(inValue);

  bool purged = false;
  while(!purged)
  {
    if(time.front() == time.back())
    {
      purged = true;
    }
    else if((time.front() + length) > time.back())
    {
      time.pop();
      value.pop();
    }
    else
    {
      purged = true;
    }
  }
}

void TimeHistory::clear()
{
  while(time.size() > 0 && value.size() > 0)
  {
    time.pop();
    value.pop();
  }
}

double TimeHistory::getDiff()
{
  return fabs(value.front() - value.back());
}

