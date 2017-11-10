#ifndef FREQUENCY_HANDLER_H
#define FREQUENCY_HANDLER_H

#include <ros/ros.h>

#include "performance_tests/mean_filter.h"

class FrequencyHandler {
 public:
  FrequencyHandler(size_t acc_window_size)
      : frequency_filter_(acc_window_size), last_time_(ros::Time::now()) {}

  double update(const ros::Time &time) {
    ros::Duration period = time - last_time_;
    if (period == ros::Duration(0)) return 0;
    double new_frequency = 1 / period.toSec();
    last_time_ = time;
    return frequency_filter_.filter(new_frequency);
  }

 private:
  MeanFilter frequency_filter_;
  ros::Time last_time_;
};

#endif  // FREQUENCY_HANDLER_H
