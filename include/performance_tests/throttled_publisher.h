#ifndef THROTTLED_PUBLISHER_H
#define THROTTLED_PUBLISHER_H

#include <ros/ros.h>

template <typename T>
class ThrottledPublisher {
 public:
  ThrottledPublisher() {}
  ThrottledPublisher(ros::NodeHandle &nh, const ros::Duration &max_frequency,
                     const std::string &topic) {
    init(nh, max_frequency, topic);
  }

  void init(ros::NodeHandle &nh, const ros::Duration &max_frequency,
            const std::string &topic) {
    period_ = ros::Duration(1.0 / max_frequency.toSec());
    publisher_ = nh.advertise<T>(topic, 1);
  }

  void publish(const T &msg) {
    if (ros::Time::now() < last_publish_time_ + period_) return;
    last_publish_time_ += period_;
    publisher_.publish(msg);
  }

 private:
  ros::Duration period_;
  ros::Publisher publisher_;
  ros::Time last_publish_time_;
};

#endif  // THROTTLED_PUBLISHER_H
