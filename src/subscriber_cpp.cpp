#include <ros/ros.h>

#include "performance_tests/Frequency.h"
#include "performance_tests/SuperAwesome.h"
#include "performance_tests/frequency_handler.h"
#include "performance_tests/throttled_publisher.h"

using namespace performance_tests;

class Subscriber {
 public:
  Subscriber();

 private:
  void testCallback(const SuperAwesomeConstPtr& msg);

  ros::AsyncSpinner spinner_;
  ros::NodeHandle nh_;
  ros::Subscriber test_subscriber_;
  FrequencyHandler frequency_handler_;
  ThrottledPublisher<Frequency> freq_publisher_;
};

Subscriber::Subscriber() : spinner_(2), frequency_handler_(10) {
  spinner_.start();
  test_subscriber_ = nh_.subscribe("test", 1, &Subscriber::testCallback, this);
  freq_publisher_ = ThrottledPublisher<Frequency>(nh_, ros::Duration(1),
                                                  "subscriber_frequency");
}

void Subscriber::testCallback(const SuperAwesomeConstPtr& msg) {
  ros::Time time = ros::Time::now();
  double frequency = frequency_handler_.update(time);
  ROS_INFO_THROTTLE(1, "Subscriber frequency: %f", frequency);
  Frequency freq_msg;
  freq_msg.stamp = time;
  freq_msg.frequency = frequency;
  freq_publisher_.publish(freq_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subscriber");
  Subscriber subscriber;
  ros::waitForShutdown();
  return 0;
}
