#include <sstream>

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include "performance_tests/Frequency.h"
#include "performance_tests/PublisherConfig.h"
#include "performance_tests/SuperAwesome.h"
#include "performance_tests/frequency_handler.h"
#include "performance_tests/throttled_publisher.h"

using namespace performance_tests;

class Publisher {
  typedef performance_tests::PublisherConfig PublisherConfig;

 public:
  Publisher();
  void start();

 private:
  void reconfigureCallback(PublisherConfig& config, uint32_t level);

  ros::AsyncSpinner spinner_;
  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<PublisherConfig> reconfigure_server_;
  ros::Publisher test_publisher_;
  ros::Duration desired_period_;
  ros::Rate loop_rate_;
  FrequencyHandler frequency_handler_;
  ThrottledPublisher<Frequency> freq_publisher_;
};

Publisher::Publisher() : spinner_(2), loop_rate_(0), frequency_handler_(10) {
  spinner_.start();
  reconfigure_server_.setCallback(
      boost::bind(&Publisher::reconfigureCallback, this, _1, _2));
  test_publisher_ = nh_.advertise<SuperAwesome>("test", 1);
  freq_publisher_ = ThrottledPublisher<Frequency>(nh_, ros::Duration(1),
                                                  "publisher_frequency");
}

void Publisher::start() {
  while (ros::ok()) {
    ros::Time time = ros::Time::now();
    double frequency = frequency_handler_.update(time);
    ROS_INFO_THROTTLE(1, "Publisher frequency: %f", frequency);

    test_publisher_.publish(SuperAwesome());
    Frequency freq_msg;
    freq_msg.stamp = time;
    freq_msg.frequency = frequency;
    freq_publisher_.publish(freq_msg);

    ros::Duration duration = desired_period_ - (ros::Time::now() - time);
    if (duration > ros::Duration(0)) duration.sleep();
    // loop_rate_.sleep();
  }
}

void Publisher::reconfigureCallback(PublisherConfig& config,
                                    uint32_t /*level*/) {
  desired_period_ = ros::Duration(1.0 / config.publish_frequency);
  // loop_rate_ = ros::Rate(config.publish_frequency);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "publisher");
  Publisher publisher;
  publisher.start();
  return 0;
}
