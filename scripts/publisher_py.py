#!/usr/bin/python

import rospy
from dynamic_reconfigure.server import Server

from performance_tests.msg import SuperAwesome, Frequency
from performance_tests.cfg import PublisherConfig
from performance_tests_utils import *


class Publisher():
    def __init__(self):
        self.loop_rate = rospy.Rate(1)
        self.frequency_handler = FrequencyHandler(10)
        self.reconfigure_server = Server(PublisherConfig, self.reconfigureCallback)
        self.test_publisher = rospy.Publisher('test', SuperAwesome, queue_size=1)
        self.freq_publisher = ThrottledPublisher(1, Frequency, 'publisher_frequency')

    def reconfigureCallback(self, config, level):
        self.loop_rate = rospy.Rate(config.publish_frequency)
        return config

    def start(self):
        while not rospy.is_shutdown():
            time = rospy.Time.now()
            frequency = self.frequency_handler.update(time)
            rospy.loginfo_throttle(1, 'Publisher frequency: ' + str(frequency))

            self.test_publisher.publish(SuperAwesome())
            msg = Frequency()
            msg.stamp = time
            msg.frequency = frequency
            self.freq_publisher.publish(msg)

            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('publisher')
    publisher = Publisher()
    publisher.start()
