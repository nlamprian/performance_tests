#!/usr/bin/python

import rospy

from performance_tests.msg import SuperAwesome, Frequency
from performance_tests_utils import *


class Subscriber():
    def __init__(self):
        self.frequency_handler = FrequencyHandler(10)
        self.test_subscriber = rospy.Subscriber('test', SuperAwesome,
                                                self.testCallback, queue_size=1)
        self.freq_publisher = ThrottledPublisher(1, Frequency, 'subscriber_frequency')

    def testCallback(self, msg):
        time = rospy.Time.now()
        frequency = self.frequency_handler.update(time)
        rospy.loginfo_throttle(1, 'Subscriber frequency: ' + str(frequency))
        freq_msg = Frequency()
        freq_msg.stamp = time
        freq_msg.frequency = frequency
        self.freq_publisher.publish(freq_msg)


if __name__ == '__main__':
    rospy.init_node('subscriber')
    subscriber = Subscriber()
    rospy.spin()
