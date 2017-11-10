#!/usr/bin/python

from collections import deque

import rospy

class MeanFilter:
    def __init__(self, window_size):
        self.data = deque(maxlen=window_size)

    def filter(self, value):
        self.data.append(value)
        return sum(self.data) / len(self.data)


class FrequencyHandler:
    def __init__(self, window_size):
        self.frequency_filter = MeanFilter(window_size)
        self.last_time = rospy.Time.now()

    def update(self, time):
        period = time - self.last_time
        if period == rospy.Duration(0): return 0
        new_frequency = 1 / period.to_sec()
        self.last_time = time
        return self.frequency_filter.filter(new_frequency)


class ThrottledPublisher:
    def __init__(self, max_frequency, data_class, topic):
        self.period = rospy.Duration(1.0 / max_frequency)
        self.publisher = rospy.Publisher(topic, data_class, queue_size=1)
        self.last_publish_time = rospy.Time(0)

    def publish(self, msg):
        if rospy.Time.now() < self.last_publish_time + self.period: return
        self.last_publish_time += self.period
        self.publisher.publish(msg)


