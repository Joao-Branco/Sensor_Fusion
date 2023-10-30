#! /usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_fusion.msg import target_position
from MARS_msgs.msg import TargetTelemetry
from std_msgs.msg import Float64


class RelayNode:
    def __init__(self):
        rospy.init_node('relay_node')
        
        self.uav_id = rospy.get_namespace()  # Extract the UAV ID from the node's namespace
        self.uav_total = rospy.get_param('/total_uav')  # Get the total number of UAVs from the parameter server
        self.source_topic = f'/uav{self.uav_id}/target_position_throttle'
        self.destination_topic = f'/uav{self.uav_id}/target_position_geolocation'

        self.publisher = rospy.Publisher(self.destination_topic, TargetTelemetry, queue_size=10)
        self.subscribers = []

    def start(self):
        rospy.loginfo(f"Relay node for {self.uav_id} is running.")
        self.setup_subscribers()
        rospy.spin()

    def setup_subscribers(self):
        for i in range(self.uav_total):
            if i != self.uav_id:
                topic = f'/uav{i}/target_position_throttle'
                subscriber = rospy.Subscriber(topic, TargetTelemetry, self.callback, callback_args=i)
                self.subscribers.append(subscriber)

    def callback(self, data, uav_id):
        if uav_id != self.uav_id:
            self.publisher.publish(data)

if __name__ == '__main__':
    try:
        relay_node = RelayNode()
        relay_node.start()
    except rospy.ROSInterruptException:
        pass