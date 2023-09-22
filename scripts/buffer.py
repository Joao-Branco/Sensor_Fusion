#! /usr/bin/env python

import rospy
import numpy as np
from sensor_fusion.msg import target_position_fuse
from sensor_fusion.msg import target_position
from std_msgs.msg import Float64


class Throttle:
    def __init__(self, uav_id):
        self.id = uav_id
        self.buffer = []
        self.sub_throttle = rospy.Subscriber('/uav' + str(uav_id) + '/target_position_throttle', target_position_fuse, self.target_callback_throttle)

    def target_callback_throttle(self, msg):
        self.buffer.append(msg)
        

class Positions:
    def __init__(self, uav_id, UAVDistance):
        self.id = uav_id
        self.UAVDistance = UAVDistance
        self.sub_position = rospy.Subscriber('/uav' + str(uav_id) + '/position', target_position, self.position_callback)

    def position_callback(self, msg):
        self.UAVDistance.update_distance(uav_index= self.id, x= msg.x, y= msg.y)
        self.UAVDistance.update_delay_matrix()

        print(self.UAVDistance.get_distance_matrix())

    
class UAVDistanceMatrix:
    def __init__(self, num_uavs, delay_constant):
        self.num_uavs = num_uavs
        self.distances = np.zeros((num_uavs, num_uavs))
        self.delays = np.zeros((num_uavs, num_uavs))
        self.delay_constant = delay_constant

    def update_distance(self, uav_index, x, y):
        uav_index = uav_index - 1
        rospy.loginfo("uav_index------%f", uav_index)
        if uav_index < 0 or uav_index >= self.num_uavs:
            raise ValueError("Invalid UAV index")

        for i in range(self.num_uavs):
            if i == uav_index:
                continue  # Skip the same UAV
            distance = np.sqrt((x - self.distances[i, 0]) ** 2 + (y - self.distances[i, 1]) ** 2)
            self.distances[uav_index, i] = distance
            self.distances[i, uav_index] = distance

    def update_delay_matrix(self):
        self.delays = self.delay_constant * self.distances

    def get_distance_matrix(self):
        return self.distances

    def get_delay_matrix(self):
        return self.delays

    


        
# def target_callback_delay(msg):
#     buffer.append(msg)


if __name__ == "__main__":
    
    rospy.init_node("Buffer_py")
    rospy.loginfo("Buffer has started")
    uav_total = rospy.get_param("/total_uav")
    # mean = rospy.get_param("/delay_mean")
    # std = rospy.get_param("/delay_std")
    mean = 0.0001
    std = 0.00000001
    

    pub_fuse = []
    sub_throttle = []
    sub_position = []

    d = UAVDistanceMatrix(num_uavs= uav_total, delay_constant= mean)
    for i in range(1, uav_total + 1):
        pub_fuse.append(rospy.Publisher('/uav' + str(i) + '/target_position_geolocation', target_position_fuse, queue_size=10))
        sub_throttle.append(Throttle(i))
        sub_position.append(Positions(uav_id= i, UAVDistance= d))


    delay = [_ for _ in range(uav_total)]
    timestamp = [_ for _ in range(uav_total)]
    time_now = [_ for _ in range(uav_total)]


    while not rospy.is_shutdown():
        for i in range(uav_total):
            if len(sub_throttle[i].buffer) > 0:
                for j in range(uav_total):
                    mean_delay = d.get_delay_matrix()
                    delay = np.random.normal(mean_delay[i,j],std) 
                    timestamp = sub_throttle[i].buffer[0].timestamp.to_nsec() * 1e-9
                    time_now = rospy.Time.now().to_nsec() * 1e-9
            
                    if time_now - timestamp >= delay and delay > 0:
                        #pub_delay.publish(delay)
                        rospy.loginfo("\n\n\nBuffer delay %f  ------ timestamp  %f", delay , timestamp)
                        pub_fuse[j].publish(sub_throttle[i].buffer[0])
                sub_throttle[i].buffer.pop(0)
        
        rospy.sleep(0)

