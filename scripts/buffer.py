#! /usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_fusion.msg import target_position
from MARS_msgs.msg import TargetTelemetry
from std_msgs.msg import Float64


class Throttle:
    def __init__(self, uav_id, uav_total):
        self.id = uav_id
        self.buffers = [list() for i in range(uav_total)]
        self.sub_throttle = rospy.Subscriber('/uav' + str(uav_id) + '/target_position_throttle', TargetTelemetry, self.target_callback_throttle)

    def target_callback_throttle(self, msg):
        for i, buf in enumerate(self.buffers):
            if i == self.id:
                continue
            buf.append(msg)
        

class Positions:
    def __init__(self, uav_id, UAVDistance):
        self.id = uav_id
        self.UAVDistance = UAVDistance
        self.sub_position = rospy.Subscriber(str(uav_id) + "/mavros/local_position/pose", PoseStamped, self.position_callback)

    def position_callback(self, msg):
        print(msg)
        self.UAVDistance.update_distance(uav_index= self.id, x= msg.Pose.Point.x, y= msg.Pose.Point.y)
        self.UAVDistance.update_delay_matrix()

        print(self.UAVDistance.get_distance_matrix())

    
class UAVDistanceMatrix:
    def __init__(self, num_uavs, delay_constant):
        self.num_uavs = num_uavs
        self.distances = np.zeros((num_uavs, num_uavs))
        self.delays = np.zeros((num_uavs, num_uavs))
        self.delay_constant = delay_constant

    def update_distance(self, uav_index, x, y):
        uav_index = uav_index
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
    mean = rospy.get_param("/delay_mean")
    std = rospy.get_param("/delay_std")
    

    pub_fuse = []
    pub_delay = []
    sub_throttle = []
    sub_position = []

    d = UAVDistanceMatrix(num_uavs= uav_total, delay_constant= mean)
    for i in range(uav_total):
        pub_fuse.append(rospy.Publisher('/uav' + str(i) + '/target_position_geolocation', TargetTelemetry, queue_size=10))
        sub_throttle.append(Throttle(i, uav_total))
        sub_position.append(Positions(uav_id= i, UAVDistance= d))
        pub_delay.append(rospy.Publisher('/uav' + str(i) + '/delay', Float64, queue_size=10))


    delay = [_ for _ in range(uav_total)]
    timestamp = [_ for _ in range(uav_total)]
    time_now = [_ for _ in range(uav_total)]


    while not rospy.is_shutdown():
        # for each source UAV
        for i in range(uav_total):
            # for each destination UAV
            # check the outbound buffer for all other UAVs
            for j in range(uav_total):
                if i == j: # don't add delay to self
                    continue

                # check outbound buffer from uav i to uav j
                if len(sub_throttle[i].buffers[j]) > 0:
                    # is it time to send the first message?
                    mean_delay = d.get_delay_matrix()
                    delay = -1
                    while delay < 0:
                        delay = np.random.normal(mean_delay[i,j],std) 
                        print(sub_throttle[i].buffers[0])
                        timestamp = sub_throttle[i].buffers[0].timestamp.to_nsec() * 1e-9
                        time_now = rospy.Time.now().to_nsec() * 1e-9
            
                    if time_now - timestamp >= delay and delay > 0:
                        out_msg = sub_throttle[i].buffers[j].pop(0)
                        pub_delay[j].publish(delay)
                        #rospy.loginfo("\n\n\nBuffer delay %f  ------ timestamp  %f", delay , timestamp)
                        pub_fuse[j].publish(out_msg)
                        

        
        rospy.sleep(0)

