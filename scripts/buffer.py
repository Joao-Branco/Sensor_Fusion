#! /usr/bin/env python

import rospy
import numpy as np
from sensor_fusion.msg import target_position_fuse
from std_msgs.msg import Float64

def target_callback_delay(msg):
    buffer.append(msg)
    
    

        

if __name__ == "__main__":
    
    rospy.init_node("Buffer_py")
    rospy.loginfo("Buffer has started")
    uav_id = rospy.get_param("~uav_id")
    uav_total = rospy.get_param("/total_uav")
    mean = rospy.get_param("/delay_mean")
    std = rospy.get_param("/delay_std")
    
    
    buffer = []
    
    pub_fuse = rospy.Publisher("target_position_fuse", target_position_fuse, queue_size=10)
    pub_delay = rospy.Publisher('delay', Float64, queue_size=1)
    
    rospy.Subscriber('/uav' + str(uav_id) + '/target_position_delay', target_position_fuse, target_callback_delay)
    


    while not rospy.is_shutdown():
        
        if len(buffer) > 0:
            delay = np.random.normal(mean,std) 
            timestamp = buffer[0].timestamp.to_nsec() * 1e-9
            time_now = rospy.Time.now().to_nsec() * 1e-9
            
            if time_now - timestamp >= delay:
                pub_delay.publish(delay)
                rospy.loginfo("\n\n\nBuffer delay %f  ------ timestamp  %f", delay , timestamp)
                pub_fuse.publish(buffer.pop(0))
        
        rospy.sleep(0)

