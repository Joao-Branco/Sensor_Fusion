#! /usr/bin/env python3

import rospy
import numpy as np
from MARS_msgs.msg import TargetTelemetry




if __name__ == "__main__":
    rospy.init_node("target_py")
    rospy.loginfo("Node Target has started")


    pub = rospy.Publisher("/target_position_true", TargetTelemetry, queue_size=10)

    msg = TargetTelemetry()
    
    
    # Target variables initialization (All changeble)
    target_accel = 0.0
    position_x = 0.0
    position_y = 0.0
    target_psi = 0 #math.pi/4
    target_velocity = 0


    # frenquency of publication of position
    f = 30

    #10 - 25 Hz (Computer Vision)

    

    rate = rospy.Rate(f)
    
    # Time related variables initialization
    start_time = rospy.get_time()
    curr_time = rospy.get_time()
    timer = 0.0
    dt = 0.0

    while not rospy.is_shutdown():
        
        
        
        # Time variation since last call on loop
        dt = rospy.get_time() - curr_time

        # Total time counter
        timer = rospy.get_time() - start_time

        curr_time = rospy.get_time()

        
        
        
        # Target movement calculations
        target_omega = 0#.02*math.cos(0.03*dt) # Changeble
        target_accel = 0#.2*math.sin(0.07*dt) # Changeble

        target_psi = target_psi + target_omega * dt
        target_velocity = target_velocity + (target_accel * dt) 
        target_vx = target_velocity * np.cos(target_psi)
        target_vy = target_velocity * np.sin(target_psi)
        position_x = position_x + target_vx * dt
        position_y = position_y + target_vy * dt

        # Defines message to be sent with target movement variables
        msg.x_pos = position_x
        msg.y_pos = position_y
        msg.vx = target_vx
        msg.vy = target_vy
        msg.omega = target_omega
        msg.accel = target_accel
        msg.vel = target_velocity
        msg.psi = target_psi
        msg.timestamp = rospy.Time.now()


    
        
        rospy.loginfo("X: %f, Y: %f, Time: %i", position_x, position_y, curr_time) 
        
        pub.publish(msg)

        
        rospy.loginfo("Target has been publish")
        


        rate.sleep()
    
