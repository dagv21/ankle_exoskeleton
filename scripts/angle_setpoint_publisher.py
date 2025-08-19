#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64, Int8, Float32
import numpy as np

class AnkleSetPointPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('angle_setpoint_publisher', anonymous=True)

        # Parameters
        self.use_external_reference = rospy.get_param('~use_external_reference', False)
        self.use_admittance = rospy.get_param('~use_admittance', False)
        self.threshold = np.deg2rad(0.5)   #Radians

        # Initialize variables
        self.current_angle = None
        self.external_reference = None
        self.time_start = rospy.get_time()
        self.theta_admittance = None

        # Publishers
        self.theta_error_pub = rospy.Publisher('/ankle_joint/theta_error', Float64, queue_size=2)
        self.goal_angle_pub = rospy.Publisher('/ankle_joint/goal_angle', Float64, queue_size=2)
        self.switching_command_pub = rospy.Publisher('/switching_command', Int8, queue_size=2)

        # Subscribers
        rospy.Subscriber('/ankle_joint/angle_rad', Float64, self.angle_callback)
        if self.use_external_reference:
            rospy.Subscriber('/reference', Float32, self.external_reference_callback)
        if self.use_admittance:
            rospy.Subscriber('/ankle_joint/theta_admittance', Float32, self.admittance_angle_callback)
            self.global_goal_angle_pub = rospy.Publisher('/ankle_joint/global_goal_angle', Float64, queue_size=2)

        # ROS rate
        self.rate = rospy.Rate(30)  # 50 Hz
        # Set shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

    def angle_callback(self, msg):
        self.current_angle = msg.data

    def external_reference_callback(self, msg):
        self.external_reference = msg.data
    
    def admittance_angle_callback(self, msg):
        self.theta_admittance = msg.data

    def calculate_sinusoidal_reference(self):
        amplitude = 15 #15  # Degrees
        frequency = 0.3 #0.3   # Hz    
        current_time = rospy.get_time() - self.time_start
        # Sinusoidal reference 
        reference_angle = amplitude * math.sin(2 * math.pi * frequency * current_time) #- 10
        return np.deg2rad(reference_angle)

    def compute_error(self):

        if (self.current_angle != None):
            if self.use_external_reference:
                reference_angle = self.external_reference
                if reference_angle == None:
                    return
            else:
                reference_angle = self.calculate_sinusoidal_reference()

            if self.use_admittance:
                if self.theta_admittance != None:
                    self.global_goal_angle_pub.publish(reference_angle)
                    reference_angle = reference_angle - self.theta_admittance
                    self.theta_admittance = None
                else:
                    return

            # Calculate error
            error = reference_angle - self.current_angle

            # Publish theta error and goal angle
            self.theta_error_pub.publish(error)
            self.goal_angle_pub.publish(reference_angle)

            # Determine switching command
            if abs(error) <= self.threshold:
                command = 0  # Zero error
            elif error > self.threshold:
                command = 1  # Positive error
            else:
                command = 2  # Negative error

            # Publish switching command
            self.switching_command_pub.publish(command)
            self.current_angle = None  
        else:
            self.switching_command_pub.publish(0)
            rospy.logwarn("Missing ankle angle estimated by IMUs ")
            return

    def run(self):
        rospy.loginfo("Starting Angle Error Calculator")
        while not rospy.is_shutdown():
            self.compute_error()
            self.rate.sleep()
        rospy.loginfo("Angle Error Calculator Finished")
    
    def shutdown_hook(self):
        # send switch command to 0
        self.switching_command_pub.publish(0) 

if __name__ == '__main__':
    try:
        controller = AnkleSetPointPublisher()
        controller.run()
    except rospy.ROSInterruptException:
        pass
