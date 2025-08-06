#!/usr/bin/env python3
import rospy
from moticon_insole.msg import InsoleData 
from std_msgs.msg import Float64, Float32
import numpy as np
from scipy.signal import butter, filtfilt, lfilter
from collections import deque

class AnkleDynamicsEstimator:
    def __init__(self):
        # ROS Node
        rospy.init_node('ankle_dynamics_estimator', anonymous=True)

        # Parameters
        self.insole_length = rospy.get_param('~insole_length', 0.2611)
        self.d_heel2ankle = rospy.get_param('~d_heel2ankle', 0.055)
        self.body_weight = rospy.get_param('~body_weight', 71)
        
        # Sampling frequency
        self.fs = 100
        
        # Butterworth filters for angle, velocity, force, and CoP
        fc = 5  # cutoff frequency for angle, force, and CoP
        self.b, self.a = butter(3, fc / (self.fs / 2), btype='low')     
        fc_vel = 10  # cutoff frequency for velocity
        self.b_vel, self.a_vel = butter(3, fc_vel / (self.fs / 2), btype='low')

        # Buffers
        self.window_size = 20 
        self.angles, self.forces, self.cop_x, self.robot_torque = [], [], [], []
        self.ankle_angle = 0

        # Variables to track state
        self.forceUpdated = False
        self.angleUpdated = False
        self.robotTorqueUpdated = False

        self.force_threshold = 0.04

        ''' Subscriber and Publisher '''
        self.insole_sub = rospy.Subscriber('/insole_data', InsoleData, self.ground_force_callback)
        self.robot_torque_sub = rospy.Subscriber('/ankle_exo/net_torque', Float32, self.robot_torque_callback)
        self.ankle_angle_sub = rospy.Subscriber('/ankle_joint/angle', Float64, self.ankle_angle_callback)
        self.ankle_power_pub = rospy.Publisher('/ankle_joint/power', Float64, queue_size=2)
        self.ankle_moment_pub = rospy.Publisher('/ankle_joint/net_moment', Float64, queue_size=2)
        self.ankle_h_moment_pub = rospy.Publisher('/ankle_joint/moment', Float64, queue_size=2)
    
    def ankle_angle_callback(self, msg):
        """Updates Angle Buffer (IMU)"""
        self.angles.append(msg.data)

        if len(self.angles) > self.window_size:
            angle_rad = np.deg2rad(self.angles)
            angle_filtered = filtfilt(self.b, self.a, angle_rad)

            vel_rps = np.diff(angle_filtered)* self.fs
            # vel_filtered = lfilter(self.b_vel, self.a_vel, vel_rps) 

            self.ankle_angle = angle_filtered[-1]
            self.ankle_angle_derivative = vel_rps[-1]
            self.angleUpdated = True

            self.angles.pop(0)
        self.try_process()

    def ground_force_callback(self, msg):
        """Updates Force Buffer (Insole)"""
        self.forces.append(msg.normalised_force if msg.normalised_force >= self.force_threshold else 0.0)
        self.cop_x.append(msg.cop_x)

        if len(self.forces) > self.window_size:
            force_filtered = filtfilt(self.b, self.a, self.forces)
            force_filtered = np.array([f if f >= self.force_threshold else 0.0 for f in force_filtered])

            self.ground_force = force_filtered[-1]
            self.cop = self.cop_x[-1]
            self.forceUpdated = True

            self.forces.pop(0)
        self.try_process()
    
    def robot_torque_callback(self, msg):
        """Updates Angle Buffer (IMU)"""
        self.robot_torque.append(msg.data)

        if len(self.robot_torque) > self.window_size:
            robot_torque_filtered = filtfilt(self.b, self.a, self.robot_torque)

            self.robot_moment = robot_torque_filtered[-1]
            self.robotTorqueUpdated = True

            self.robot_torque.pop(0)
        self.try_process()
        

    def try_process(self):
        if self.forceUpdated == True and self.angleUpdated == True and self.robotTorqueUpdated == True:
        
            cop_adjusted = self.insole_length * (self.cop + 0.5) - self.d_heel2ankle
            moment_net = (cop_adjusted * self.ground_force)*10
            # moment_filtered = filtfilt(self.b, self.a, moment_net) 
            moment = (moment_net*self.body_weight - self.robot_moment)/self.body_weight
            power = -moment * self.ankle_angle_derivative
     
            self.ankle_moment_pub.publish(float(moment_net))
            self.ankle_power_pub.publish(float(power))    
            self.ankle_h_moment_pub.publish(float(moment))

            self.angleUpdated = False
            self.forceUpdated = False

    def run(self):
        rospy.loginfo("Estimating Ankle Power and Moment")
        rospy.loginfo("Ankle Dynamics Estimator initialized with:")
        rospy.loginfo(" - Insole Length: %.4f m", self.insole_length)
        rospy.loginfo(" - Distance Heel to Ankle: %.4f m", self.d_heel2ankle)
        rospy.spin()

if __name__ == '__main__':
    try:
        estimator = AnkleDynamicsEstimator()
        estimator.run()
    except rospy.ROSInterruptException:
        pass
