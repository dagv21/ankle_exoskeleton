#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int8, Int32
import time
from scipy.signal import butter, filtfilt
import numpy as np

class AnkleAngleController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ankle_angle_controller', anonymous=True)

        # Gains for frontal and posterior motors
        self.kp_frontal = rospy.get_param('~kp_frontal', 450.0)  
        self.kd_frontal = rospy.get_param('~kd_frontal', 50)  
        self.ki_frontal = rospy.get_param('~ki_frontal', 0.0) 
        self.kp_posterior = rospy.get_param('~kp_posterior', 450.0)  
        self.kd_posterior = rospy.get_param('~kd_posterior', 50)  
        self.ki_posterior = rospy.get_param('~ki_posterior', 0.0) 

        self.fs = 100.0
         # Butterworth filters for angle, velocity, force, and CoP
        # fc = 5  # cutoff frequency for angle, force, and CoP
        # self.b, self.a = butter(3, fc / (self.fs / 2), btype='low')     
        # fc_vel = 10  # cutoff frequency for velocity
        # self.b_vel, self.a_vel = butter(3, fc_vel / (self.fs / 2), btype='low')

        self.window_size = 12

        self.angles = []
        # self.ankle_angle = None
        # self.ankle_velocity = None
        self.angleUpdated = False

        # Previous errors and time for PD control
        self.prev_theta_error = 0.0
        self.prev_frontal_error_deriv = 0.0
        self.prev_posterior_error_deriv = 0.0
        self.prev_time = time.time()

        # Switching command state
        self.switching_command = 0  # 0 means no control

        # RPM limits
        # self.max_rpm = 52.0
        # self.min_rpm = -52.0

        # Conversion factor from RPM to control value
        self.rpm_to_value = 1 / 0.229

        # ROS rate
        self.rate = rospy.Rate(self.fs)  

         # Set shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

        # Subscribers
        rospy.Subscriber('/ankle_joint/theta_error', Float64, self.theta_error_callback)
        # rospy.Subscriber('/ankle_joint/angle_rad', Float64, self.angle_callback)
        rospy.Subscriber('/switching_command', Int8, self.switching_command_callback)

        # Publishers for motor goal velocities
        self.frontal_velocity_pub = rospy.Publisher('/ankle_exo/frontal/dynamixel_motor/goal_velocity_angleLoop', Int32, queue_size=2)
        self.posterior_velocity_pub = rospy.Publisher('/ankle_exo/posterior/dynamixel_motor/goal_velocity_angleLoop', Int32, queue_size=2)
        self.kpF_pub = rospy.Publisher('/kp_front_angle', Float64, queue_size=2)
        self.kdF_pub = rospy.Publisher('/kd_front_angle', Float64, queue_size=2)
        self.kiF_pub = rospy.Publisher('/ki_front_angle', Float64, queue_size=2)
        self.kpP_pub = rospy.Publisher('/kp_post_angle', Float64, queue_size=2)
        self.kdP_pub = rospy.Publisher('/kd_post_angle', Float64, queue_size=2)
        self.kiP_pub = rospy.Publisher('/ki_post_angle', Float64, queue_size=2)
        self.errorF_pub = rospy.Publisher('/error_front_angle', Float64, queue_size=2)
        self.errorP_pub = rospy.Publisher('/error_post_angle', Float64, queue_size=2)
        self.derivF_pub = rospy.Publisher('/error_derivative_front_angle', Float64, queue_size=2)
        self.derivP_pub = rospy.Publisher('/error_derivative_post_angle', Float64, queue_size=2)
    
    # def angle_callback(self, msg):
    #     self.angles.append(msg.data)

    #     if len(self.angles) > self.window_size:
    #         angle_rad = np.deg2rad(self.angles)
    #         angle_filtered = filtfilt(self.b, self.a, angle_rad)

    #         vel_rps = np.diff(angle_filtered) #* self.fs
    #         # vel_filtered = lfilter(self.b_vel, self.a_vel, vel_rps) 

    #         self.ankle_angle = angle_filtered[-1]
    #         self.ankle_velocity = vel_rps[-1]
    #         self.angleUpdated = True

    #         self.angles.pop(0)

    def theta_error_callback(self, msg):
        self.theta_error = msg.data

    def switching_command_callback(self, msg):
        self.switching_command = msg.data

    def calculate_velocity(self, error, prev_error, prev_deriv,kp, kd, ki, id_motor):
        # current_time = time.time()
        # dt = current_time - self.prev_time 
        dt = 1/self.fs

        # Proportional and Derivative terms
        derivative = (error - prev_error) / dt
        # derivative = derivative if abs(derivative-prev_deriv) < 0.5 else (derivative/0.5)
        integral = (error - prev_error) * dt if dt > 0 else 0.0

        velocity_p = kp * error
        velocity_d = kd * derivative
        velocity_i = ki * integral


        velocity = velocity_p + velocity_d + velocity_i

        if id_motor == 1:
            self.kpF_pub.publish(velocity_p)
            self.kdF_pub.publish(velocity_d)
            self.kiF_pub.publish(velocity_i)
            self.errorF_pub.publish(error)
            self.derivF_pub.publish(derivative)
            self.derivF_pub.publish(derivative)
        else:
            self.kpP_pub.publish(velocity_p)
            self.kdP_pub.publish(velocity_d)
            self.kiP_pub.publish(velocity_i)
            self.errorP_pub.publish(error)
            self.derivP_pub.publish(derivative)

        # Convert RPM to control value
        # self.prev_time = current_time
        return round(velocity * self.rpm_to_value), error, derivative
    
    def is_node_running(self, node_substring):
        try:
            master = rosgraph.Master('/rospy')
            _, _, publishers = master.getSystemState()
            all_nodes = sum(publishers, [])
            return any(node_substring in node for node in all_nodes)
        except:
            return False

    def shutdown_hook(self):
        # Stop the motors by setting the velocity to 0
        self.frontal_velocity_pub.publish(0)
        self.posterior_velocity_pub.publish(0)

    def run(self):
        rospy.loginfo("Ankle Angle Controller Started")

        while not rospy.is_shutdown():
            if not self.is_node_running('/angle_setpoint_publisher'):
                if hasattr(self, 'theta_error'):
                    # Get the absolute value of the error
                    abs_error = abs(self.theta_error)

                    # Control logic based on switching command
                    if self.switching_command == 0:  # Error is zero, no control applied
                        frontal_velocity = 0
                        posterior_velocity = 0
                    elif self.switching_command == 1:  # Positive error, control only frontal motor
                        frontal_velocity, self.prev_frontal_error, self.prev_frontal_error_deriv = self.calculate_velocity(abs_error, self.prev_theta_error,self.prev_frontal_error_deriv, self.kp_frontal, self.kd_frontal, self.ki_frontal,1)
                        posterior_velocity = 0
                    elif self.switching_command == 2:  # Negative error, control only posterior motor
                        frontal_velocity = 0
                        posterior_velocity, self.prev_posterior_error, self.prev_posterior_error_deriv  = self.calculate_velocity(abs_error, self.prev_theta_error,self.prev_posterior_error_deriv, self.kp_posterior, self.kd_posterior, self.ki_posterior,2)

                    # Publish velocities
                    self.frontal_velocity_pub.publish(frontal_velocity)
                    self.posterior_velocity_pub.publish(posterior_velocity)

                    # Update previous values
                    self.prev_theta_error = abs_error
            else:
                pass
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = AnkleAngleController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
