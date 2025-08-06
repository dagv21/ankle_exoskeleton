#!/usr/bin/env python3

import rospy, rosgraph
from std_msgs.msg import Float32MultiArray, Int32, Int8, Float64, Float32
# from ankle_exoskeleton.srv import DynamixelCmdSimplified, DynamixelCmdSimplifiedRequest
import time
import numpy as np

class TendonForceController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('tendon_force_controller', anonymous=True)

        self.desired_tendon_force = rospy.get_param('~desired_tendon_force', 5)  # Default value is 3N for walking tests
        self.gain_factor = self.desired_tendon_force/5.0


        # RPM limits
        self.max_rpm = 52.0
        self.min_rpm = -52.0
        # Conversion factor from RPM to control value
        self.rpm_to_value = 1 / 0.229
        # Distances in meters
        self.frontal_distance = 0.16  # 16 cm
        self.posterior_distance = 0.15  # 15 cm

        # Current tendon forces variables
        self.frontal_force = None
        self.posterior_force = None

        # Previous errors and times for PD control
        self.prev_frontal_error = 0.0
        self.prev_posterior_error = 0.0
        self.prev_frontal_error_deriv = 0.0
        self.prev_posterior_error_deriv = 0.0
        # self.prev_time = time.time()

        # Alpha controls the weight (0 = full force control, 1 = full angle correction)
        self.alpha = 0

        #Switch Initialization
        self.sw_cmd = 0 #Angle Error is zero

        # ROS rate
        self.rate = rospy.Rate(100)  #100 Hz

        # Set shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

         # Subscriber for tendons force and switching commands
        rospy.Subscriber('ankle_exo/tendons_force', Float32MultiArray, self.tendons_force_callback)
        rospy.Subscriber('ankle_joint/theta_error', Float64, self.theta_error_callback)
        rospy.Subscriber('switching_command', Int8, self.switching_command_callback)
        rospy.Subscriber('/desired_tendon_force', Float32, self.desired_tendon_force_callback)

        # Publishers for motor goal velocities
        self.frontal_velocity_pub = rospy.Publisher('/ankle_exo/frontal/dynamixel_motor/goal_velocity_forceLoop', Int32, queue_size=2)
        self.posterior_velocity_pub = rospy.Publisher('/ankle_exo/posterior/dynamixel_motor/goal_velocity_forceLoop', Int32, queue_size=2)
        self.kpF_pub = rospy.Publisher('/kp_front_force', Float64, queue_size=2)
        self.kdF_pub = rospy.Publisher('/kd_front_force', Float64, queue_size=2)
        self.kiF_pub = rospy.Publisher('/ki_front_force', Float64, queue_size=2)
        self.kpP_pub = rospy.Publisher('/kp_post_force', Float64, queue_size=2)
        self.kdP_pub = rospy.Publisher('/kd_post_force', Float64, queue_size=2)
        self.kiP_pub = rospy.Publisher('/ki_post_force', Float64, queue_size=2)
        self.errorF_pub = rospy.Publisher('/error_front_force', Float64, queue_size=2)
        self.errorP_pub = rospy.Publisher('/error_post_force', Float64, queue_size=2)
        self.derivF_pub = rospy.Publisher('/error_derivative_front_force', Float64, queue_size=2)
        self.derivP_pub = rospy.Publisher('/error_derivative_post_force', Float64, queue_size=2)
    
    def is_node_running(self, node_substring):
        try:
            master = rosgraph.Master('/rospy')
            _, _, publishers = master.getSystemState()
            all_nodes = sum(publishers, [])
            return any(node_substring in node for node in all_nodes)
        except:
            return False

    def desired_tendon_force_callback(self, msg):
        self.desired_tendon_force = msg.data
        self.gain_factor = self.desired_tendon_force/5.0
        rospy.loginfo("Desired Tendon Force: " + str(self.desired_tendon_force) + "N")

    def tendons_force_callback(self, msg):
        self.frontal_force = msg.data[0]
        self.posterior_force = msg.data[1]

    def theta_error_callback(self, msg):
        angle_error = msg.data
        max_angle_error = 0.35 # 20 degrees
        self.alpha = np.clip(abs(angle_error) / max_angle_error, 0, 1) 
    
    def switching_command_callback(self, msg):
        self.sw_cmd = msg.data

    def calculate_velocity(self, current_force, prev_error, prev_deriv, desired_force, id_motor):
        # Get the current time and compute the time difference
        # current_time = time.time()
        # dt = current_time - self.prev_time
        dt = 1/84.0

        # Error and threshold 
        error = desired_force - current_force
        threshold = 1.5 #1.5 #Newtons

        # Control Gains (Relationship between lever arms)
        if id_motor == 1: #Frontal Motor
            Kp_rolling = 4.0 
            Kp_unrolling = Kp_rolling/3
            Kd_rolling = 0.25 
            Kd_unrolling = Kd_rolling/2
            Ki = 0
        else:             #Posterior Motor
            Kp_rolling = 2.5
            Kp_unrolling = Kp_rolling/2
            Kd_rolling = 0.25
            Kd_unrolling = Kd_rolling/3
            Ki = 0

        # PD controller 
        if abs(error) > threshold:
            derivative = (error - prev_error) / dt
            derivative = derivative if abs(derivative-prev_deriv) < 50 else (derivative/10.0)
            integral = (error - prev_error) * dt if dt > 0 else 0.0
            if error > threshold: #Rolling Tendon
                velocity_p = Kp_rolling / self.gain_factor * error 
                velocity_d = Kd_rolling / self.gain_factor * derivative
                velocity_i = Ki / self.gain_factor * integral
            else: #Unrolling Tendon
                velocity_p = Kp_unrolling / self.gain_factor * error
                velocity_d = Kd_unrolling / self.gain_factor * derivative
                velocity_i = Ki / self.gain_factor * integral
        else:
            error = 0
            derivative = 0
            velocity_p = 0
            velocity_d = 0
            velocity_i = 0
            
        velocity = velocity_p + velocity_d + velocity_i

        if id_motor == 1: #Frontal Motor
            self.kpF_pub.publish(velocity_p)
            self.kdF_pub.publish(velocity_d)
            self.kiF_pub.publish(velocity_i)
            self.errorF_pub.publish(error)
            self.derivF_pub.publish(derivative)
        else:             #Posterior Motor
            self.kpP_pub.publish(velocity_p)
            self.kdP_pub.publish(velocity_d)
            self.kiP_pub.publish(velocity_i)
            self.errorP_pub.publish(error)
            self.derivP_pub.publish(derivative)
      
        # Convert RPM to control value
        # self.prev_time = current_time
        return round(velocity * self.rpm_to_value), error, derivative

    def run(self):
        rospy.loginfo("Tendon Force Controller Started")
        rospy.loginfo("Desired Tendon Force: " + str(self.desired_tendon_force) + "N")
        
        while not rospy.is_shutdown():
            if ((self.frontal_force != None) and (self.posterior_force != None)):
                # Tendon Force Calculator
                self.desired_frontal_force = self.desired_tendon_force  # Newtons larger distance
                self.desired_posterior_force = self.desired_frontal_force*self.frontal_distance/self.posterior_distance # Newtons

                cmd = self.sw_cmd

                # Validation of the ankle angle controller
                if not self.is_node_running('/ankle_angle_controller'):
                    cmd = 0 #Angle error is not considered without controller

                #Switch: Saturation of Tendon Force from the Angle Error (3 cases)
                if cmd == 1:    #Angle error is positive 
                    if self.frontal_force > self.desired_frontal_force:
                        # self.frontal_force = self.desired_frontal_force
                        # Blend the actual force and the limited force
                        self.frontal_force = (1 - self.alpha) * self.frontal_force + self.alpha * self.desired_frontal_force
                       
                elif cmd == 2:  #Angle error is negative
                    if self.posterior_force > self.desired_posterior_force:
                        # self.posterior_force = self.desired_posterior_force  
                        self.posterior_force = (1 - self.alpha) * self.posterior_force + self.alpha * self.desired_posterior_force
                else:           #Angle error is zero
                    pass
                    # print(str(self.frontal_force) + "," + str(self.posterior_force))
                
                # print(cmd)

                frontal_velocity, self.prev_frontal_error, self.prev_frontal_error_deriv = self.calculate_velocity(self.frontal_force, self.prev_frontal_error, self.prev_frontal_error_deriv, self.desired_frontal_force,1)
                posterior_velocity, self.prev_posterior_error, self.prev_posterior_error_deriv = self.calculate_velocity(self.posterior_force, self.prev_posterior_error, self.prev_posterior_error_deriv, self.desired_posterior_force,2)

                self.frontal_velocity_pub.publish(frontal_velocity)
                self.posterior_velocity_pub.publish(posterior_velocity)

                self.frontal_force = None
                self.posterior_force = None
                self.sw_cmd = 0

            self.rate.sleep()
           
        rospy.loginfo("Tendon Force Controller Finished")

    def shutdown_hook(self):
        # Stop the motors by setting the velocity to 0
        self.frontal_velocity_pub.publish(0)
        self.posterior_velocity_pub.publish(0)

if __name__ == '__main__':
    try:
        controller = TendonForceController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
