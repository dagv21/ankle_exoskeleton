#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from ankle_exoskeleton.srv import DynamixelCmdSimplified, DynamixelCmdSimplifiedRequest
from ankle_exoskeleton.msg import DynamixelStatusList
import numpy as np

class VelocityPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('velocity_publisher', anonymous=True)

        # Publishers for frontal and posterior goal velocities
        self.frontal_velocity_pub = rospy.Publisher('/ankle_exo/frontal/dynamixel_motor/goal_velocity', Int32, queue_size=2)
        self.posterior_velocity_pub = rospy.Publisher('/ankle_exo/posterior/dynamixel_motor/goal_velocity', Int32, queue_size=2)

        # Variables to store values from both loops
        self.frontal_force_loop = 0
        self.frontal_angle_loop = 0
        self.frontal_position = None
        self.initial_frontal_position = None

        self.posterior_force_loop = 0
        self.posterior_angle_loop = 0
        self.posterior_position = None
        self.initial_posterior_position = None

        # Maximum and minimum velocity limits
        self.max_velocity = 240
        self.min_velocity = -240

        # Subscribers for the force and angle loop velocities
        rospy.Subscriber('/ankle_exo/frontal/dynamixel_motor/goal_velocity_forceLoop', Int32, self.frontal_force_loop_callback)
        rospy.Subscriber('/ankle_exo/frontal/dynamixel_motor/goal_velocity_angleLoop', Int32, self.frontal_angle_loop_callback)
        rospy.Subscriber('/ankle_exo/frontal/dynamixel_motor/status', DynamixelStatusList, self.frontal_status_callback)
        
        rospy.Subscriber('/ankle_exo/posterior/dynamixel_motor/goal_velocity_forceLoop', Int32, self.posterior_force_loop_callback)
        rospy.Subscriber('/ankle_exo/posterior/dynamixel_motor/goal_velocity_angleLoop', Int32, self.posterior_angle_loop_callback)
        rospy.Subscriber('/ankle_exo/posterior/dynamixel_motor/status', DynamixelStatusList, self.posterior_status_callback)

        # ROS rate for control loop
        self.rate = rospy.Rate(100)  # 100 Hz

        # Activate motors on start
        self.activate_motors()

        # Set shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
    
    def activate_motors(self):
        try:
            rospy.wait_for_service('/ankle_exo/frontal/dynamixel_motor/torque_enable')
            rospy.wait_for_service('/ankle_exo/posterior/dynamixel_motor/torque_enable')

            enable_torque_frontal = rospy.ServiceProxy('/ankle_exo/frontal/dynamixel_motor/torque_enable', DynamixelCmdSimplified)
            enable_torque_posterior = rospy.ServiceProxy('/ankle_exo/posterior/dynamixel_motor/torque_enable', DynamixelCmdSimplified)

            # Create the request to enable torque (id: 0, value: 1)
            request = DynamixelCmdSimplifiedRequest(id=0, value=1)

            # Call the services to enable the torque
            res_frontal = enable_torque_frontal(request)
            res_posterior = enable_torque_posterior(request)

            if res_frontal.comm_result and res_posterior.comm_result:
                rospy.loginfo("Torque enabled for both motors.")
            else:
                rospy.logwarn("Failed to enable torque for one or both motors.")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def frontal_force_loop_callback(self, msg):
        self.frontal_force_loop = msg.data

    def frontal_angle_loop_callback(self, msg):
        self.frontal_angle_loop = msg.data
    
    def frontal_status_callback(self, msg):
        frontal_status = msg.dynamixel_status
        self.frontal_position = frontal_status[0].present_position
        # Store the initial frontal position if not already set
        if self.initial_frontal_position is None:
            self.initial_frontal_position = self.frontal_position

    def posterior_force_loop_callback(self, msg):
        self.posterior_force_loop = msg.data

    def posterior_angle_loop_callback(self, msg):
        self.posterior_angle_loop = msg.data
    
    def posterior_status_callback(self, msg):
        posterior_status = msg.dynamixel_status
        self.posterior_position = posterior_status[0].present_position
        # Store the initial posterior position if not already set
        if self.initial_posterior_position is None:
            self.initial_posterior_position = self.posterior_position

    def run(self):
        while not rospy.is_shutdown():

            # Check safety conditions for frontal motor
            if (self.frontal_position is not None and
                self.initial_frontal_position is not None and
                (self.frontal_position > self.initial_frontal_position + 5000 or 
                self.frontal_position < self.initial_frontal_position - 2000)):
                
                rospy.logwarn("Frontal motor exceeded safety limits. Shutting down.")
                rospy.signal_shutdown("Frontal motor position exceeded limits.")

            # Check safety conditions for posterior motor
            if (self.posterior_position is not None and
                self.initial_posterior_position is not None and
                (self.posterior_position > self.initial_posterior_position + 5000 or 
                self.posterior_position < self.initial_posterior_position - 2000)):

                rospy.logwarn("Posterior motor exceeded safety limits. Shutting down.")
                rospy.signal_shutdown("Posterior motor position exceeded limits.")

            # Calculate the sum of force and angle loops for both frontal and posterior motors
            # Linear Saturation to not exceed motor limits 
            frontal_velocity = int(self.max_velocity*np.tanh((self.frontal_force_loop + self.frontal_angle_loop)/self.max_velocity))
            posterior_velocity = int(self.max_velocity*np.tanh((self.posterior_force_loop + self.posterior_angle_loop)/self.max_velocity))

            # Publish the clamped velocities
            self.frontal_velocity_pub.publish(frontal_velocity)
            self.posterior_velocity_pub.publish(posterior_velocity)

            # Sleep to maintain the loop rate
            self.rate.sleep()

    def shutdown_hook(self):
        # Stop the motors by setting the velocity to 0
        self.frontal_velocity_pub.publish(0)
        self.posterior_velocity_pub.publish(0)

        # Call services to disable torque for both motors
        try:
            rospy.wait_for_service('/ankle_exo/frontal/dynamixel_motor/torque_enable')
            rospy.wait_for_service('/ankle_exo/posterior/dynamixel_motor/torque_enable')

            disable_torque_frontal = rospy.ServiceProxy('/ankle_exo/frontal/dynamixel_motor/torque_enable', DynamixelCmdSimplified)
            disable_torque_posterior = rospy.ServiceProxy('/ankle_exo/posterior/dynamixel_motor/torque_enable', DynamixelCmdSimplified)

            # Create the request to disable torque (id: 0, value: 0)
            request = DynamixelCmdSimplifiedRequest(id=0, value=0)

            # Call the services to disable the torque
            res_frontal = disable_torque_frontal(request)
            res_posterior = disable_torque_posterior(request)

            if res_frontal.comm_result and res_posterior.comm_result:
                rospy.loginfo("Torque disabled for both motors.")
            else:
                rospy.logwarn("Failed to disable torque for one or both motors.")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        velocity_publisher = VelocityPublisher()
        velocity_publisher.run()
    except rospy.ROSInterruptException:
        pass
