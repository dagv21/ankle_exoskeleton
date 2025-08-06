#!/usr/bin/env python3

import rospy, rosgraph
from std_msgs.msg import Int32
from ankle_exoskeleton.srv import DynamixelCmdSimplified, DynamixelCmdSimplifiedRequest
from ankle_exoskeleton.msg import DynamixelStatusList
import numpy as np

class VelocityPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('velocity_publisher', anonymous=True)

        # Variables to store values from both loops
        self.frontal_force_loop = 0
        self.frontal_angle_loop = 0
        self.frontal_position = None
        self.initial_frontal_position = None

        self.posterior_force_loop = 0
        self.posterior_angle_loop = 0
        self.posterior_position = None
        self.initial_posterior_position = None

        # Maximum and minimum velocity limits (Register Value)
        self.max_velocity = 235
        self.min_velocity = -235

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

        # Publishers for frontal and posterior goal velocities
        self.frontal_velocity_pub = rospy.Publisher('/ankle_exo/frontal/dynamixel_motor/goal_velocity', Int32, queue_size=2)
        self.posterior_velocity_pub = rospy.Publisher('/ankle_exo/posterior/dynamixel_motor/goal_velocity', Int32, queue_size=2)
    
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

    def is_node_running(self, node_substring):
        try:
            master = rosgraph.Master('/rospy')
            _, _, publishers = master.getSystemState()
            all_nodes = sum(publishers, [])
            return any(node_substring in node for node in all_nodes)
        except:
            return False

    def run(self):
        last_setpoint_change_time = rospy.get_time()
        prev_frontal_angle_loop = self.frontal_angle_loop
        prev_posterior_angle_loop = self.posterior_angle_loop

        while not rospy.is_shutdown():
            current_time = rospy.get_time()

            # Check safety conditions for frontal motor
            if (self.frontal_position is not None and
                self.initial_frontal_position is not None and
                (self.frontal_position > self.initial_frontal_position + 8000 or 
                self.frontal_position < self.initial_frontal_position - 8000)):
                
                rospy.logwarn("Frontal motor exceeded safety limits. Shutting down.")
                rospy.signal_shutdown("Frontal motor position exceeded limits.")

            # Check safety conditions for posterior motor
            if (self.posterior_position is not None and
                self.initial_posterior_position is not None and
                (self.posterior_position > self.initial_posterior_position + 8000 or 
                self.posterior_position < self.initial_posterior_position - 8000)):

                rospy.logwarn("Posterior motor exceeded safety limits. Shutting down.")
                rospy.signal_shutdown("Posterior motor position exceeded limits.")

            # Validation of the ankle angle controller
            if not self.is_node_running('/ankle_angle_controller'):
                self.frontal_angle_loop = 0
                self.posterior_angle_loop = 0
            # else:
            #     # Check if setpoint changed
            #     if (self.frontal_angle_loop != prev_frontal_angle_loop or 
            #         self.posterior_angle_loop != prev_posterior_angle_loop):
            #         last_setpoint_change_time = current_time
            #         prev_frontal_angle_loop = self.frontal_angle_loop
            #         prev_posterior_angle_loop = self.posterior_angle_loop
            #     elif current_time - last_setpoint_change_time > 1:
            #         # No change in setpoints for more than 0.5 seconds
            #         self.frontal_angle_loop = 0
            #         self.posterior_angle_loop = 0
            #         rospy.loginfo("Set point not changing for 1 seconds. Setting angle loops to 0.")

            
            if not self.is_node_running('/tendon_force_controller'):
                self.frontal_force_loop = 0
                self.posterior_force_loop = 0

            # Calculate the sum of force and angle loops for both frontal and posterior motors
            # Linear Saturation to not exceed motor limits 
            frontal_velocity = int(self.max_velocity*np.tanh(2.65*(self.frontal_force_loop + self.frontal_angle_loop)/self.max_velocity))
            posterior_velocity = int(self.max_velocity*np.tanh(2.65*(self.posterior_force_loop + self.posterior_angle_loop)/self.max_velocity))

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
