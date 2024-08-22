#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int32
from ankle_exoskeleton.srv import DynamixelCmdSimplified, DynamixelCmdSimplifiedRequest
import time

class AnkleExoskeletonMotorController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ankle_exoskeleton_motor_controller', anonymous=True)

        # Publishers for motor goal velocities
        self.frontal_velocity_pub = rospy.Publisher('/ankle_exo_frontal/dynamixel_motor/goal_velocity', Int32, queue_size=10)
        self.posterior_velocity_pub = rospy.Publisher('/ankle_exo_posterior/dynamixel_motor/goal_velocity', Int32, queue_size=10)

        # Subscribers for tendon forces
        rospy.Subscriber('ankle_exo_frontal/tendon_force', Float32, self.frontal_force_callback)
        rospy.Subscriber('ankle_exo_posterior/tendon_force', Float32, self.posterior_force_callback)
        rospy.Subscriber('ankle_joint/torque', Float32, self.joint_torque_callback)
        rospy.Subscriber('ankle_joint/desired_torque', Float32, self.joint_desired_torque_callback)

        # RPM limits
        self.max_rpm = 55.0
        self.min_rpm = -55.0

        # Conversion factor from RPM to control value
        self.rpm_to_value = 1 / 0.229

        # Distances in meters
        self.frontal_distance = 0.19  # 19 cm
        self.posterior_distance = 0.15  # 15 cm

        # Current tendon forces and ankle torque
        self.frontal_force = 0.0
        self.posterior_force = 0.0
        self.joint_torque = 0.0
        self.joint_desired_torque = 0.0

        self.desired_pretension_frontal = 15  # Newtons
        self.desired_pretension_posterior = (self.desired_pretension_frontal*self.frontal_distance + self.joint_desired_torque)/self.posterior_distance # Newtons

        # Previous errors and times for PD control
        self.prev_frontal_error = 0.0
        self.prev_posterior_error = 0.0
        self.prev_time = time.time()

        # Control gains
        self.kp = 1.0  # Proportional gain
        self.kd = 1.0  # Derivative gain

        # ROS rate
        self.rate = rospy.Rate(10)  # 10 Hz

        # Activate motors on start
        self.activate_motors()

        # Set shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

    def activate_motors(self):
        try:
            rospy.wait_for_service('/ankle_exo_frontal/dynamixel_motor/torque_enable')
            rospy.wait_for_service('/ankle_exo_posterior/dynamixel_motor/torque_enable')

            enable_torque_frontal = rospy.ServiceProxy('/ankle_exo_frontal/dynamixel_motor/torque_enable', DynamixelCmdSimplified)
            enable_torque_posterior = rospy.ServiceProxy('/ankle_exo_posterior/dynamixel_motor/torque_enable', DynamixelCmdSimplified)

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

    def frontal_force_callback(self, msg):
        self.frontal_force = msg.data

    def posterior_force_callback(self, msg):
        self.posterior_force = msg.data

    def joint_torque_callback(self, msg):
        self.joint_torque = msg.data

    def joint_desired_torque_callback(self, msg):
        self.joint_desired_torque = msg.data

    def calculate_velocity(self, current_force, prev_error, desired_pretension):
        # Get the current time and compute the time difference
        current_time = time.time()
        dt = current_time - self.prev_time

        # PD control
        error = round(desired_pretension) - round(current_force)
        threshold = 4

        if abs(error) <= threshold:
            error = 0
            velocity_p = 0
            velocity_d = 0
        else:
            derivative = (error - prev_error) / dt if dt > 0 else 0.0
            if error > threshold: #Rolling Tendon
                velocity_p = 3 * error
                velocity_d = 0
            else: #Unrolling Tendon
                velocity_p = 1 * error
                velocity_d = 0

        velocity = velocity_p + velocity_d
        # Clamp the velocity within the limits
        if velocity > self.max_rpm:
            velocity = self.max_rpm
        if velocity < self.min_rpm:
            velocity = self.min_rpm

        # Convert RPM to control value
        self.prev_time = current_time
        return round(velocity * self.rpm_to_value), error

    def run(self):
        while not rospy.is_shutdown():
            # Desired pretension level
            threshold = 2
            if self.joint_desired_torque >= threshold:
                self.desired_pretension_frontal = 15  # Newtons
                self.desired_pretension_posterior = (self.desired_pretension_frontal*self.frontal_distance + self.joint_desired_torque)/self.posterior_distance # Newtons
            elif self.joint_desired_torque <= -threshold:
                self.desired_pretension_posterior = 15  # Newtons
                self.desired_pretension_frontal = (self.desired_pretension_posterior*self.posterior_distance - self.joint_desired_torque)/self.frontal_distance # Newtons
            else:
                print("threshold")

            frontal_velocity, self.prev_frontal_error = self.calculate_velocity(self.frontal_force, self.prev_frontal_error, self.desired_pretension_frontal)
            posterior_velocity, self.prev_posterior_error = self.calculate_velocity(self.posterior_force, self.prev_posterior_error, self.desired_pretension_posterior)

            self.frontal_velocity_pub.publish(frontal_velocity)
            #self.posterior_velocity_pub.publish(posterior_velocity)

            self.rate.sleep()

    def shutdown_hook(self):
        # Stop the motors by setting the velocity to 0
        self.frontal_velocity_pub.publish(0)
        self.posterior_velocity_pub.publish(0)

        # Call services to disable torque for both motors
        try:
            rospy.wait_for_service('/ankle_exo_frontal/dynamixel_motor/torque_enable')
            rospy.wait_for_service('/ankle_exo_posterior/dynamixel_motor/torque_enable')

            disable_torque_frontal = rospy.ServiceProxy('/ankle_exo_frontal/dynamixel_motor/torque_enable', DynamixelCmdSimplified)
            disable_torque_posterior = rospy.ServiceProxy('/ankle_exo_posterior/dynamixel_motor/torque_enable', DynamixelCmdSimplified)

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
        controller = AnkleExoskeletonMotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
