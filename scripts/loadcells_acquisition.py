#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray
import serial, os
import numpy as np

class LoadcellAcquisitionNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('loadcells_acquisition_node', anonymous=True)

        # Publisher for tendon force and joint torque
        self.tendons_force = rospy.Publisher('ankle_exo/tendons_force', Float32MultiArray, queue_size=2)
        self.joint_torque_pub = rospy.Publisher('ankle_exo/net_torque', Float32, queue_size=2)

        # Serial port configuration
        self.serial_port = "/dev/ttyACM0"
        self.baud_rate = 250000
        os.system("sudo chmod 777 " + self.serial_port)
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        # Distances in meters
        self.frontal_distance = 0.16  # 16 cm
        self.posterior_distance = 0.15  # 15 cm

        # ROS rate
        self.rate = rospy.Rate(100)  # 100 Hz

        #Variables
        self.prev_frontal_force = 0
        self.prev_posterior_force = 0
        self.alpha = 0.1  # Low-pass filter smoothing factor
        self.window_size = 5  # Moving average window size
        self.frontal_force_window = []
        self.posterior_force_window = []

    def calculate_torque(self, force, distance):
        return force * distance

    def low_pass_filter(self, new_value, prev_value):
        # Simple low-pass filter: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
        return self.alpha * new_value + (1 - self.alpha) * prev_value

    def moving_average(self, data_window, new_value):
        # Maintain a fixed-size window for moving average
        data_window.append(new_value)
        if len(data_window) > self.window_size:
            data_window.pop(0)
        return np.mean(data_window)

    def run(self):
        rospy.loginfo("Publishing Tendons Force and Joint Torque Data")
        while not rospy.is_shutdown():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    forces = line.split(',')
                    if len(forces) == 2:
                        frontal_force_raw = float(forces[0])
                        posterior_force_raw = float(forces[1])

                        # Apply low-pass filter
                        frontal_force_filtered = self.low_pass_filter(frontal_force_raw, self.prev_frontal_force)
                        posterior_force_filtered = self.low_pass_filter(posterior_force_raw, self.prev_posterior_force)

                        # Apply moving average smoothing
                        frontal_force_smooth = self.moving_average(self.frontal_force_window, frontal_force_filtered)
                        posterior_force_smooth = self.moving_average(self.posterior_force_window, posterior_force_filtered)

                        # Update previous values
                        self.prev_frontal_force = frontal_force_filtered
                        self.prev_posterior_force = posterior_force_filtered

                        # Publish smoothed forces (rounded for example)
                        # frontal_force = round(frontal_force_smooth)
                        # posterior_force = round(posterior_force_smooth)
                        frontal_force = frontal_force_smooth
                        posterior_force = posterior_force_smooth

                        if frontal_force < 0:
                            frontal_force = 0
                        if posterior_force < 0:
                            posterior_force = 0

                        # Calculate torques
                        frontal_torque = self.calculate_torque(frontal_force, self.frontal_distance)
                        posterior_torque = self.calculate_torque(posterior_force, self.posterior_distance)

                        # Publish tendon forces
                        msg = Float32MultiArray()
                        msg.data = [frontal_force, posterior_force]
                        self.tendons_force.publish(msg)

                        # Publish joint torque
                        self.joint_torque_pub.publish(frontal_torque-posterior_torque)

            except serial.SerialException as e:
                rospy.logerr(f"Serial port error: {e}")
            except ValueError as e:
                rospy.logerr(f"Value error: {e}")

            self.rate.sleep()
        rospy.loginfo("Force and Torque Data Finished")

if __name__ == '__main__':
    try:
        node = LoadcellAcquisitionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
