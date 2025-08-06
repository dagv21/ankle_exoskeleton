#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int16
from scipy.interpolate import interp1d
import numpy as np
import math

class AnkleAnglePublisher:
    def __init__(self, gait_data):
        # Initialize the ROS node
        rospy.init_node('ankle_angle_publisher', anonymous=True)

        # ROS Parameter to switch mode
        self.use_topic = rospy.get_param('~use_topic', False)  # True: use topic, False: use timer
        
        # ROS Publisher
        self.angle_pub = rospy.Publisher('/reference', Float32, queue_size=10)

        # Initialize gait percentage
        self.gait_percentage_from_topic = None
        self.gaitUpdated = False
        if self.use_topic:
            rospy.Subscriber('/gait_percentage_CNNs', Int16, self.gait_percentage_callback)

        
        # Define the gait data and interpolation function
        self.gait_data = np.array(gait_data)
        self.gait_interpolator = interp1d(self.gait_data[:, 0], self.gait_data[:, 1], kind='nearest', fill_value="extrapolate")

    def gait_percentage_callback(self, msg):
        self.gait_percentage_from_topic = msg.data
        self.gaitUpdated = True

    def calculate_angle(self, gait_percentage):
        # Interpolates the angle based on the gait percentage
        return self.gait_interpolator(gait_percentage)

    def publish_angle(self):
        # Set the desired walking speed (2 km/h)
        # walking_speed_mps = 2 / 3.6  # Convert speed to meters per second
        gait_period = 10.0  # Duration of the gait cycle in seconds

        # Loop to publish the angle periodically
        rate = rospy.Rate(100)  # Set the rate at 100 Hz

        while not rospy.is_shutdown():

            if self.use_topic:
                if self.gaitUpdated == True and self.gait_percentage_from_topic is not None:
                    gait_percentage = self.gait_percentage_from_topic

                    # Get the corresponding ankle angle from the gait data
                    angle = self.calculate_angle(gait_percentage + 15)

                    # Publish the angle
                    self.angle_pub.publish(math.radians(angle))
                    self.gaitUpdated = False
            else:
                current_time = rospy.get_time() % gait_period
                gait_percentage = (current_time / gait_period) * 100

                # Get the corresponding ankle angle from the gait data
                angle = self.calculate_angle(gait_percentage)
                print(gait_percentage)

                # Publish the angle
                self.angle_pub.publish(math.radians(angle)*0.6)
            
            
            # Sleep to maintain the loop rate
            rate.sleep()

if __name__ == '__main__':
    # Gait data (x: percentage, y: ankle angle in degrees)
    gait_data = [
        [1.0576923076923077, 3.8127090301003346],
        [1.6346153846153848, 2.809364548494983],
        [1.826923076923077, 1.5551839464882944],
        [2.307692307692308, 0.4013377926421405],
        [2.7884615384615388, -0.9531772575250836],
        [3.4615384615384617, -1.9063545150501673],
        [4.711538461538462, -2.408026755852843],
        [5.673076923076923, -1.8561872909698998],
        [6.4423076923076925, -1.0033444816053512],
        [7.403846153846154, 0.10033444816053512],
        [8.076923076923077, 1.3043478260869565],
        [8.846153846153847, 2.307692307692308],
        [9.903846153846155, 3.411371237458194],
        [11.346153846153847, 4.31438127090301],
        [12.692307692307693, 5.418060200668896],
        [14.615384615384617, 6.822742474916388],
        [16.923076923076923, 7.775919732441472],
        [18.75, 9.03010033444816],
        [20.576923076923077, 9.531772575250836],
        [23.461538461538463, 10.434782608695652],
        [26.923076923076923, 11.187290969899665],
        [29.32692307692308, 11.939799331103679],
        [31.346153846153847, 12.240802675585284],
        [33.46153846153846, 12.94314381270903],
        [35.38461538461539, 13.745819397993312],
        [37.11538461538462, 14.347826086956522],
        [38.84615384615385, 14.849498327759198],
        [40.38461538461539, 15.200668896321071],
        [42.5, 15.652173913043478],
        [44.23076923076923, 15.702341137123746],
        [46.057692307692314, 15.150501672240804],
        [47.5, 14.698996655518394],
        [48.84615384615385, 13.645484949832776],
        [49.61538461538462, 12.842809364548495],
        [50.48076923076923, 11.889632107023411],
        [51.34615384615385, 10.936454849498329],
        [51.82692307692308, 10.08361204013378],
        [52.40384615384615, 9.230769230769232],
        [52.78846153846154, 7.826086956521739],
        [53.46153846153847, 6.321070234113712],
        [53.94230769230769, 5.117056856187291],
        [54.23076923076923, 4.013377926421405],
        [54.61538461538462, 2.809364548494983],
        [55, 1.7558528428093645],
        [55.48076923076923, 0.45150501672240806],
        [55.67307692307693, -0.6521739130434783],
        [56.15384615384616, -1.7558528428093645],
        [56.34615384615385, -3.2608695652173916],
        [56.82692307692308, -4.464882943143813],
        [57.019230769230774, -5.4682274247491645],
        [57.40384615384616, -6.5719063545150505],
        [57.78846153846154, -7.6755852842809364],
        [57.98076923076923, -8.729096989966555],
        [58.46153846153847, -9.983277591973245],
        [58.94230769230769, -11.036789297658864],
        [59.519230769230774, -12.090301003344482],
        [60.19230769230769, -12.993311036789297],
        [61.92307692307693, -13.043478260869566],
        [62.59615384615385, -12.290969899665551],
        [63.36538461538462, -11.638795986622075],
        [64.61538461538461, -9.983277591973245],
        [66.15384615384616, -8.528428093645486],
        [67.78846153846155, -7.3244147157190636],
        [69.23076923076924, -6.0200668896321075],
        [70.1923076923077, -4.615384615384616],
        [71.25, -3.2608695652173916],
        [72.3076923076923, -1.8561872909698998],
        [73.75, -0.3010033444816054],
        [74.90384615384616, 0.8528428093645485],
        [76.82692307692308, 2.3578595317725752],
        [77.6923076923077, 3.1103678929765888],
        [79.90384615384616, 4.414715719063545],
        [81.92307692307693, 5.317725752508362],
        [84.03846153846155, 5.418060200668896],
        [86.25, 5.167224080267559],
        [88.26923076923077, 4.765886287625418],
        [90.1923076923077, 4.11371237458194],
        [92.40384615384616, 3.8628762541806023],
        [94.13461538461539, 3.8127090301003346],
        [96.25, 3.9130434782608696],
        [97.5, 4.464882943143813],
        [98.9423076923077, 4.264214046822743]
    ]

    # Create an instance of the class and start publishing angles
    ankle_angle_publisher = AnkleAnglePublisher(gait_data)
    ankle_angle_publisher.publish_angle()
