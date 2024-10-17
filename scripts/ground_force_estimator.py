#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray, Float64, Int8
from scipy.signal import butter, filtfilt
import numpy as np
from collections import deque

class vGRFEstimator:
    def __init__(self):
        #ROS variables
        rospy.init_node('ground_reaction_estimator', anonymous=True)

        self.foot_imu_sub = rospy.Subscriber('/insole_data', Int32MultiArray, self.insole_data_callback)

        self.vGRF_pub = rospy.Publisher('/vGRF', Float64, queue_size=2)
        self.insole_data_filtered_pub = rospy.Publisher('/insole_data_filtered', Int32MultiArray, queue_size=2)
        self.cop_pub = rospy.Publisher('/CoP', Int8, queue_size=2)
        
        # Butterworth filter parameters
        self.cutoff_freq = 2  # Cutoff frequency in Hz
        self.Fs = 100  # Sample rate in Hz
        self.order = 3
        self.b, self.a = self.butter_lowpass(self.cutoff_freq, self.Fs, self.order)
        self.buffer_size = 13  # Should be greater than the filter order
        self.data_buffer = deque(maxlen=self.buffer_size)  # Stores the last N samples

        # Median filter parameters
        self.vGRF_buffer = deque(maxlen=5) # Filter window

        # vGRF normalization parameters 
        # Method: Running Peak Tracking | fun -> normalize_vGRF
        self.running_peak = 0  # Store the running peak
        self.decay_factor = 0.9999  # Decay factor to slowly reduce the peak over time
        # Method Last Cycle Peak | fun -> normalize_vGRF2
        self.noise_threshold = 200
        self.last_peak = 1.0  # Store the last peak for normalization, initialized to 1 to avoid division by zero
        self.vGRF_cycle = []  # Buffer to store vGRF values during the current cycle
        self.in_stance = False

        
    def butter_lowpass(self, cutoff, fs, order=2):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def apply_filter(self, data):
        # Apply the filter to the entire data
        filtered_data = filtfilt(self.b, self.a, data)
        return filtered_data

    def insole_data_callback(self, msg):
        current_data = np.array(msg.data)
        current_data[current_data<50] = 0 # Remove noise by treadmill
        self.data_buffer.append(current_data) 
        
        if len(self.data_buffer) == self.buffer_size: 
            buffer_array = np.array(self.data_buffer)
            # Apply the filter to each position (column) in the buffer
            filtered_data = np.apply_along_axis(self.apply_filter, 0, buffer_array)  # Apply along columns
            self.process_data(np.round(filtered_data[-1]).astype(int))

            int_filtered_data = np.round(filtered_data[-1]).astype(int) 
            # Prepare the Int32MultiArray for publishing
            int_array_msg = Int32MultiArray(data=int_filtered_data.astype(np.int32).tolist())
            self.insole_data_filtered_pub.publish(int_array_msg)

    def median_filter(self, vGRF):
        # Add the new vGRF to the buffer
        self.vGRF_buffer.append(vGRF)
        # Apply median filter
        return np.median(self.vGRF_buffer)

    def normalize_vGRF2(self, vGRF_filtered):
        # Update the cycle and detect peak during the stance phase
        if vGRF_filtered > self.noise_threshold:  # Stance phase
            self.vGRF_cycle.append(vGRF_filtered)
            self.in_stance = True
        else:  # Swing phase
            if self.in_stance and len(self.vGRF_cycle) > 0:  # End of stance phase, detect peak
                self.last_peak = max(self.vGRF_cycle)  # Get the peak of the stance phase
                self.vGRF_cycle.clear()  # Clear the cycle buffer for the next cycle
            self.in_stance = False

        # Normalize using the last peak (avoid division by zero)
        if self.last_peak > 0:
            return vGRF_filtered / self.last_peak
        else:
            return 0.0
    
    def normalize_vGRF(self, vGRF_filtered):
        # Update the running peak
        self.running_peak = max(self.running_peak * self.decay_factor, vGRF_filtered)

        # Normalize using the running peak (avoid division by zero)
        if self.running_peak > 0:
            return vGRF_filtered / self.running_peak
        else:
            return 0.0

    def process_data(self, data):
        
        # Group insole data in regions: heel, mid and tip
        heel1_data = data[13] + data[15]
        heel2_data = data[11] + data[12] + data[14]  
        mid1_data = data[5] + data[9] + data[10] 
        mid2_data = data[6] + data[7] + data[8] 
        tip1_data = data[2] + data[3] + data[4]
        tip2_data = data[0] + data[1]
       
        
        # Estimate the ground reaction force as the maximum value
        regions = [heel1_data, heel2_data, mid1_data, mid2_data, tip1_data, tip2_data]

        # Find the maximum value and the corresponding region index (0 to 5)
        max_region_index = np.argmax(regions)
        max_value = regions[max_region_index]

        vGRF = max_value
        if vGRF < self.noise_threshold:
            vGRF = 0
 
        # Filter ground reaction force with median value
        vGRF_filtered = int(self.median_filter(vGRF))
        
        #Normalize the filtered vGRF using the peak of the last cycle
        vGRF_normalized = self.normalize_vGRF(vGRF_filtered)
        
        try:
            self.vGRF_pub.publish(float(vGRF_normalized))
            self.cop_pub.publish(max_region_index)
        except IOError:
            rospy.loginfo("Program Finished")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        estimator = vGRFEstimator()
        estimator.run()
    except rospy.ROSInterruptException:
        pass
