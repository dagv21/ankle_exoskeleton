#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float32,Int16
from scipy.signal import butter, lfilter
import numpy as np

class PowerIntegrator:
    def __init__(self):
        rospy.init_node('power_integrator')

        # Parameters
        self.f_cut = 5
        self.fs = 80
        self.T_min = 10
        self.T_max = 35
        self.P_min = 0
        self.P_max = 3

        # State
        self.buffer = []
        self.phase_buffer = []
        self.prev_phase = None
        self.last_auc = None

        # Butterworth filter
        self.b, self.a = butter(N=2, Wn=self.f_cut / (0.5 * self.fs), btype='low')
        self.power_filtered = 0.0

        # Subscribers
        rospy.Subscriber('/ankle_joint/power', Float64, self.power_cb)
        rospy.Subscriber('/gait_percentage_CNNs', Int16, self.phase_cb)

        # Publisher
        self.pub_force = rospy.Publisher('/desired_tendon_force', Float32, queue_size=1)

        rospy.loginfo("Propulsion High Level controller Node Initialized.")
        rospy.spin()

    def power_cb(self, msg):
        self.raw_power = msg.data

    def phase_cb(self, msg):
        current_phase = msg.data
        if not hasattr(self, 'raw_power'):
            return

        # Filter the current power value
        if not hasattr(self, 'power_hist'):
            self.power_hist = [self.raw_power] * 4
        self.power_hist.append(self.raw_power)
        self.power_hist = self.power_hist[-4:]
        self.power_filtered = lfilter(self.b, self.a, self.power_hist)[-1]

        # Save phase and filtered power
        self.phase_buffer.append(current_phase)
        self.buffer.append(self.power_filtered)

        # Detect gait cycle reset (e.g., from 99% to 1%)
        if self.prev_phase is not None and current_phase < self.prev_phase:
            self.process_last_cycle()
            self.buffer = []
            self.phase_buffer = []

        self.prev_phase = current_phase

    def process_last_cycle(self):
        power_array = np.array(self.buffer)
        phase_array = np.array(self.phase_buffer)

        # Select data in 40–60% gait cycle range
        mask = (phase_array >= 40) & (phase_array <= 60)
        if not np.any(mask):
            return

        auc = np.trapz(abs(power_array[mask]), phase_array[mask])
        print(phase_array[mask])
        self.last_auc = auc
        desired_force = self.map_power_to_force(auc)

        # Publish the result
        self.pub_force.publish(int(desired_force))
        rospy.loginfo(f"AUC = {auc:.2f}, Desired Force = {desired_force:.2f} N")

    def map_power_to_force(self, auc):
        # Normalize power to 0–1 range
        P_norm = (auc - self.P_min) / (self.P_max - self.P_min)
        P_norm = np.clip(P_norm, 0, 1)
        # Inverse relation: more power → less tension
        tension = self.T_max - (self.T_max - self.T_min) * P_norm
        return tension

if __name__ == '__main__':
    try:
        PowerIntegrator()
    except rospy.ROSInterruptException:
        pass
