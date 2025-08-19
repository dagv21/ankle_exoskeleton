#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Float64

class AdmittanceController:
    def __init__(self):
        rospy.init_node('admittance_controller')

        # === Parameters ===
        self.mass = rospy.get_param('~mass', 0.2)       # 0.5 kg·m²
        self.damping = rospy.get_param('~damping', 8)  # 5.5 N·m·s/rad
        self.stiffness = rospy.get_param('~stiffness', 50.0)  # 30.0 N·m/rad
        # New adaptive stiffness parameters
        self.stiffness_base = rospy.get_param('~stiffness_base', 2.0)
        self.stiffness_gain = rospy.get_param('~stiffness_gain', 400.0)

        self.control_rate = 80.0  # Hz
        self.dt = 1.0 / self.control_rate

        # === Internal state ===
        self.goal_angle = 0.0         # /ankle_goal_angle
        self.net_torque = None        # /ankle_joint/net_torque
        self.theta_error = 0
        self.admittance_angle = 0.0   # x(t)
        self.admittance_velocity = 0.0  # x_dot(t)

        # === ROS Publishers & Subscribers ===
        rospy.Subscriber("/ankle_exo/net_torque", Float32, self.torque_callback)
        rospy.Subscriber('/ankle_joint/theta_error', Float64, self.angle_callback)
        self.theta_admittance_pub = rospy.Publisher("/ankle_joint/theta_admittance", Float32, queue_size=2)
        self.K_admittance_pub = rospy.Publisher("/k_admittance", Float32, queue_size=2)

        # rate = rospy.Rate(self.control_rate)
        rospy.loginfo("Admittance Controller Started")
        rospy.loginfo("Mass: " + str(self.mass) + " kg·m²")
        rospy.loginfo("Damping: " + str(self.damping) + " N·m·s/rad")

        

        rospy.spin()
    
    def torque_callback(self, msg):
        self.net_torque = msg.data
        self.run()
            
    def angle_callback(self, msg):
        self.theta_error = msg.data

    def run(self):       

        # === Adaptive stiffness ===
        adaptive_K = self.stiffness_base + self.stiffness_gain * abs(self.theta_error)
        adaptive_K = max(2.0, min(50.0, adaptive_K))

        # === Admittance model ===
        acc = (self.net_torque - self.damping * self.admittance_velocity - adaptive_K * self.admittance_angle) / self.mass
        self.admittance_velocity += acc * self.dt
        self.admittance_angle += self.admittance_velocity * self.dt

        # === Publish result ===
        
        self.theta_admittance_pub.publish(float(self.admittance_angle))
        self.K_admittance_pub.publish(adaptive_K)

if __name__ == "__main__":
    try:
        AdmittanceController()
    except rospy.ROSInterruptException:
        pass
