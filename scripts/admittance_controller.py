#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Float64

class AdmittanceController:
    def __init__(self):
        rospy.init_node('admittance_controller')

        # === Parameters ===
        self.mass = rospy.get_param('~mass', 0.8)       # 0.5 kg·m²
        self.damping = rospy.get_param('~damping', 2.5)  # 5.5 N·m·s/rad
        self.stiffness = rospy.get_param('~stiffness', 100.0)  # 30.0 N·m/rad
        self.control_rate = 80.0  # Hz
        self.dt = 1.0 / self.control_rate

        # === Internal state ===
        self.goal_angle = 0.0         # /ankle_goal_angle
        self.net_torque = None        # /ankle_joint/net_torque
        self.admittance_angle = 0.0   # x(t)
        self.admittance_velocity = 0.0  # x_dot(t)

        # === ROS Publishers & Subscribers ===
        rospy.Subscriber("/ankle_exo/net_torque", Float32, self.torque_callback)
        self.theta_admittance_pub = rospy.Publisher("/ankle_joint/theta_admittance", Float32, queue_size=2)

        self.run()

    def torque_callback(self, msg):
        self.net_torque = msg.data

    def run(self):
        rate = rospy.Rate(self.control_rate)
        rospy.loginfo("Admittance Controller Started")
        rospy.loginfo("Mass: " + str(self.mass) + " kg·m²")
        rospy.loginfo("Damping: " + str(self.damping) + " N·m·s/rad")
        rospy.loginfo("Stiffness: " + str(self.stiffness) + " N·m/rad")
        while not rospy.is_shutdown():
            if not self.net_torque == None:
                # === Admittance equation: M*a + B*v + K*x = torque ===
                acc = (self.net_torque - self.damping * self.admittance_velocity - self.stiffness * self.admittance_angle) / self.mass
                self.admittance_velocity += acc * self.dt
                self.admittance_angle += self.admittance_velocity * self.dt
                self.theta_admittance_pub.publish(float(self.admittance_angle))

                self.net_torque = None

            rate.sleep()


if __name__ == "__main__":
    try:
        AdmittanceController()
    except rospy.ROSInterruptException:
        pass
