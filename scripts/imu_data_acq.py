#!/usr/bin/env python3

import rospy
import serial
from ankle_exoskeleton.msg import IMUData

class IMUBNOArduinoNode:
    def __init__(self, port, name):
        rospy.init_node(name + "_imu", anonymous=True)
        self.imu_pub = rospy.Publisher(name, IMUData, queue_size=1)
        self.serial_port = None
        self.calibrated = False
        self.port = port
        self.name = name
        self.calibrate_imu()

    def calibrate_imu(self):
        # Open serial port to Arduino
        self.serial_port = serial.Serial(self.port, 250000)
        rospy.loginfo("Calibrating IMU on port {}...".format(self.port))
        while not rospy.is_shutdown():
            calibration_str = self.serial_port.readline().strip().decode()
            #print(len(calibration_str))
            if len(calibration_str) > 20:
                rospy.loginfo("IMU on port {} already calibrated!".format(self.port))
                self.calibrated = True
                break
            try:
                system, gyro, accel, mag = map(int, calibration_str.split(","))
                rospy.loginfo("Calibration on port {}: Sys=%d Gyro=%d Accel=%d Mag=%d".format(self.port), system, gyro, accel, mag)
                if system == -1 and gyro == -1 and accel == -1 and mag == -1:
                    rospy.loginfo("IMU on port {} not found!".format(self.port))
                # Check if IMU is fully calibrated
                if system == 3 and gyro == 3 and accel == 3 and mag == 3:
                    rospy.loginfo("IMU on port {} calibrated!".format(self.port))
                    self.calibrated = True
                    break
            except:
                rospy.logerr("Incomplete Data")

    def publish_imu_data(self):
        if not self.calibrated:
            return

        try:
            data_str = self.serial_port.readline().strip().decode()
            ox, oy, oz, gx, gy, gz, ax, ay, az, gvx, gvy, gvz, qx, qy, qz, qw = map(float, data_str.split(","))
            # Publish sensor data
            imu_msg = IMUData()
            imu_msg.accel_x = ax
            imu_msg.accel_y = ay
            imu_msg.accel_z = az
            imu_msg.gyro_x = gx
            imu_msg.gyro_y = gy
            imu_msg.gyro_z = gz
            imu_msg.quat_x = qx
            imu_msg.quat_y = qy
            imu_msg.quat_z = qz
            imu_msg.quat_w = qw
            imu_msg.euler_x = ox
            imu_msg.euler_y = oy
            imu_msg.euler_z = oz
            imu_msg.gravity_x = gvx
            imu_msg.gravity_y = gvy
            imu_msg.gravity_z = gvz

            self.imu_pub.publish(imu_msg)
        except ValueError as e:
            rospy.logwarn(f"Failed to parse IMU data: {e}")
        except serial.SerialException as e:
            rospy.logerr(f"Serial port error: {e}")

if __name__ == '__main__':
    try:
        port = rospy.get_param('port_name', '')
        name = rospy.get_param('node_name', '')
        print(port)
        print(name)
        print("----")

        imu_node = IMUBNOArduinoNode(port,name)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            imu_node.publish_imu_data()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
