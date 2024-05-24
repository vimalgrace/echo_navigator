#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf_transformations

import py_qmc5883l
import time

sensor = py_qmc5883l.QMC5883L()


sensor.mode_continuous()

sensor.declination = -1.48

sensor.calibration = [[ 1.00891199e+00,-1.66022293e-02 , 2.15077417e+03],
                        [-1.66022293e-02, 1.03092846e+00, 2.42391646e+03],
                        [ 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]


class Imu_sensor(Node):
    def __init__(self,name):
        super().__init__(name)
        self.create_timer(0.1,self.imu_callback)
        self.pub = self.create_publisher(Imu,"imu",10)


    def imu_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu"
        imu_data = sensor.get_bearing()

        quat_imu = tf_transformations.quaternion_from_euler(0.0,0.0,imu_data)

        msg.orientation.x = quat_imu[0]
        msg.orientation.y = quat_imu[1]
        msg.orientation.z = quat_imu[2]
        msg.orientation.w = quat_imu[3]

        print(f"yaw = {imu_data}")

        self.pub.publish(msg)












def main():
    rclpy.init()
    node = Imu_sensor("imu_node")
    node.get_logger().info("Imu Node initialized")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
