#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import LaserScan
import numpy as np
import array

part_1_angle = 0
part_2_angle = 120
part_3_angle = 240
part_4_angle = 360

class LidarExtractor(Node):
    def __init__(self,name):
        super().__init__(name)
        self.create_subscription(LaserScan,"scan_raw",self.callback_scan,10)
        self.pub = self.create_publisher(LaserScan,"scan",10)


    def callback_scan(self,data):
        msg = LaserScan()
        msg.header = data.header
        msg.angle_min = data.angle_min
        msg.angle_max = data.angle_max
        msg.angle_increment = data.angle_increment
        msg.time_increment = data.time_increment
        msg.scan_time = data.scan_time
        msg.range_min = data.range_min
        msg.range_max = data.range_max

        total_deg_with_step = len(data.ranges)
        total_deg = math.degrees(msg.angle_max - msg.angle_min)

        step = total_deg_with_step/total_deg
       
        
        zeros_array = array.array('f', [0.0] * int(119 * step))


        data.ranges[int(121*step):int(240*step)] = zeros_array
        data.intensities[int(121*step):int(240*step)] = zeros_array

        msg.ranges = array.array('f', data.ranges[int(part_1_angle*step):int(part_2_angle*step)] + data.ranges[int((part_2_angle)*step):int(part_3_angle*step)] + data.ranges[int((part_3_angle)*step):int(part_4_angle*step)])
        msg.intensities = array.array('f', data.intensities[int(part_1_angle*step):int(part_2_angle*step)] + data.intensities[int((part_2_angle)*step):int(part_3_angle*step)] + data.intensities[int((part_3_angle)*step):int(part_4_angle*step)])

        self.pub.publish(msg)



def main():
    rclpy.init()
    
    node = LidarExtractor("scan_filtering_node")
    node.get_logger().info("Lidar extractor node initialized")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()