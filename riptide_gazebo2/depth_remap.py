#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Header
from riptide_msgs2.msg import Depth

class DepthRemap(Node):
    def __init__(self):
        super().__init__('depth_remap')
        self.sub = self.create_subscription(FluidPressure, "depth/pressure", self.depthCb, qos_profile_system_default)
        self.pub = self.create_publisher(Depth, "depth/raw", qos_profile_sensor_data)
        self.surfacePressure = 101.325
        self.kPaPerM = 9.80638

    def depthCb(self, msg):
        depth = Depth()
        depth.header = msg.header
        depth.depth = (self.surfacePressure - msg.fluid_pressure)/self.kPaPerM
        depth.variance = msg.variance / (self.kPaPerM ** 2)
        self.pub.publish(depth)

def main(args=None):
    rclpy.init(args=args)

    node = DepthRemap()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    