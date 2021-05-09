#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Header
from riptide_hardware.msg import Depth
class DepthRemap():
    def __init__(self):
        self.sub = rospy.Subscriber("depth/pressure", FluidPressure, self.depthCb, queue_size=1)
        self.pub = rospy.Publisher("depth/raw", Depth, queue_size=10)
        self.surfacePressure = 101.325
        self.kPaPerM = 9.80638

    def depthCb(self, msg):
        depth = Depth()
        depth.header = msg.header
        depth.depth = (self.surfacePressure - msg.fluid_pressure)/self.kPaPerM
        depth.variance = msg.variance / (self.kPaPerM ** 2)
        self.pub.publish(depth)

if __name__ == '__main__':
    rospy.init_node('depth_remap')
    DepthRemap()
    rospy.spin()
    