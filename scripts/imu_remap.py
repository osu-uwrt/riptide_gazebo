#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from riptide_msgs.msg import Depth
import yaml

class ImuRemap():
    def __init__(self):
        # Get the mass and COM
        with open(rospy.get_param('~vehicle_config'), 'r') as stream:
            vehicle = yaml.safe_load(stream)
            imu_topic = vehicle["imu"]["topic"]

        self.sub = rospy.Subscriber(imu_topic + "/sim", Imu, self.imuCb)
        self.pub = rospy.Publisher(imu_topic, Imu, queue_size=10)

    def imuCb(self, msg):
        msg.header.frame_id = msg.header.frame_id.replace("_revolute", "")
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('imu_remap')
    ImuRemap()
    rospy.spin()
    