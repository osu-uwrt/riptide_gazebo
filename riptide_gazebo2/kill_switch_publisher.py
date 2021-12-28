#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
class killSwitchPublisher():
    def __init__(self):
        self.pub = rospy.Publisher("state/kill_switch", Bool, queue_size=10)
        self.timer = rospy.timer.Timer(rospy.Duration(0.2), self.tick)

    def tick(self, event):
        self.pub.publish(True)

if __name__ == '__main__':
    rospy.init_node('kill_switch_publisher')
    killSwitchPublisher()
    rospy.spin()