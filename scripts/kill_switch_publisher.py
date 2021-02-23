#!/usr/bin/env python3

import rospy
from riptide_msgs.msg import SwitchState
from std_msgs.msg import Header
class killSwitchPublisher():
    def __init__(self):
        self.pub = rospy.Publisher("state/switches", SwitchState, queue_size=10)
        self.timer = rospy.timer.Timer(rospy.Duration(0.2), self.tick)

    def tick(self, event):
        self.pub.publish(SwitchState(Header(), True, False, False, False, False, False))

if __name__ == '__main__':
    rospy.init_node('kill_switch_publisher')
    killSwitchPublisher()
    rospy.spin()