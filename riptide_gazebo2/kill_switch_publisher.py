#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from std_msgs.msg import Bool

class killSwitchPublisher(Node):
    def __init__(self):
        self.pub = self.create_publisher(Bool, "state/kill_switch", qos_profile_system_default)
        self.timer = self.create_timer(0.2, self.tick)

    def tick(self, event):
        self.pub.publish(True)

def main(args=None):
    rclpy.init(args=args)

    node = killSwitchPublisher()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    