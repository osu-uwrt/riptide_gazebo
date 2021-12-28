#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from sensor_msgs.msg import Imu
import yaml

class ImuRemap(Node):
    def __init__(self):
        super().__init__('imu_remap')

        self.declare_parameter('vehicle_config', '/config/tempest.yaml')
        self._vehicle_config_path = self.get_parameter("vehicle_config")

        # pull vehicle parameters
        with open(self._vehicle_config_path, 'r') as stream:
            vehicle = yaml.safe_load(stream)
            imu_topic = vehicle["imu"]["topic"]

        self.sub = self.create_subscription(Imu, imu_topic + "/sim", self.imuCb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Imu, imu_topic, qos_profile_sensor_data)

    def imuCb(self, msg):
        msg.header.frame_id = msg.header.frame_id.replace("_revolute", "")
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = ImuRemap()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    