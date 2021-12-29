#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
import yaml
import math
from std_msgs.msg import Float32MultiArray, Header
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class ThrustRemap(Node):
    def __init__(self):
        super().__init__('thrust_remap')

        self.declare_parameter('vehicle_config', '/config/tempest.yaml')
        self._vehicle_config_path = self.get_parameter("vehicle_config").value
        
        self._pubs = {}
        
        self.get_logger().info('Thrust remmaping using config file {}'.format(self._vehicle_config_path))
        with open(self._vehicle_config_path, 'r') as stream:
            config = yaml.safe_load(stream)
            self.num_of_thrusters = len(config['thrusters'])
            self._coeff = config['thruster']['rotor_constant']

        for i in range(self.num_of_thrusters):
            input_topic = "thrusters/thruster%d/input" % i
            self._pubs[i] = self.create_publisher(FloatStamped, input_topic, qos_profile_sensor_data)

        self._sub = self.create_subscription(Float32MultiArray, "thruster_forces", self.command_cb, qos_profile_system_default)

    def command_cb(self, msg):
        """Convert thrust to angular velocity commands for uuv_thruster_ros_plugin"""
        for i in range(self.num_of_thrusters):
            cmd = FloatStamped()
            force = msg.data[i]

            # Compute angular velocity from force
            sign = 1 if force >= 0 else -1
            ang_vel = sign * math.sqrt(abs(force / self._coeff))
            cmd.header = Header()
            cmd.data = ang_vel
            self._pubs[i].publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    node = ThrustRemap()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()