#!/usr/bin/env python

import rospy
import yaml
import math
import numpy
from std_msgs.msg import Float32MultiArray
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class ThrustConverter(object):

    def __init__(self):
        self._vehicle_config_file = rospy.get_param("~vehicle_config")
        self._pubs = {}
        
        with open(self._vehicle_config_file, 'r') as stream:
            self._coeff = yaml.safe_load(stream)['thruster']['rotor_constant']

        for i in range(len(self._thrusters)):
            input_topic = "thrusters/%d/input" % i
            pub = rospy.Publisher(input_topic, FloatStamped, queue_size=1)
            self._pubs[i] = pub
        self._sub = rospy.Subscriber("thruster_forces", Float32MultiArray, self.command_cb)
        

    def command_cb(self, msg):
        """Convert thrust to angular velocity commands for uuv_thruster_ros_plugin"""
        for i in range(len(self._thrusters)):
            cmd = FloatStamped()
            force = msg.data[i]

            # Compute angular velocity from force
            sign = 1 if force >= 0 else -1
            ang_vel = sign * math.sqrt(abs(force / self._coeff))
            cmd.header = msg.header
            cmd.data = ang_vel
            self._pubs[i].publish(cmd)

if __name__ == "__main__":
    rospy.init_node('thrust_converter')
    tc = ThrustConverter()
    rospy.spin()