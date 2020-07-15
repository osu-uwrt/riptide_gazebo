#!/usr/bin/env python
import roslaunch
import rospy

num_runs = 3
package = 'riptide_gazebo'
launch_file = 'world.launch'

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

for i in range(num_runs):
    arg = 'world:=run' + str(i)
    cli_args = [package, launch_file, arg]
    roslaunch_args = cli_args[2:]
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    exec('launch_file' + str(i) + ' = [(roslaunch_file, roslaunch_args)]')
    exec('launch' + str(i) + '= roslaunch.parent.ROSLaunchParent(uuid, launch_file' + str(i) + ')')
    rospy.loginfo("started")
    exec('launch' + str(i) + '.start()')
    rospy.sleep(60)
    exec('launch' + str(i) + '.shutdown()')
    rospy.sleep(10)