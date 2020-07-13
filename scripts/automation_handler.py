#!/usr/bin/env python
import roslaunch
import rospy

package = 'riptide_gazebo'
launch_file = 'world.launch'

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

num_runs = 3


for i in range(num_runs):
    cli_args = [package, launch_file, 'world:=run' + str(i)]
    roslaunch_args = cli_args[2:]
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    print roslaunch_file
    launch_file = [(roslaunch_file, roslaunch_args)] 
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)

    rospy.loginfo("started")
    launch.start()
    rospy.sleep(60)
    launch.shutdown()
