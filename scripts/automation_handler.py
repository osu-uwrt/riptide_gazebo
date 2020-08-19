#!/usr/bin/env python
import roslaunch
import rospy
 
def launch_kill(num_runs, package, launch_file):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_files = []

    for i in range(num_runs):
        arg = 'world:=run' + str(i)
        cli_args = [package, launch_file, arg]
        roslaunch_args = cli_args[2:]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        launch_files.append([(roslaunch_file, roslaunch_args)])
        launch_files[i] = roslaunch.parent.ROSLaunchParent(uuid, launch_files[i])
        rospy.loginfo("run" + str(i) + "started")
        launch_files[i].start()
        rospy.sleep(60)
        launch_files[i].shutdown()
        rospy.sleep(10)
'''
    old exec implementation...bad practice

    for i in range(num_runs):
        arg = 'world:=run' + str(i)
        cli_args = [package, launch_file, arg]
        roslaunch_args = cli_args[2:]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        exec('launch_file' + str(i) + ' = [(roslaunch_file, roslaunch_args)]')
        exec('launch' + str(i) + '= roslaunch.parent.ROSLaunchParent(uuid, launch_file' + str(i) + ')')
        rospy.loginfo("run" + str(i) + "started")
        exec('launch' + str(i) + '.start()')
        rospy.sleep(60)
        exec('launch' + str(i) + '.shutdown()')
        rospy.sleep(10)
'''