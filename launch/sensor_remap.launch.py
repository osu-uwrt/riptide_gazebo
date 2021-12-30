import launch
import os
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.actions.group_action import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ld = launch.LaunchDescription()

    # Create robot name argument for the launch file and read it
    ld.add_action(
        DeclareLaunchArgument(
            'robot',
            default_value='tempest',
            description='name of the robot to use'
        )
    )
    robot = launch.substitutions.LaunchConfiguration('robot')

    # grab the robot config file name here
    configFile = [
        launch.substitutions.PathJoinSubstitution([
            get_package_share_directory('riptide_descriptions2'),
            'config',
            robot
        ]),
        launch.substitutions.TextSubstitution(text='.yaml')
    ]

    stereoLaunch = os.path.join(
        get_package_share_directory('stereo_image_proc'),
        'launch',
        'stereo_image_proc.launch.py'
    )

    ld.add_action( 
        GroupAction([
            PushRosNamespace(robot), 
            Node(package='riptide_gazebo2', executable='depth_remap', name='depth_remap', respawn=True, output='screen'),
            Node( 
                package='riptide_gazebo2', executable='imu_remap', name='imu_remap', respawn=True, output='screen',
                parameters = [
                    {'vehicle_config': configFile}
                ]
            ),
            Node(
                package='riptide_gazebo2', executable='thrust_remap', name='thrust_remap', respawn=True, output='screen',
                parameters = [
                    {'vehicle_config': configFile}
                ]
            )
        ])
    )

    ld.add_action(IncludeLaunchDescription(
            launch_description_source= PythonLaunchDescriptionSource(stereoLaunch),
            launch_arguments={
                'launch_image_proc': 'true',
                'approximate_sync': 'true',
                'left_namespace': [robot, '/stereo/left'],
                'right_namespace': [robot, '/stereo/right'],
            }.items(),
        )
    )
    
    return ld