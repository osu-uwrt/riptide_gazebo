import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    ld = launch.LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            'world',
            default_value='cutie.world',
            description='name of the world to use'
        )
    )

    gazeboLaunch = launch.substitutions.PathJoinSubstitution([
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])

    world = launch.substitutions.PathJoinSubstitution([
        get_package_share_directory('riptide_gazebo2'),
        'worlds',
        launch.substitutions.LaunchConfiguration('world')
    ])
    
    ld.add_action( 
        GroupAction([
            PushRosNamespace('gazebo'), 
            IncludeLaunchDescription(
                launch_description_source= PythonLaunchDescriptionSource(gazeboLaunch),
                launch_arguments={
                    'world': world,
                    'gui_required': 'true',
                    'server_required': 'true',
                    'verbose': 'true',
                    'extra_gazebo_args': '-s libgazebo_ros_force_system.so --ros-args -r gazebo:__ns:=/gazebo'
                }.items(),
            )
        ])
    )
        

    return ld