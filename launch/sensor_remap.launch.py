import launch
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = launch.LaunchDescription()

    # Create robot name argument for the launch file and read it
    ld.add_action(
        DeclareLaunchArgument(
            'robot',
            default_value='puddles',
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

    # add the depth remap
    ld.add_action(  
        Node(
            package='riptide_gazebo2',
            executable='depth_remap',
            name='depth_remap',
            respawn=True,
            output='screen',
            namespace=robot,
        )
    )

    # add the kill switch publisher
    ld.add_action(
        Node(
            package='riptide_gazebo2',
            executable='kill_switch_publisher',
            name='kill_switch_publisher',
            respawn=True,
            output='screen',
            namespace=robot,
        )
    )

    # add the Imu republisher
    ld.add_action(  
        Node(
            package='riptide_gazebo2',
            executable='imu_remap',
            name='imu_remap',
            respawn=True,
            output='screen',
            namespace=robot,
            
            # use the parameters on the node
            parameters = [
                {'vehicle_config': configFile}
            ]
        )
    )

    # add the thrust republisher
    ld.add_action(  
        Node(
            package='riptide_gazebo2',
            executable='thrust_remap',
            name='thrust_remap',
            respawn=True,
            output='screen',
            namespace=robot,
            
            # use the parameters on the node
            parameters = [
                {'vehicle_config': configFile}
            ]
        )
    )

    # add the camera publisher
    ld.add_action(  
        ComposableNodeContainer(
            package='rclcpp_components', executable='component_container',
            name='stereo_image_proc_container', 
            namespace=[robot, "/stereo"],
            composable_node_descriptions=[
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    parameters=[{
                        'approximate_sync': True,
                        'use_system_default_qos': False,
                    }]
                ),
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    parameters=[{
                        'approximate_sync': True,
                        'use_system_default_qos': False,
                    }]
                ),
            ],
        ),
    )
    
    return ld