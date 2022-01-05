import launch
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration as LC
import os
import xacro

# evaluates LaunchConfigurations in context for use with xacro.process_file(). Returns a list of launch actions to be included in launch description
def evaluate_xacro(context, *args, **kwargs):
    robot = LC('robot').perform(context)
    debug = LC('debug').perform(context)
    x = LC('x').perform(context)
    y = LC('y').perform(context)
    z = LC('z').perform(context)
    roll = LC('roll').perform(context)
    pitch = LC('pitch').perform(context)
    yaw = LC('yaw').perform(context)

    modelPath = launch.substitutions.PathJoinSubstitution([
        get_package_share_directory('riptide_descriptions2'),
        'robots',
        robot + '.xacro'
    ]).perform(context)

    print('using model definition', modelPath)

    xacroData = xacro.process_file(modelPath,  mappings={'debug': debug, 'namespace': robot, 'inertial_reference_frame':'world'}).toxml()
    xacroFilePath = os.path.join(
        '/tmp',
        '{}.xml'.format(robot)    
    )
    
    f = open(xacroFilePath, "w")
    f.write(xacroData)
    f.close()

    print('Converted model path', xacroFilePath)

   # URDF spawner
    args=('-gazebo_namespace /gazebo '
        '-x %s -y %s -z %s -R %s -P %s -Y %s -entity %s -file %s' 
        %(x, y, z, roll, pitch, yaw, robot, xacroFilePath)).split()

    # Urdf spawner. NB: node namespace does not prefix the spawning service, 
    # as using a leading /
    # NB 2: node namespace prefixes the robot_description topic
    urdf_spawner = Node(
        name = 'urdf_spawner',
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=args
    )

    robot_state_publisher = Node(
        name = 'robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output = 'screen',
        parameters=[{'robot_description': xacroData}], # Use subst here
    )

    # Message to tf
    message_to_tf_launch = os.path.join(
        get_package_share_directory('uuv_assistants'),
        'launch',
        'message_to_tf.launch'
    )

    message_to_tf_launch = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(message_to_tf_launch),
            launch_arguments=[
                ('namespace', robot),
                ('world_frame', 'world'),
                ('child_frame_id', '/' + robot + '/base_link')
            ]
        )


    return [
        GroupAction([
            PushRosNamespace(robot), 
            urdf_spawner,
            robot_state_publisher
        ]),
        message_to_tf_launch
    ]

def generate_launch_description():
        return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='tempest', description='name of the robot to spawn'),
        DeclareLaunchArgument('debug', default_value='0', description='whether to put gazebo into debug mode'),
        DeclareLaunchArgument('x', default_value='0.0', description="X coordinate of the vehicle's initial position (in ENU)"),
        DeclareLaunchArgument('y', default_value='0.0',  description="Y coordinate of the vehicle's initial position (in ENU)"),
        DeclareLaunchArgument('z', default_value='0.0',  description="Z coordinate of the vehicle's initial position (in ENU)"),
        DeclareLaunchArgument('roll', default_value='0.0', description="Z coordinate of the vehicle's initial position (in ENU)"),
        DeclareLaunchArgument('pitch', default_value='0.0', description="Z coordinate of the vehicle's initial position (in ENU)"),
        DeclareLaunchArgument('yaw', default_value='0.0', description="Z coordinate of the vehicle's initial position (in ENU)"),
        OpaqueFunction(function=evaluate_xacro)
    ]) 

