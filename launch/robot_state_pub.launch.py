import launch
from launch_ros.actions import Node
from launch.launch_description import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import xacro
import os

robot = 'puddles'

modelPath = os.path.join(
    get_package_share_directory('riptide_descriptions2'),
    'robots',
    robot + '.xacro'
)

print('using model definition', modelPath)

xacroData = xacro.process_file(modelPath,  mappings={
    'debug': 'true',
    'namespace': robot,
    'inertial_reference_frame': 'world'
}).toxml()

xacroFilePath = os.path.join(
    '/tmp',
    '{}.xml'.format(robot)
)


def generate_launch_description():
    robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': xacroData}],  # Use subst here
        arguments=['--ros-args', '--log-level', 'DEBUG'],
    )

    return LaunchDescription([
        robot_state_publisher
    ])
