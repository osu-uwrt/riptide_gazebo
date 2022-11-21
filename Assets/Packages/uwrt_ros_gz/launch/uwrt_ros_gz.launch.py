import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="uwrt_ros_gz",
            executable="uwrt_ros_gz_bridge",
            name="ros_gz_bridge",
        )
    ])
