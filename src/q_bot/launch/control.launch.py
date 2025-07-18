from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                os.path.join(
                    get_package_share_directory("q_bot"),
                    "config",
                    "skid_control.yaml"
                ),
                {"use_sim_time": True}
            ],
            remappings=[
                ('/cmd_vel', '/diff_cont/cmd_vel_unstamped')
            ],
            output="screen"
        )
    ])


