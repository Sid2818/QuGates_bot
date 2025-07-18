import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
#from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='q_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    """gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )"""
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )


    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'q_bot'],
                        output='screen')

    # skid_control = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         os.path.join(
    #             get_package_share_directory("q_bot"),  # Replace if your package name differs
    #             "config",
    #             "skid_control.yaml"
    #         ),
    #         {"use_sim_time": True}
    #     ],
    #     output="screen"
    # )

    pkg_q_bot = get_package_share_directory('q_bot')
    param_file_path = os.path.join(pkg_q_bot, 'config', 'skid_control.yaml')
    # ros2_control_node
    # ros2_control_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[param_file_path],
    #     output='screen'
    # )

    # # Spawners for joint_state_broadcaster and skid_steer_controller
    # joint_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
    #     output='screen'
    # )

    delayed_spawn_entity = TimerAction(period=6.0, actions=[spawn_entity])

    # spawn_skid_controller = TimerAction(
    #     period=15.0,  # delay to allow Gazebo+robot to fully spawn
    #     actions=[
    #         Node(
    #             package="controller_manager",
    #             executable="spawner",
    #             arguments=["skid_steer_controller", "--controller-manager", "/controller_manager"],
    #             output="screen"
    #         )
    #     ]
    # )

    DeclareLaunchArgument(
        'robot_description',
        default_value=Command(['xacro ', PathJoinSubstitution([pkg_q_bot, 'description', 'robot.urdf.xacro'])]),
        description='URDF for the robot',
)

    delayed_ros2_control = TimerAction(
        period=70.0,  # Wait 5 seconds for robot to be spawned
        actions=[
            Node(  
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[LaunchConfiguration('robot_description'),
                    PathJoinSubstitution([pkg_q_bot, "config", "skid_control.yaml"]),
                    {"use_sim_time": True}
                ],
                output="screen"
            )
        ]
    )

    delayed_spawners = TimerAction(
        period=90.0,  # Wait more for ros2_control_node to initialize
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
                output="screen"
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["skid_steer_controller", "--controller-manager-timeout", "60"],
                output="screen"
            ),
        ]
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        twist_mux,
        gazebo,
        delayed_spawn_entity,
        delayed_ros2_control,
        delayed_spawners,
        #ros2_control_node,
        #spawn_skid_controller,
        #joint_broadcaster_spawner,

    ])

#ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity bot_name
#ros2 launch q_bot launch_sim.launch.py
#ros2 launch q_bot launch_sim.launch.py world:=./src/q_bot/worlds/obstacles.world
#ros2 launch slam_toolbox online_async_launch.py params_file:=./src/q_bot/config/mapper_params_online_async.yaml use_sim_time:=true
#ros2 run teleop_twist_keyboard teleop_twist_keyboard
#//ros2 run twist_mux twist_mux --ros-args --params-file ./src/q_bot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped
#ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
#ros2 launch q_bot navigation_launch.py use_sim_time:=true
#colcon build --packages-select q_bot --symlink-install
