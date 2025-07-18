import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='qubot' #<--- CHANGE ME

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
                                   '-entity', 'qubot'],
                        output='screen')


    delayed_spawn = TimerAction(period=10.0, actions=[spawn_entity])

    # Launch them all!
    return LaunchDescription([
        rsp,
        twist_mux,
        gazebo,
        delayed_spawn,
    ])

#ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity bot_name
#ros2 launch qubot launch_sim.launch.py
#ros2 launch qubot launch_sim.launch.py world:=./src/qubot/worlds/obstacles.world
#ros2 launch slam_toolbox online_async_launch.py params_file:=./src/qubot/config/mapper_params_online_async.yaml use_sim_time:=true
#ros2 run teleop_twist_keyboard teleop_twist_keyboard
#//ros2 run twist_mux twist_mux --ros-args --params-file ./src/qubot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped
#ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
#ros2 launch qubot navigation_launch.py use_sim_time:=true

