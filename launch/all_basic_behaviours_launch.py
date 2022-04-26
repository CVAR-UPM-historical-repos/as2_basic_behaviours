from os.path import join

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_takeoff = join(
        get_package_share_directory('takeoff_behaviour'),
        'config',
        'takeoff_behaviour.yaml'
    )
    config_land = join(
        get_package_share_directory('land_behaviour'),
        'config',
        'land_behaviour.yaml'
    )
    config_goto = join(
        get_package_share_directory('goto_behaviour'),
        'config',
        'goto_behaviour.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        Node(
            package='takeoff_behaviour',
            executable='takeoff_behaviour_node',
            name='takeoff_behaviour_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[config_takeoff],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='land_behaviour',
            executable='land_behaviour_node',
            name='land_behaviour_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[config_land],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='goto_behaviour',
            executable='goto_behaviour_node',
            name='goto_behaviour_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[config_goto],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='as2_basic_behaviours',
            executable='follow_path_behaviour_node',
            name='follow_path_behaviour_node',
            namespace=LaunchConfiguration('drone_id'),
            output='screen',
            emulate_tty=True
        )
    ])