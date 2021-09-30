from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='as2_basic_behaviours',
            # namespace='drone0',
            executable='takeoff_behaviour_node',
            name='takeoff_behaviour_node',
            remappings=[
                ('/drone0/self_localization/odom', '/drone0/platform/odometry')
            ],
            output='screen'
        ),
        Node(
            package='as2_basic_behaviours',
            # namespace='drone0',
            executable='land_behaviour_node',
            name='land_behaviour_node',
            remappings=[
                ('/drone0/self_localization/odom', '/drone0/platform/odometry')
            ],
            output='screen'
        ),
        Node(
            package='as2_basic_behaviours',
            # namespace='drone0',
            executable='gotowaypoint_behaviour_node',
            name='gotowaypoint_behaviour_node',
            remappings=[
                ('/drone0/self_localization/odom', '/drone0/platform/odometry')
            ],
            output='screen'
        )
    ])