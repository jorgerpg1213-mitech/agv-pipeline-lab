from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=['/root/agv_ws/src/agv_bringup/config/nav2_params.yaml']
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=['/root/agv_ws/src/agv_bringup/config/nav2_params.yaml']
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=['/root/agv_ws/src/agv_bringup/config/nav2_params.yaml']
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=['/root/agv_ws/src/agv_bringup/config/nav2_params.yaml']
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator',
                'behavior_server'
            ]
        }]
    )

    return LaunchDescription([
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        lifecycle_manager
    ])
