from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    agv_bringup_dir = get_package_share_directory("agv_bringup")
    rplidar_dir = get_package_share_directory("rplidar_ros")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    ekf_params = os.path.join(agv_bringup_dir, "config", "ekf.yaml")
    nav2_params = os.path.join(agv_bringup_dir, "config", "nav2_params.yaml")
    slam_params = os.path.join(
        agv_bringup_dir,
        "config",
        "slam_toolbox_online.yaml"
    )

    esp32_bridge = Node(
        package="esp32_bridge_node",
        executable="esp32_bridge_node",
        name="esp32_bridge_node",
        output="screen",
        parameters=[{"cmd_vel_topic": "/cmd_vel_safe"}]
    )

    odom_node = Node(
        package="odom_node",
        executable="odom_node",
        name="odom_node",
        output="screen",
        parameters=[{"publish_tf": False}]
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params]
    )

    perception_node = Node(
        package="perception_node",
        executable="perception_node",
        name="perception_node",
        output="screen"
    )

    safety_policy_node = Node(
        package="safety_policy_node",
        executable="safety_policy_node",
        name="safety_policy_node",
        output="screen",
        parameters=[{
            "cmd_in_topic": "/cmd_vel",
            "cmd_out_topic": "/cmd_vel_safe",
            "detections_topic": "/detections",
            "image_width_px": 1280.0
        }]
    )

    tf_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_base_footprint",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "base_link", "--child-frame-id", "base_footprint"]
    )

    tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_laser",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "base_link", "--child-frame-id", "laser"]
    )

    tf_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_odom_to_base_link",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "odom", "--child-frame-id", "base_link"]
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_dir, "launch", "rplidar_a1_launch.py")
        ),
        launch_arguments={
            "serial_port": "/dev/rplidar"
        }.items()
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "False",
            "slam_params_file": slam_params
        }.items()
    )

    nav2 = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "False",
                    "params_file": nav2_params
                }.items()
            )
        ]
    )

    watchdog_process = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "source /opt/ros/humble/setup.bash && source /root/agv_ws/install/setup.bash && python3 /root/agv_ws/src/agv_bringup/scripts/agv_watchdog_node.py"
        ],
        output="screen"
    )
    recovery_manager_process = ExecuteProcess(

        cmd=[

            "bash",

            "-c",

            "source /opt/ros/humble/setup.bash && source /root/agv_ws/install/setup.bash && python3 /root/agv_ws/src/agv_bringup/scripts/agv_recovery_manager.py"

        ],

        output="screen"

    )


    return LaunchDescription([
        esp32_bridge,
        odom_node,
        ekf_node,
        # perception_node,
        safety_policy_node,
        tf_base_footprint,
        tf_laser,
        tf_odom,
        rplidar,
        slam,
        nav2,
        watchdog_process,
        recovery_manager_process,
    ])
