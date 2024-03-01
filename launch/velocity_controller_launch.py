from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    drive_mode = LaunchConfiguration("drive_mode")
    scale_linear_x = LaunchConfiguration("scale_linear_x")
    scale_angular_yaw = LaunchConfiguration("scale_angular_yaw")

    package_name = "my_joy_controller"
    joy_params = os.path.join(
        get_package_share_directory(package_name), "config", "joystick.yaml"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_params, {"use_sim_time": use_sim_time}],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[
            joy_params,
            {
                "use_sim_time": use_sim_time,
                "scale_linear": scale_linear_x,
                "scale_angular": scale_angular_yaw,
            },
        ],
        # remappings=[("/cmd_vel", cmd_vel_topic)],
    )

    robot_vel_controller_node = Node(
        package="gears_velocity_controller",
        executable="robot_vel_controller_node",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "cmd_vel_topic": cmd_vel_topic,
                "serial_port": serial_port,
                "baudrate": baudrate,
                "drive_mode": drive_mode,
            },
        ],
        # remappings=[("/cmd_vel", cmd_vel_topic)],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=False,
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="cmd_vel",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "baudrate",
                default_value=115200,
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "drive_mode",
                default_value="4WS",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "scale_linear_x",
                default_value=0.15,
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "scale_angular_yaw",
                default_value=0.15,
                description="Use sim time if true",
            ),
            joy_node,
            teleop_node,
            robot_vel_controller_node,
        ]
    )
