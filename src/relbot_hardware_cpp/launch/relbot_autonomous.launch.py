from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial device for the Microcontroller bridge (Dynamixel XL430 + MG996R)",
    )

    hw_config = PathJoinSubstitution(
        [FindPackageShare("relbot_hardware_cpp"), "config", "hardware_params.yaml"]
    )

    hardware_node = Node(
        package="relbot_hardware_cpp",
        executable="relbot_hardware_node",
        name="relbot_hardware_node",
        output="screen",
        parameters=[
            hw_config,
            {"serial_port": LaunchConfiguration("serial_port")},
        ],
    )

    navigation_node = Node(
        package="relbot_navigation_rs",
        executable="relbot_navigation_rs",
        name="relbot_navigation_node",
        output="screen",
    )

    return LaunchDescription(
        [
            serial_port_arg,
            hardware_node,
            navigation_node,
        ]
    )
