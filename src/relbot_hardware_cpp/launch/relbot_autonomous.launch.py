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
    camera_source_arg = DeclareLaunchArgument(
        "camera_source",
        default_value="v4l2",
        description="Camera input mode: 'v4l2' (/dev/video0) or 'ros_topic' (/image)",
    )
    rotate_180_arg = DeclareLaunchArgument(
        "rotate_180",
        default_value="true",
        description="Rotate camera frame 180 degrees (RELbot camera is mounted upside down)",
    )
    enable_motion_arg = DeclareLaunchArgument(
        "enable_motion",
        default_value="false",
        description="Publish non-zero /cmd_vel and /arm/command (false = safe detection-only test)",
    )
    follow_only_arg = DeclareLaunchArgument(
        "follow_only",
        default_value="false",
        description="Continuously follow the green pickup ball (no pick-and-place state transitions)",
    )
    manual_mode_arg = DeclareLaunchArgument(
        "manual_mode",
        default_value="false",
        description="Start in manual teleop mode (controlled via terminal keys or web UI)",
    )
    search_when_lost_arg = DeclareLaunchArgument(
        "search_when_lost",
        default_value="false",
        description="Spin in place to search when ball is lost in follow_only mode",
    )
    follow_standoff_arg = DeclareLaunchArgument(
        "follow_standoff_m",
        default_value="0.25",
        description="Desired standoff distance when following the ball (meters)",
    )
    http_port_arg = DeclareLaunchArgument(
        "http_port",
        default_value="8080",
        description="Port for built-in MJPEG live detection preview (0 to disable)",
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
            {"serial_port": "/dev/null"},
        ],
    )

    perception_nav_node = Node(
        package="relbot_hardware_cpp",
        executable="relbot_perception_nav_node",
        name="relbot_perception_nav_node",
        output="screen",
        parameters=[
            {
                "esp32_serial_port": LaunchConfiguration("serial_port"),
                "camera_source": LaunchConfiguration("camera_source"),
                "rotate_180": LaunchConfiguration("rotate_180"),
                "enable_motion": LaunchConfiguration("enable_motion"),
                "follow_only": LaunchConfiguration("follow_only"),
                "manual_mode": LaunchConfiguration("manual_mode"),
                "search_when_lost": LaunchConfiguration("search_when_lost"),
                "follow_standoff_m": LaunchConfiguration("follow_standoff_m"),
                "http_port": LaunchConfiguration("http_port"),
            }
        ],
    )

    return LaunchDescription(
        [
            serial_port_arg,
            camera_source_arg,
            rotate_180_arg,
            enable_motion_arg,
            follow_only_arg,
            manual_mode_arg,
            search_when_lost_arg,
            follow_standoff_arg,
            http_port_arg,
            hardware_node,
            perception_nav_node,
        ]
    )
