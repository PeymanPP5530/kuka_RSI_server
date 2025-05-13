from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    test_ros2_control_kuka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("kuka_ros2_control_support"),
                "launch",
                "test_ros2_control_kuka.launch.py"
            ])
        )
    )

    ros_server_node = Node(
        package="kuka_rsi_ros_interface",
        executable="kuka_rsi_ros",
        output="screen"
    )

    return LaunchDescription([
        test_ros2_control_kuka_launch,
        ros_server_node
    ])
