import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ros2_socketcan_interface_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("socketcan_interface") + "/launch/socketcan_interface.launch.py"]))
    ros2_tcp_interface_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("tcp_interface") + "/launch/tcp_interface.launch.py"]))
    stear_node_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("stear_node") + "/launch/stear_node.launch.py"]))
    er_angle_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("er_angle_node") + "/launch/er_angle_node.launch.py"]))
    rr_manual_node_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("er_manual_node") + "/launch/er_manual_node.launch.py"]))
    return launch.LaunchDescription([
        ros2_socketcan_interface_launch,
        ros2_tcp_interface_launch,
        stear_node_launch,
        er_angle_launch,
        rr_manual_node_launch
    ])