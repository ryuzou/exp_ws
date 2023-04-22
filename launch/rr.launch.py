import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ros2_socketcan_interface_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("socketcan_interface") + "/launch/socketcan_interface.launch.py"]))
    ros2_tcp_interface_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("tcp_interface") + "/launch/tcp_interface.launch.py"]))
    mechanam_node_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("mechanam_node") + "/launch/mechanam_node.launch.py"]))
    rr_manual_node_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("rr_manual_node") + "/launch/rr_manual_node.launch.py"]))
    return launch.LaunchDescription([
        ros2_socketcan_interface_launch,
        ros2_tcp_interface_launch,
        mechanam_node_launch,
        rr_manual_node_launch
    ])