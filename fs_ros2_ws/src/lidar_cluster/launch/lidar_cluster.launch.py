from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the config file
    config_dir = os.path.join(get_package_share_directory('lidar_cluster'), 'config')
    config_file = os.path.join(config_dir, 'lidar_cluster.yaml')
    
    # Declare the config file as a launch argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the ROS2 parameters file to use')
    
    # Create the LiDAR cluster node with the parameters loaded from the config file
    lidar_cluster_node = Node(
        package='lidar_cluster',
        executable='lidar_cluster_node',
        name='lidar_cluster_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True
    )
    
    # Return the launch description with the node
    return LaunchDescription([
        config_file_arg,
        lidar_cluster_node
    ])