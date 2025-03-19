from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    
    imu_port_arg = DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0')
    
    imu_node = Node(
                package='imu_driver',
                executable='imu_driver_exe',
                name='imu_driver_exe',
                parameters=[
                    {"port_name": LaunchConfiguration("imu_port")}
                ],  # Removed the tilde
                output='both'
            )
    return LaunchDescription([
        imu_port_arg,
        imu_node
    ])

