from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_driver',
            executable='imu_driver_exe',
            name='imu_driver_exe',
          #  parameters=[{"port_name": "/dev/imu_usb"}],  # Removed the tilde
            output='both'
        )
    ])

