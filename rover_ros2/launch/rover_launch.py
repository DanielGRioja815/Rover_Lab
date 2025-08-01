# rover_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('rover_ros2')
    world_path = os.path.join(package_dir, 'worlds', 'Rover_Ros.wbt')

    webots = WebotsLauncher(
        world=world_path
    )

    rover_controller = Node(
        package='rover_ros2',
        executable='rover_controller',
        output='screen'
    )

    arm_controller = Node(
        package='rover_ros2',
        executable='arm_controller',
        output='screen'
    )

    return LaunchDescription([
        webots,
        rover_controller,
        arm_controller
    ])