import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('rover')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_rover.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', '4_wheeled_robot.wbt')
    )

    rover_ros2 = WebotsController(
        robot_name='robot(1)',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    return LaunchDescription([
        webots,
        rover_ros2,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])