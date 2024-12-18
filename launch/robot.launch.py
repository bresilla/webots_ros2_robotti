import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_robotti')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'robotti_webots.urdf')
    robotti_driver = WebotsController(
        robot_name='Robotti',
        parameters=[
            {'robot_description': robot_description_path}
        ],
        respawn=True
    )


    drone_description_path = os.path.join(package_dir, 'resource', 'mavic_webots.urdf')
    mavic_driver = WebotsController(
        robot_name='Mavic2Pro',
        parameters=[
            {'robot_description': drone_description_path},
        ],
        respawn=True
    )


    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='farm.wbt'),
        webots,
        webots._supervisor,
        robotti_driver,
        mavic_driver,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
