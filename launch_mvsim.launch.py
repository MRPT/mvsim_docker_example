# launch_mvsim.launch.py
from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument,
                            EmitEvent, LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import os


def generate_launch_description():
    mvsimDir = get_package_share_directory("mvsim")
    # print('mvsimDir: ' + mvsimDir)

    # args that can be set from the command line or a default will be used
    world_file_launch_arg = DeclareLaunchArgument(
        "world_file", default_value=TextSubstitution(
            text=os.path.join(mvsimDir, 'mvsim_tutorial', 'demo_warehouse.world.xml')))

    headless_launch_arg = DeclareLaunchArgument(
        "headless", default_value='False')

    do_fake_localization_arg = DeclareLaunchArgument(
        "do_fake_localization", default_value='True', description='publish tf odom -> base_link')

    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[
            os.path.join(mvsimDir, 'mvsim_tutorial',
                         'mvsim_ros2_params.yaml'),
            {
                "world_file": LaunchConfiguration('world_file'),
                "headless": LaunchConfiguration('headless'),
                "do_fake_localization": LaunchConfiguration('do_fake_localization'),
            }]
    )

    robot_commander_node = Node(
        # package='ros2_ws',  # Change this if the script is in another package
        executable='/home/jlblanco/repos/mvsim_docker_example/robot_commander.py',
        name='robot_commander',
        output='screen'
    )

    return LaunchDescription([
        world_file_launch_arg,
        headless_launch_arg,
        do_fake_localization_arg,
        mvsim_node,
        robot_commander_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=robot_commander_node,
                on_exit=[
                    LogInfo(msg=('robot_commander_node ended')),
                    EmitEvent(event=Shutdown(
                        reason='robot_commander_node ended'))
                ]
            )
        )
    ])
