"""Launch the full drop-off pipeline (detector + controller).

Both nodes are started together but the controller is harmless on its
own -- it stays in `idle` and publishes zero commands until the
operator publishes a real mode on `/drop_off/mode`. Detection runs
unconditionally so the operator can verify the overlay in
`rqt_image_view /drop_off/image_overlay` while driving manually.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('drop_off')
    detector_yaml = os.path.join(pkg_share, 'config', 'line_detector.yaml')
    follower_yaml = os.path.join(pkg_share, 'config', 'line_follower.yaml')
    guard_yaml = os.path.join(pkg_share, 'config', 'aruco_distance_guard.yaml')

    detector_params_arg = DeclareLaunchArgument(
        'detector_params',
        default_value=detector_yaml,
        description='Path to the line_detector parameter YAML.',
    )
    follower_params_arg = DeclareLaunchArgument(
        'follower_params',
        default_value=follower_yaml,
        description='Path to the line_follower_controller parameter YAML.',
    )
    guard_params_arg = DeclareLaunchArgument(
        'guard_params',
        default_value=guard_yaml,
        description='Path to the aruco_distance_guard parameter YAML.',
    )

    detector = Node(
        package='drop_off',
        executable='line_detector',
        name='line_detector',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('detector_params')],
    )

    controller = Node(
        package='drop_off',
        executable='line_follower_controller',
        name='line_follower_controller',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('follower_params')],
    )

    guard = Node(
        package='drop_off',
        executable='aruco_distance_guard',
        name='aruco_distance_guard',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('guard_params')],
    )

    return LaunchDescription([
        detector_params_arg,
        follower_params_arg,
        guard_params_arg,
        detector,
        controller,
        guard,
    ])
