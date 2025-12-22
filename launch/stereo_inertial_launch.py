#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch ORB-SLAM3 ROS2 stereo-inertial node with configurable topics."""
    
    # Declare launch arguments
    vocabulary_path_arg = DeclareLaunchArgument(
        'vocabulary_path',
        description='Path to ORB-SLAM3 vocabulary file'
    )

    # Get config path
    zedx_config_path = os.path.join(
        get_package_share_directory('orbslam3_ros2'),
        'config',
        'stereo-inertial',
        'ZedX.yaml',
    )
    
    settings_path_arg = DeclareLaunchArgument(
        'settings_path',
        default_value=zedx_config_path,
        description='Path to ORB-SLAM3 settings file'
    )
    
    do_rectify_arg = DeclareLaunchArgument(
        'do_rectify',
        default_value='false',
        description='Enable stereo image rectification'
    )
    
    do_equalize_arg = DeclareLaunchArgument(
        'do_equalize',
        default_value='false',
        description='Enable CLAHE histogram equalization'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/place_detector_cam/zed_node/imu/data',
        description='IMU topic name'
    )
    
    left_image_topic_arg = DeclareLaunchArgument(
        'left_image_topic',
        default_value='/place_detector_cam/zed_node/left/image_rect_color',
        description='Left camera image topic'
    )
    
    right_image_topic_arg = DeclareLaunchArgument(
        'right_image_topic',
        default_value='/place_detector_cam/zed_node/right/image_rect_color',
        description='Right camera image topic'
    )
    
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/orb3/pose',
        description='Output pose topic'
    )
    
    # Create ORB-SLAM3 node
    orb_slam3_node = Node(
        package='orbslam3_ros2',
        executable='stereo-inertial',
        name='orbslam3_stereo_inertial',
        output='screen',
        arguments=[
            LaunchConfiguration('vocabulary_path'),
            LaunchConfiguration('settings_path'),
            LaunchConfiguration('do_rectify'),
            LaunchConfiguration('do_equalize')
        ],
        parameters=[{
            'imu_topic': LaunchConfiguration('imu_topic'),
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'pose_topic': LaunchConfiguration('pose_topic')
        }],
        remappings=[
            ('/place_detector_cam/zed_node/imu/data', LaunchConfiguration('imu_topic')),
            ('/place_detector_cam/zed_node/left/image_rect_color', LaunchConfiguration('left_image_topic')),
            ('/place_detector_cam/zed_node/right/image_rect_color', LaunchConfiguration('right_image_topic')),
            ('/orb3/pose', LaunchConfiguration('pose_topic'))
        ]
    )
    
    return LaunchDescription([
        vocabulary_path_arg,
        settings_path_arg,
        do_rectify_arg,
        do_equalize_arg,
        imu_topic_arg,
        left_image_topic_arg,
        right_image_topic_arg,
        pose_topic_arg,
        orb_slam3_node
    ])
