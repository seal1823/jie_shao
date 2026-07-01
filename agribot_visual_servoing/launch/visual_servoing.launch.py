import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agribot_visual_servoing',
            executable='crop_row_detector_node',
            name='agribot_vs',
            output='screen',
            parameters=[{
                # Camera topic (from Gazebo simulation)
                'camera_topic': '/front_camera/image_raw',
                'back_camera_topic': '/back_camera/image_raw',
                # Pose/odometry topic
                'pose_topic': '/odom',
                # Control parameters
                'linear_velocity': 0.3,
                'angular_gain': 0.3,
                'lateral_gain': 0.3,
                'max_angular_vel': 0.8,
                'min_angular_vel': 0.05,
                'heading_gain': 1.0,
                # Turn parameters
                'row_end_x': 12.5,
                'row_start_x': -7.0,
                # HSV thresholds for green vegetation detection
                'green_h_low': 25,
                'green_s_low': 30,
                'green_v_low': 30,
                'green_h_high': 95,
                'green_s_high': 255,
                'green_v_high': 255,
                # Aisle / snake-path parameters
                # Actual crop rows are at y=-3,-1,1,3; drivable aisles are at y=-2,0,2
                'aisle_y_positions': [-2.0, 0.0, 2.0],
                'u_path_drive_speed': 0.3,
                'u_path_turn_speed': 0.5,
                'u_path_turn_tolerance': 0.08,
                'u_path_y_tolerance': 0.3,
                # Centerline stability parameters
                'min_lane_width_px': 20.0,
                'max_lane_width_px': 320.0,
                'max_center_jitter': 0.30,
                'min_confidence': 0.0,
                'control_rate_hz': 20.0,
            }],
            remappings=[
                # Remap cmd_vel to the robot's actual cmd_vel topic if needed
                # ('/cmd_vel', '/agribot/cmd_vel'),
            ],
        ),
        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('agribot_visual_servoing'),
                'rviz', 'visual_servoing.rviz')],
        ),
    ])