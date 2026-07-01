from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    xacro_file = '/home/zrh/agribot/install/agribot_control/share/agribot_control/urdf/agribot.xacro'
    urdf_file = '/home/zrh/agribot/agribot_gazebo/urdf/agribot_raw.urdf'
    preprocessed_world = '/home/zrh/agribot/agribot_gazebo/worlds/farm_world_preprocessed.world'
    preprocess_script = '/home/zrh/agribot/agribot_gazebo/scripts/preprocess_world.py'
    
    return LaunchDescription([
        SetEnvironmentVariable('HOME', '/tmp/home'),
        SetEnvironmentVariable('GAZEBO_HOME', '/tmp/home/.gazebo'),
        SetEnvironmentVariable('ROS_HOME', '/tmp/home/.ros'),
        
        # Step 1: Generate URDF from xacro and replace package:// paths with absolute paths
        ExecuteProcess(
            cmd=['bash', '-c',
                f'source /opt/ros/humble/setup.bash && source /home/zrh/agribot/install/local_setup.bash && '
                f'xacro {xacro_file} > {urdf_file} 2>/dev/null && '
                f'echo "URDF generated"'],
            output='screen'
        ),
        
        # Step 2: Pre-process farm world (replace model:// references with local paths)
        ExecuteProcess(
            cmd=['python3', preprocess_script],
            output='screen'
        ),
        
        # Step 3: Start Gazebo server with farm world        
        ExecuteProcess(
            cmd=['bash', '-c',
                'source /opt/ros/humble/setup.bash && '
                'GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib '
                'GAZEBO_MODEL_PATH=/home/zrh/agribot/install/agribot_gazebo/share/agribot_gazebo/models '
                'ROS_HOME=/tmp/home/.ros '
                'HOME=/tmp/home '
                'gzserver ' + preprocessed_world + ' '
                '-s libgazebo_ros_init.so '
                '-s libgazebo_ros_factory.so '
                '--verbose'],
            output='screen'
        ),
        
        # Step 4: Start Gazebo client (GUI)
        ExecuteProcess(
            cmd=['bash', '-c',
                'source /opt/ros/humble/setup.bash && '
                'GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib '
                'HOME=/tmp/home '
                'gzclient'],
            output='screen'
        ),
        
        # Step 5: Robot state publisher (publishes TF from URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    open(urdf_file).read(),
                    value_type=str
                )
            }]
        ),
        
        # Step 5b: Joint state publisher (publishes default joint positions for TF)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'source_list': ['/joint_states'],
                'use_sim_time': True
            }]
        ),
        
        # Step 6: Wait for world to load, then spawn robot model
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'sleep 8 && source /opt/ros/humble/setup.bash && '
                'GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib '
                'ROS_HOME=/tmp/home/.ros '
                'ros2 run gazebo_ros spawn_entity.py '
                '-file ' + urdf_file + ' '
                '-entity agribot '
                '-x -5.0 -y -2.0 -z 1.13 -Y 0'
            ],
            output='screen'
        ),
    ])