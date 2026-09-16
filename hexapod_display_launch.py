import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file_name = "/home/pradnesh/Desktop/MMR/Hexapod/hexapod/urdf/hexapod_with_collisions.urdf"
    rviz_config_path = '/home/pradnesh/Desktop/MMR/hexapod_view.rviz'

    if not os.path.exists(urdf_file_name):
        raise FileNotFoundError(f"Could not find URDF file at: {urdf_file_name}")
        
    with open(urdf_file_name, 'r') as f:
        robot_description_content = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Custom MediaPipe tracking node execution
    mediapipe_node = Node(
        package='your_package_name', # Replace with your ROS2 package name if applicable
        executable='mediapipe_to_ros', # Or run it as a standalone python script
        name='mediapipe_arm_bridge',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        # mediapipe_node, # Alternatively, just run `python3 mediapipe_to_ros.py` in a separate terminal
    ])