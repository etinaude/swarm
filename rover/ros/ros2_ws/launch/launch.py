from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

urdf_file = "/home/etienne/Documents/git/swarm/rover/urdf/robot.urdf"


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='usb_cam',
        #     executable='usb_cam_node_exe',
        #     name='usb_cam_node',
        #     parameters=[{
        #         'video_device': '/dev/video2',
        #         'image_width': 640,
        #         'image_height': 480,
        #         'camera_frame_id': 'camera',
        #         'io_method': 'mmap',
        #     }],
        #     output='screen'
        # ),
        Node(
            package='sub',
            executable='keyboard_sub',
            name='keyboard_subscriber',
            output='screen'
        ),
        # Node(
        #     package='sub',
        #     executable='image_sub',
        #     name='image_subscriber',
        #     output='screen'
        # ),

        # teleop_twist_keyboard
               Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='konsole -e'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory("sub"), 'rviz', 'your_config.rviz')]
        )
    ])
# ros2 run  teleop_twist_keyboard