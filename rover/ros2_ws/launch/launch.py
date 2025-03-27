from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            parameters=[{
                'video_device': '/dev/video2',
                'image_width': 640,
                'image_height': 480,
                'camera_frame_id': 'camera',
                'io_method': 'mmap',
            }],
            output='screen'
        ),
        Node(
            package='sub',
            executable='keyboard_sub',
            name='keyboard_subscriber',
            output='screen'
        ),
        Node(
            package='sub',
            executable='image_sub',
            name='image_subscriber',
            output='screen'
        )
    ])
