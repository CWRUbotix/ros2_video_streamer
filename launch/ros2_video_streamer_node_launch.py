import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    """Asynchronously launches gui node."""
    node_name_arg = DeclareLaunchArgument(
	    'node_name', default_value='ros2_video_streamer_node'
    )
    image_topic_name_arg = DeclareLaunchArgument(
        'image_topic_name', default_value='~/image/compressed'
    )
    info_topic_name_arg = DeclareLaunchArgument(
        'info_topic_name', default_value='~/camera_info'
    )
    config_file_name_arg = DeclareLaunchArgument(
        'config_file_name', default_value='',
        description='Name of the config file. Default is an empty string, which means no config file. Contents published on `CameraInfo` message.'
    )
    loop_arg = DeclareLaunchArgument(
        'loop', default_value='true'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='',
        description='`frame_id` field in the `CameraInfo` topic'
    )
    type_arg = DeclareLaunchArgument(
        'type', description='Type of media source, (e.g. image or video)'
    )
    path_arg = DeclareLaunchArgument(
        'path', default_value='cam1.mp4',
        description='Absolute path to the media source'
    )
    start_arg = DeclareLaunchArgument(
        'start', default_value='0'
    )

    streamer_node: Node = Node(
        package='ros2_video_streamer',
        executable='ros2_video_streamer_node',
        parameters=[
            {'config_file_path': LaunchConfiguration('config_file_name')},
            {'image_topic_name': LaunchConfiguration('image_topic_name')},
            {'info_topic_name': LaunchConfiguration('info_topic_name')},
            {'loop': LaunchConfiguration('loop')},
            {'frame_id': LaunchConfiguration('frame_id')},
            {'type': LaunchConfiguration('type')},
            {'path': LaunchConfiguration('path')},
            {'start': LaunchConfiguration('start')}
        ]
    )

    return launch.LaunchDescription([
        node_name_arg,
        image_topic_name_arg,
        info_topic_name_arg,
        config_file_name_arg,
        loop_arg,
        frame_id_arg,
        type_arg,
        path_arg,
        start_arg,
        streamer_node
    ])
