import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def generate_launch_description():
    """Launch streamer node with description from launch_setup."""
    # Using OpaqueFunction lets us access the launch context to evaluate
    # params early
    return launch.LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


def launch_setup(context, *args, **kwargs):
    """Generate array to be included in launch description."""
    # Declare and early evaluate camera_name argument
    # Only strictly necessary to set streamer node name & namespace,
    # but makes other substitutions nicer too
    camera_name_argument: DeclareLaunchArgument = DeclareLaunchArgument(
            'camera_name', default_value='simulated_cam',
            description='Name of the camera, used to autopopulate' +
            '`image_topic_name`, `info_topic_name`, & `path`. Will find video' +
            'files named `<camera_name>.mp4`.')
    camera_name_str = LaunchConfiguration('camera_name').perform(context)

    launch_arguments: list[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            'node_name', default_value='streamer_node'
        ),
        DeclareLaunchArgument(
            'image_topic_name',
            default_value=f'/{camera_name_str}/image_raw'
        ),
        DeclareLaunchArgument(
            'info_topic_name',
            default_value=f'/{camera_name_str}/camera_info'
        ),
        DeclareLaunchArgument(
            'config_file_name', default_value='',
            description='Name of the config file. Defaults to empty string, which' +
            'means no config file. Contents published on `CameraInfo` message.'
        ),
        DeclareLaunchArgument(
            'loop', default_value='true'
        ),
        DeclareLaunchArgument(
            'frame_id', default_value='',
            description='`frame_id` field in the `CameraInfo` topic'
        ),
        DeclareLaunchArgument(
            'type', description='Type of media source, (e.g. image or video)'
        ),
        DeclareLaunchArgument(
            'path', default_value=f'{camera_name_str}.mp4',
            description='Absolute path to the media source'
        ),
        DeclareLaunchArgument(
            'start', default_value='0'
        )
    ]

    streamer_node: Node = Node(
        package='ros2_video_streamer',
        executable='ros2_video_streamer_node',
        name='streamer_node',
        namespace=f'{camera_name_str}',
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

    return [
        camera_name_argument,
        *launch_arguments,
        streamer_node
    ]
