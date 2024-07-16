from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    
    namespace = LaunchConfiguration('namespace').perform(context)

    aruco_marker_publisher_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': 'camera_color_frame',
    }

    aruco_marker_publisher = Node(
        package='aruco_ros',
        executable='marker_publisher',
        parameters=[aruco_marker_publisher_params],
        remappings=[
            ('/camera_info', f'{namespace}/camera/color/camera_info'),
            ('/image', f'{namespace}/camera/color/image_raw'),
        ],
    )

    return [aruco_marker_publisher]


def generate_launch_description():
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.165',
        description='Marker size in meters.'
    )

    reference_frame = DeclareLaunchArgument(
        'reference_frame',
        default_value='camera_color_frame',
        description='Reference frame. '
                    'Leave it empty and the pose will be published with respect to param parent_name.'
    )

    # namespace_arg = DeclareLaunchArgument(
    #     'namespace',
    #     default_value='',
    #     description='Namespace for aruco-ros'
    # )

    # Create the launch description and populate
    ld = LaunchDescription()

    # ld.add_action(namespace_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(reference_frame)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
