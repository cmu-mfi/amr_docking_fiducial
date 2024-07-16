from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the docking node'
    )

    namespace = LaunchConfiguration('namespace')


    launch_file_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={'camera_namespace': namespace}.items()
    )

 
    launch_file_aruco = TimerAction(
        period=5.0, 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('aruco_ros'), 'launch', 'docking_markers.launch.py')
                ),
                launch_arguments={'namespace': namespace}.items()
            )
        ]
    )

    docking_node = Node(
        package='docking_with_fiducial',
        executable='docking_with_markers',
        namespace=namespace,
        name='docking_with_markers',
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        launch_file_realsense,
        launch_file_aruco,
        docking_node
    ])
