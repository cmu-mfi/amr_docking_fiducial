from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription(

        launch_file_aruco = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('aruco_ros'), 'launch/docking_markers.launch.py')
            )
        )

        launch_file_realsense = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense2_camera'), 'launch/rs_launch.py')
            )
        )

        docking_node = Node(
            package = 'docking_with_fiducial'
            executable = 'docking_with_markers'
        )


    )

