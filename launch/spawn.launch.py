import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory("krabby_description")
    xacro_filepath = os.path.join(pkg, "urdf", "krabby.xacro")

    doc = xacro.process_file(xacro_filepath, mappings={'radius': '0.9'})
    robot_desc = doc.toprettyxml(indent='  ')


    ros_gz_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_spawn_model.launch.py'])]),
        launch_arguments=[('model_string', robot_desc)])



    ld = launch.LaunchDescription()
    ld.add_action(ros_gz_spawn)
    return ld
