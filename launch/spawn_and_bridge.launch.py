import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from ament_index_python.packages import get_package_share_directory
import os

def get_robot_description_string(pkg):
    xacro_filepath = os.path.join(pkg, "urdf", "krabby.xacro")

    doc = xacro.process_file(xacro_filepath, mappings={'radius': '0.9'})
    return doc.toprettyxml(indent='  ')


def generate_launch_description():
    pkg = get_package_share_directory("krabby_description")

    robot_desc = get_robot_description_string(pkg)
    bridge_config_file = os.path.join(pkg, "params", "gz_bridge.yaml")

    ros_gz_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'ros_gz_spawn_model.launch.py'])]),
        launch_arguments=[
            ('bridge_name', 'gz_bridge'),
            ('model_string', robot_desc),
            # ('x', 1.23), # can be used to define the full pose (xyzRPY)
            ('config_file', bridge_config_file),
        ])

    # arams/gz_bridge.yaml"/>


    ld = launch.LaunchDescription()
    ld.add_action(ros_gz_spawn)
    return ld
