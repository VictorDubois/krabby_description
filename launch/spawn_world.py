import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import ExecuteProcess



def generate_launch_description():
    pkg = get_package_share_directory("krabby_description")

    

    isBlue_value = LaunchConfiguration('isBlue')
    xRobotPos_value = LaunchConfiguration('xRobotPos')
    yRobotPos_value = LaunchConfiguration('yRobotPos')
    zRobotOrientation_value = LaunchConfiguration('zRobotOrientation')
    world_value = LaunchConfiguration('world')
  
    isBlue_launch_arg = DeclareLaunchArgument(
        'isBlue',
        default_value='False'
    )
    xRobotPos_launch_arg = DeclareLaunchArgument(
        'xRobotPos',
        default_value='1.25'
    )
    yRobotPos_launch_arg = DeclareLaunchArgument(
        'yRobotPos',
        default_value='0.5'
    )
    zRobotOrientation_launch_arg = DeclareLaunchArgument(
        'zRobotOrientation',
        default_value='0.0'
    )
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='table2026.world'
    )

    # Set gazebo environment variables
    pkg_share_dir = get_package_share_directory('krabi_description')
    models_path = pkg_share_dir + "/models"
    worlds_path = pkg_share_dir + "/worlds"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path

    gz_proc = ExecuteProcess(cmd=['gz', 'sim', world_value, '-v', '-r'], output='screen')


    ld = launch.LaunchDescription([world_launch_arg])
    ld.add_action(gz_proc)
    return ld
