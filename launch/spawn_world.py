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
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    pkg = get_package_share_directory("krabi_description")

    

    isBlue_value = LaunchConfiguration('isBlue')
    xRobotPos_value = LaunchConfiguration('xRobotPos')
    yRobotPos_value = LaunchConfiguration('yRobotPos')
    zRobotOrientation_value = LaunchConfiguration('zRobotOrientation')
    world_value = LaunchConfiguration('world')
    gui_value = LaunchConfiguration('gui')
  
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

    gui_launch_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run with GUI (true) or headless (false)'
    )

    # Set gazebo environment variables
    pkg_share_dir = get_package_share_directory('krabi_description')
    models_path = pkg_share_dir + "/models"
    worlds_path = pkg_share_dir + "/worlds"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path

    # GUI version
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', world_value, '-v', '-r'],
        output='screen',
        condition=IfCondition(gui_value)
    )

    # Headless version
    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', world_value, '-v', '-r', '-s', '--headless-rendering'],
        output='screen',
        condition=UnlessCondition(gui_value)
    )

    ld = launch.LaunchDescription([world_launch_arg, gui_launch_arg])
    ld.add_action(gz_gui)
    ld.add_action(gz_headless)
    return ld
