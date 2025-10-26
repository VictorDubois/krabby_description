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


def get_robot_description_string(pkg):
    xacro_filepath = os.path.join(pkg, "urdf", "krabby.xacro")

    doc = xacro.process_file(xacro_filepath, mappings={'radius': '0.9'})
    return doc.toprettyxml(indent='  ')


def generate_launch_description():
    pkg = get_package_share_directory("krabi_description")

    isBlue_value = LaunchConfiguration('isBlue')
    xRobotPos_value = LaunchConfiguration('xRobotPos')
    yRobotPos_value = LaunchConfiguration('yRobotPos')
    zRobotOrientation_value = LaunchConfiguration('zRobotOrientation')
  
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

    robot_desc = get_robot_description_string(pkg)
    bridge_config_file = os.path.join(pkg, "params", "gz_bridge.yaml")

    # Set gazebo environment variables
    pkg_share_dir = pkg
    models_path = pkg_share_dir + "/models"
    worlds_path = pkg_share_dir + "/worlds"
    os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ":" + worlds_path

    ros_gz_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'ros_gz_spawn_model.launch.py'])]),
        launch_arguments=[
            ('bridge_name', 'gz_bridge'),
            ('model_string', robot_desc),
            ('x', xRobotPos_value),
            ('y', yRobotPos_value),
            #('z', "0.5"),
            ("Y", zRobotOrientation_value),
            ('config_file', bridge_config_file),
        ])


    krabi_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, "robot_description": robot_desc}]
    )

    ld = launch.LaunchDescription([krabi_state_pub_node])
    ld.add_action(ros_gz_spawn)
    return ld
