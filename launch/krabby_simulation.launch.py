# Author: Addison Sears-Collins
# Date: September 19, 2021
# Description: Load a world file into Gazebo.
# https://automaticaddison.com
 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import xacro
 
def generate_launch_description():
 
  # Set the path to the Gazebo ROS package
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
   
  # Set the path to this package.
  pkg_share = FindPackageShare(package='eurobot2020_gazebo').find('eurobot2020_gazebo')
 
  # Set the path to the world file
  world_file_name = 'eurobot2023_basic.world'
  world_path = os.path.join(pkg_share, 'worlds', world_file_name)
   
  # Set the path to the SDF model files.
  gazebo_models_path = os.path.join(pkg_share, 'models')
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

  krabby_simulation_pkg = FindPackageShare(package='krabby_description').find('krabby_description')
  krabby_simulation_urdf = os.path.join(krabby_simulation_pkg, 'urdf')
  krabi_xacro_file_name =  os.path.join(krabby_simulation_urdf, 'krabby.xacro')
  print("xacro_file_name : {}".format(krabi_xacro_file_name))


  doc = xacro.process_file(krabi_xacro_file_name, mappings={})#"/cmd_vel": "/krabby/cmd_vel"})
  robot_desc = doc.toprettyxml(indent='  ')
  rsp_params = {'robot_description': robot_desc}
  rsp = Node(package='robot_state_publisher',
              executable='robot_state_publisher',
              output='both',
              #namespace="krabby",
              namespace="krabi_ns",
              parameters=[rsp_params])
  isBlue_value = LaunchConfiguration('isBlue')
  xRobotPos_value = LaunchConfiguration('xRobotPos')
  yRobotPos_value = LaunchConfiguration('yRobotPos')
  zRobotOrientation_value = LaunchConfiguration('zRobotOrientation')
  
  isBlue_launch_arg = DeclareLaunchArgument(
    'isBlue',
    default_value='false'
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

  

  krabby_spawn = Node(package='gazebo_ros',
              executable='spawn_entity.py',
              output='both',
              #namespace="krabi_ns",
              namespace="krabi_ns",
              arguments=['-topic', 'robot_description',
                                   '-entity', 'krabby', 
                                   "-x", xRobotPos_value, "-y", yRobotPos_value, '-z', '1.0', "-Y", zRobotOrientation_value ]
              #parameters=[krabby_spawn_params]#"-urdf -param robot_description -entity krabby -x $(arg xRobotPos) -y $(arg yRobotPos) -z 1.0 -Y $(arg zRobotOrientation)"])
  )
  

  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  #headless = LaunchConfiguration('headless')
  headless = "True"
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
    
  # Specify the actions
   
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
  
  krabby_state_pub_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=[krabi_xacro_file_name]
  )

  lidar_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
    output='screen'
  )
 
  # Create the launch description and populate
  ld = LaunchDescription([isBlue_launch_arg, xRobotPos_launch_arg, yRobotPos_launch_arg, zRobotOrientation_launch_arg, lidar_bridge])

  # Declare the launch options
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
  #ld.add_action(krabby_state_pub_node)
  ld.add_action(rsp)
  ld.add_action(krabby_spawn)
 
  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  #ld.add_action(PushRosNamespace("krabby"))
 
  return ld