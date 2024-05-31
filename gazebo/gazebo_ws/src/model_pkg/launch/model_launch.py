import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import launch
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Names 
    robotXacroName = 'igvc_robot'
    namePackage = 'model_pkg'
    # Relative Paths
    modelFileRelativePath = 'model/robot.xacro'
    worldFileRelativePath = 'model/green_world.world'
    configFileRelativePath = 'config/ekf.yaml'
    # Absolute Paths
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    pathRvizFile = os.path.join(get_package_share_directory(namePackage),'rviz/rviz_basic_settings.rviz')
    pathConfigFile = os.path.join(get_package_share_directory(namePackage), 'config/ekf.yaml')
    # Processing xacro
    robotDescription = xacro.process_file(pathModelFile).toxml()
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))

    slam_launch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py'))

    # Declare the launch arguments  
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
	    name='urdf_model', 
	    default_value=pathModelFile, 
	    description='Absolute path to robot urdf file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
	    name='rviz_config_file',
	    default_value=pathRvizFile,
	    description='Full path to the RVIZ config file to use')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
	    name='gui',
	    default_value='True',
	    description='Flag to enable joint_state_publisher_gui')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
	    name='use_robot_state_pub',
	    default_value='True',
	    description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
	    name='use_rviz',
	    default_value='True',
	    description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
	    name='use_sim_time',
	    default_value='True',
	    description='Use simulation (Gazebo) clock if true')


    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={'world': pathWorldFile}.items()
    )


    spawnModelNode = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )

    params = {'robot_description': robotDescription, 'use_sim_time': True}

    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

#     robot_localization_node = Node(
#        package='robot_localization',
#        executable='ekf_node',
#        name='ekf_filter_node',
#        output='screen',
#        parameters=[pathConfigFile, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
# )
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[pathConfigFile, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
)

    launchDescriptionObject = LaunchDescription([
        launch.actions.ExecuteProcess(cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 'slam_params_file:=./src/model_pkg/config/mapper_params_online_async.yaml', 'use_sim_time:=true']),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose',"/home/govind/abhiyaan/gazebo_ws/src/model_pkg/model/world.world", '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', "use_sim_time:=true", "headless:=false"], output='screen'),
        # launch.actions.ExecuteProcess(cmd=['ros2','run','teleop_twist_keyboard','teleop_twist_keyboard','cmd_vel:=cmd_vel_joy'])
        ])
    launchDescriptionObject.add_action(declare_urdf_model_path_cmd)
    launchDescriptionObject.add_action(declare_rviz_config_file_cmd)
    launchDescriptionObject.add_action(declare_use_joint_state_publisher_cmd)
    launchDescriptionObject.add_action(declare_use_robot_state_pub_cmd)  
    launchDescriptionObject.add_action(declare_use_rviz_cmd) 
    launchDescriptionObject.add_action(declare_use_sim_time_cmd)
    # launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(joint_state_publisher_node)
    # launchDescriptionObject.add_action(joint_state_publisher_gui_node)
    # launchDescriptionObject.add_action(robot_localization_node)
    launchDescriptionObject.add_action(rviz_node)

    return launchDescriptionObject
