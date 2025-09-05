from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    model_name = LaunchConfiguration("model_name")
    
    # Package shares
    desc_share = FindPackageShare("so101_description")
    bringup_share = FindPackageShare("so101_bringup")
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')
    so101_ign_share = FindPackageShare('so101_ignition')  # Fixed package name
    
    # Build robot_description from Xacro
    xacro_file = PathJoinSubstitution([desc_share, "urdf", "so101.urdf.xacro"])
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    # World file (fixed extension)
    world_path = PathJoinSubstitution([so101_ign_share, 'worlds', 'empty.sdf'])
    
    # Publish TF + /robot_description
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        output="screen",
    )
    
    # Start Ignition Gazebo
    ign = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', world_path],
        output='screen'
    )
    
    # Spawn robot
    spawn = Node(
        package="ros_ign_gazebo",  # Fixed package name
        executable="create",
        arguments=["-name", model_name, "-topic", "/robot_description", "-z", "0.1"],
        output="screen",
    )
    
    # Bridge for joint states
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen'
    )
    
    # Controller spawners (if controllers are configured)
    controllers_yaml = PathJoinSubstitution([bringup_share, "config", "so101_controllers.yaml"])
    
    spawner_js = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", ["/", model_name, "/controller_manager"],
        ],
        output="screen",
    )
    
    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller", 
            "-c", ["/", model_name, "/controller_manager"],
        ],
        output="screen",
    )
    
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("model_name", default_value="so101"),
        rsp,
        ign,
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=4.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[spawner_js]),
        TimerAction(period=6.0, actions=[spawner_arm]),
    ])