from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    model_name   = LaunchConfiguration("model_name")
    use_rviz     = LaunchConfiguration("use_rviz")
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_ignition = LaunchConfiguration('use_ignition')



    desc_share   = FindPackageShare("so101_description")
    ign_share    = FindPackageShare("ros_ign_gazebo")
    bringup_share= FindPackageShare("so101_bringup")
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')
    so101_ign_share  = FindPackageShare('so101_ign')
    
    # Build robot_description from Xacro
    xacro_file  = PathJoinSubstitution([desc_share, "urdf", "so101.urdf.xacro"])
    # robot_description_content = Command(['xacro ', xacro_file])
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' use_ignition:=', use_ignition
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Known-good world shipped with ros_ign_gazebo
    gz_launch  = PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
    world_path = PathJoinSubstitution([so101_ign_share, 'worlds', 'empty.sdf'])
    rviz_cfg = PathJoinSubstitution([bringup_share, 'rviz', 'bringup.rviz'])



    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time
        }],
        output="screen",
    )

    ign = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r -v 4 '), world_path]
        }.items()
    )

    spawn = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-name", model_name,
            "-topic", "/robot_description",
            "-z", "0.015"
        ],
        output="screen",
    )

    controllers_yaml = PathJoinSubstitution([bringup_share, "config", "so101_controllers1.yaml"])


    spawner_js = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-type", "joint_state_broadcaster/JointStateBroadcaster",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-type", "joint_trajectory_controller/JointTrajectoryController",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    spawner_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-type", "position_controllers/JointGroupPositionController",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument("model_name",   default_value="so101"),
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        DeclareLaunchArgument('use_ignition', default_value='true'),
        rsp,
        ign,
        TimerAction(period=1.0, actions=[spawn]),
        TimerAction(period=3.0, actions=[spawner_js]),
        TimerAction(period=4.0, actions=[spawner_arm]),
        TimerAction(period=5.0, actions=[spawner_gripper]),
        TimerAction(period=6.0, actions=[rviz], condition=IfCondition(use_rviz)),
    ])
