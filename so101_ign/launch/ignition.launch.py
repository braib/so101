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
    model_name   = LaunchConfiguration("model_name")

    desc_share   = FindPackageShare("so101_description")
    ign_share    = FindPackageShare("ros_ign_gazebo")
    bringup_share= FindPackageShare("so101_bringup")
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')
    so101_ign_share  = FindPackageShare('so101_ign')
    
    # Build robot_description from Xacro
    xacro_file  = PathJoinSubstitution([desc_share, "urdf", "so101.urdf.xacro"])
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Known-good world shipped with ros_ign_gazebo
    gz_launch  = PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
    world_path = PathJoinSubstitution([so101_ign_share, 'worlds', 'empty.world'])




    # bcr_arm_gazebo_models_dir = os.path.join(pkg_share_gazebo, gazebo_models_path_name)
    # bcr_arm_description_share_dir_parent = os.path.abspath(os.path.join(pkg_share_description, '..'))

    # set_gz_resource_path_for_gazebo_pkg = AppendEnvironmentVariable(
    #     'GZ_SIM_RESOURCE_PATH',
    #     bcr_arm_gazebo_models_dir
    # )
    # set_gz_resource_path_for_description_pkg = AppendEnvironmentVariable(
    #     'GZ_SIM_RESOURCE_PATH',
    #     bcr_arm_description_share_dir_parent
    # )




    # Publish TF + /robot_description (needed for RViz and for 'create' to pull from topic)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        output="screen",
    )

    # Start Ignition Fortress as a process
    ign = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            # IMPORTANT: pass one concatenated string via a list of substitutions
            'gz_args': [TextSubstitution(text='-r -v 4 '), world_path]
        }.items()
    )

    # Spawn robot from /robot_description into Ignition
    spawn = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=["-name", model_name, "-topic", "/robot_description", "-z", "0.05"],
        output="screen",
    )

    controllers_yaml = PathJoinSubstitution([bringup_share, "config", "so101_controllers.yaml"])

    # NOTE: the controller manager is usually under /<model_name>/controller_manager with ign_ros2_control
    spawner_js = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-type", "joint_state_broadcaster/JointStateBroadcaster",
            "-c", "/so101/controller_manager",
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
            "-c", "/so101/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("model_name",   default_value="so101"),

        rsp,
        ign,
        TimerAction(period=1.0, actions=[spawn]),
        TimerAction(period=2.0, actions=[spawner_js]),
        TimerAction(period=3.0, actions=[spawner_arm]),
    ])
