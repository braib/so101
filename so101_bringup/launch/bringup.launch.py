from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    desc_pkg = get_package_share_directory('so101_description')
    bringup_pkg = get_package_share_directory('so101_bringup')


    xacro_file = os.path.join(desc_pkg, 'urdf', 'so101.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])

    # urdf_file = os.path.join(pkg_share, 'urdf', 'so101.urdf')
    # with open(urdf_file, 'r') as infp:
    #     robot_description_content = infp.read()

    robot_description = ParameterValue(robot_description_content, value_type=str)



    controllers_yaml = os.path.join(bringup_pkg, 'config', 'so101_controllers.yaml')
    rviz_cfg = os.path.join(bringup_pkg, 'rviz', 'bringup.rviz')


    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, controllers_yaml],
            output='screen'
        ),

        TimerAction(period=1.0, actions=[
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster'], output='screen'),
            Node(package='controller_manager', executable='spawner',
                 arguments=['arm_controller'], output='screen'),
        ]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
            output='screen'
        )
    ])


# ros2 run controller_manager spawner joint_state_broadcaster
# ros2 run controller_manager spawner arm_controller     # if arm
# ros2 run controller_manager spawner base_controller    # if car






# to check
# ros2 control list_hardware_interfaces
# ros2 control list_controllers

# ros2 param get /arm_controller joints










# Spawners usually do load → configure → activate for you:
# ros2 run controller_manager spawner joint_state_broadcaster
# ros2 run controller_manager spawner arm_controller


# # list controllers + states
# ros2 control list_controllers

# # Load (if not in YAML or not pre-loaded)
# ros2 control load_controller arm_controller

# # Configure
# ros2 control set_controller_state arm_controller configure

# # Activate (start running)
# ros2 control set_controller_state arm_controller activate

# # Deactivate / stop
# ros2 control set_controller_state arm_controller deactivate

# # Unload (remove from CM)
# ros2 control unload_controller arm_controller


# # What controllers exist and their states
# ros2 control list_controllers

# # What command/state interfaces are available/claimed
# ros2 control list_hardware_interfaces

# # What controller types are available on your system
# ros2 control list_controller_types

# # What hardware components are loaded
# ros2 control list_hardware_components

# # Get a controller’s parameters (e.g., its joint list)
# ros2 param get /arm_controller joints
