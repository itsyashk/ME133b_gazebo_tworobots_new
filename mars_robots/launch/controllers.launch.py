from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot1',
            description='Name of the robot'
        )
    )
    
    # Initialize Arguments
    robot_name = LaunchConfiguration('robot_name')
    
    # Get package share directory
    pkg_share = FindPackageShare('mars_robots').find('mars_robots')
    
    # Get controller parameters
    controller_params_file = PathJoinSubstitution(
        [pkg_share, 'config', 'controllers.yaml'])

    # Setup controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params_file],
        namespace=robot_name,
        output='screen',
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', [robot_name, '/controller_manager']],
        namespace=robot_name,
        output='screen',
    )

    # Differential drive controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', [robot_name, '/controller_manager']],
        namespace=robot_name,
        output='screen',
    )

    # Create and return launch description
    return LaunchDescription(
        declared_arguments + [
            controller_manager,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]
    )