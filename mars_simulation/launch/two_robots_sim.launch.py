from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('mars_robots')
    default_model_path = os.path.join(pkg_share, 'urdf/mars_robot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf.rviz')
    world_path = os.path.join(pkg_share, 'worlds/mars.world')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz')
    
    # Start Gazebo with our world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items(),
    )
    
    # Spawn first robot
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot1',
                  '-file', default_model_path,
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.1',
                  '-R', '0.0',
                  '-P', '0.0',
                  '-Y', '0.0'],
        output='screen'
    )
    
    # Spawn second robot
    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot2',
                  '-file', default_model_path,
                  '-x', '2.0',
                  '-y', '0.0',
                  '-z', '0.1',
                  '-R', '0.0',
                  '-P', '0.0',
                  '-Y', '0.0'],
        output='screen'
    )
    
    # Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Robot state publisher for both robots
    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', default_model_path])
        }],
        remappings=[
            ('/joint_states', '/robot1/joint_states'),
            ('/tf', '/robot1/tf'),
            ('/tf_static', '/robot1/tf_static')
        ]
    )
    
    robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_2',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', default_model_path])
        }],
        remappings=[
            ('/joint_states', '/robot2/joint_states'),
            ('/tf', '/robot2/tf'),
            ('/tf_static', '/robot2/tf_static')
        ]
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_rviz_cmd,
        gazebo,
        spawn_robot1,
        spawn_robot2,
        robot_state_publisher1,
        robot_state_publisher2,
        rviz_node
    ])