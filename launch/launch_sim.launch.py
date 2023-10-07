import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.actions import Node
import xacro
import random
import math

def generate_random_pose():
    # Generate random x, y, z coordinates within a specific range
    x = random.uniform(-0.15, 0.2)  # Adjust the range as needed
    y = random.uniform(-0.15, 0.2)  # Adjust the range as needed
    z = 0.0
    # Generate a random yaw angle (rotation around the z-axis) in radians
    yaw = random.uniform(0, math.pi)

    return x, y, z, yaw

def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'reactivebot_one'
    # Set the path to the world file
    world_file_name = 'question_mark_wall.world'

    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    world_path = os.path.join(pkg_path, 'worlds', world_file_name)
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')

    robot_description_raw = xacro.process_file(xacro_file).toxml()

    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient'
    )
     
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator'
    )
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load'
    )

    # Specify the actions
    # Start Gazebo server
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
                launch_arguments={'world': world}.items()
            )
    
     # Configure the node
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    x, y, z, yaw = generate_random_pose()

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description', 
                                '-entity', 'my_bot', '-x', {str(x)}, '-y', {str(y)}, '-z', {str(z)}, '-Y', {str(yaw)}],
                    output='screen')

    # Create the launch description and populate
    launchDescription = LaunchDescription()
    
    # Declare the launch options
    launchDescription.add_action(declare_simulator_cmd)
    launchDescription.add_action(declare_use_sim_time_cmd)
    launchDescription.add_action(declare_use_simulator_cmd)
    launchDescription.add_action(declare_world_cmd)
    
    # Add any actions
    launchDescription.add_action(spawn_entity)
    launchDescription.add_action(node_robot_state_publisher)
    launchDescription.add_action(gazebo)
    
    return launchDescription
