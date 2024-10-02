# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
import xacro
# Libraries for node launching
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Import package's filepath
pkg_filepath = get_package_share_directory("rosbot")

# Process xacro file
xacro_filepath = os.path.join(pkg_filepath, "urdf", "rosbot.urdf.xacro")
robot_description_file = xacro.process_file(xacro_filepath).toxml()

# Define world file path
world_file_name = 'emptyfarm.world'  # Cambia esto por el nombre de tu archivo de mundo
world_file_path = os.path.join(pkg_filepath, "worlds", world_file_name)  # Asegúrate de que esta ruta sea correcta


# Declare launch arguments
use_sim_time = LaunchConfiguration("use_sim_time")
use_ros2_control = LaunchConfiguration('use_ros2_control')

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", 
        default_value= "true", 
        description= "Use sim time if true"
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control if true'
    )

    robot_state_publisher_node = Node(
        package= "robot_state_publisher",
        executable= "robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_file,
                "use_sim_time": use_sim_time
            }
        ]
    )
    
    gazebo_cmd = ExecuteProcess(
        cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so", world_file_path],
        output="screen"
    )
    
    gazebo_spawner_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-topic', 'robot_description', '-entity', 'rosbot'],
        output="screen"
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        remappings=[
            ("~/robot_description", "robot_description")
		]
    )
    jsb_spawner_node = Node(
		package="controller_manager",
		executable="spawner",
		arguments= ["joint_state_broadcaster"]
	)
    jgpc_spawner_node = Node(
		package="controller_manager",
		executable="spawner",
		arguments= ["tricycle_controller"]
	)

    
    nodes_to_run = [
        use_sim_time_arg,
        use_ros2_control_arg,
        robot_state_publisher_node,
        gazebo_cmd, 
        gazebo_spawner_node,
        ros2_control_node,
        jsb_spawner_node,
        jgpc_spawner_node
    ]
    return LaunchDescription(nodes_to_run)
