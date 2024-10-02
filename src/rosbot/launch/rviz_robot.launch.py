import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # Ruta al archivo URDF
    pkg_name = get_package_share_directory('rosbot')
    file = 'urdf/rosbot.urdf.xacro'
    
    config_file_path = os.path.join(
            os.getenv('HOME'),
            'rosbot', 'src', 'rosbot', 'config', 'my_robot_config.rviz'
        )
    # Obtiene la ruta completa al archivo .xacro del modelo URDF del robot
    xacro_file = os.path.join(pkg_name, file)
    
    # Procesa el archivo .xacro y lo convierte en una cadena XML
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Nodo para publicar el modelo URDF en el tópico /robot_description
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 
                     'use_sim_time': True}]
    )
    
    # Nodo para lanzar la interfaz gráfica del estado de las articulaciones
    node_robot_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    
    # Nodo para lanzar RViz con la configuración predeterminada
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config_file_path]
    )
    # Nota: Cambia la ruta anterior según la ubicación de tu archivo de configuración RViz.

    return LaunchDescription([
        node_robot_state_publisher,
        rviz,
        node_robot_state_publisher_gui,
    ])
