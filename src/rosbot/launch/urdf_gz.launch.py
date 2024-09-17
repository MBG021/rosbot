import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import xacro

def generate_launch_description():
    """
    Este script configura y lanza los nodos necesarios para:
    - Publicar el modelo URDF del robot en el tópico `/robot_description`.
    - Iniciar el simulador Gazebo con la configuración requerida.
    - Generar la entidad del robot en Gazebo basándose en la descripción publicada.
     
    Elementos lanzados:
    - Nodo `robot_state_publisher`: Publica el modelo URDF del robot en el tópico `/robot_description`, 
      que es utilizado por otros nodos y simuladores como Gazebo para comprender la estructura del robot.
    - Proceso `gazebo`: Ejecuta el simulador Gazebo con las opciones:
        - `--verbose`: Proporciona una salida detallada en la terminal para el diagnóstico.
        - `-s libgazebo_ros_factory.so`: Carga la extensión de ROS para interactuar con Gazebo, permitiendo 
          la integración entre ROS y Gazebo.
    - Nodo `spawn_entity.py`: Es un script de ROS que genera la entidad del robot en el simulador Gazebo.
        - `-topic /robot_description`: Especifica que la descripción del robot que se debe utilizar está 
          publicada en el tópico `/robot_description`.
        - `-entity "rosbot"`: Define el nombre de la entidad del robot en Gazebo, en este caso, "rosbot".

    Returns:
    LaunchDescription: Una instancia de LaunchDescription con los nodos y procesos configurados.
    """
    pkg_name = 'rosbot'
    file = 'urdf/rosbot.urdf.xacro'

    # Obtiene la ruta completa al archivo .xacro del modelo URDF del robot
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file)
    
    # Procesa el archivo .xacro y lo convierte en una cadena XML
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    return LaunchDescription([
        # Nodo para publicar el modelo URDF en el tópico /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}]
        ),
        
        # Lanzar Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', ],
            output='screen'
        ),
        
        # Nodo para generar la entidad del robot en Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', '/robot_description', '-entity', "rosbot"],
            output='screen'
        ),
    ])