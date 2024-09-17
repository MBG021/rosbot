import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    """
    Genera la descripción del lanzamiento para iniciar el robot en RViz y otros nodos relacionados.

    Este script configura y lanza los nodos necesarios para:
    - Publicar el modelo URDF del robot en el tópico `/robot_description`.
    - Lanzar la interfaz gráfica para ajustar los estados de las articulaciones.
    - Iniciar RViz con una configuración predeterminada.

    Elementos clave:
    - `pkg_name`: Obtiene el directorio del paquete `rosbot` para acceder a los archivos necesarios.
    - `xacro_file`: Especifica la ruta al archivo `.xacro` del modelo URDF del robot.
    - `robot_description_raw`: Procesa el archivo `.xacro` y lo convierte en una cadena XML que 
      luego se publica en el tópico `/robot_description` para que otros nodos lo utilicen.
    - `world_file_path`: Define la ruta al archivo de configuración `my_robot_config.rviz` utilizado 
      por RViz. Esta ruta debe ajustarse de acuerdo con la estructura de directorios de tu proyecto.
      En este caso, la ruta se construye utilizando la variable de entorno `HOME` para acceder al 
      directorio principal del usuario, seguida de la estructura de directorios específica:
      `rosbot/src/rosbot/rviz/my_robot_config.rviz`.
    - Nodos lanzados:
        - `robot_state_publisher`: Publica el modelo URDF del robot en el tópico `/robot_description`.
        - `joint_state_publisher_gui`: Inicia la interfaz gráfica para ajustar los estados de las articulaciones.
        - `rviz2`: Inicia RViz con una configuración predeterminada utilizando el archivo RViz especificado en `world_file_path`.

    @note Asegúrate de modificar la ruta del archivo `my_robot_config.rviz` según la ubicación 
    específica en la que esté tu archivo de configuración.

    @returns:
    LaunchDescription: Una instancia de LaunchDescription con los nodos y procesos configurados.
    """
        
    # Ruta al archivo URDF
    pkg_name = get_package_share_directory('rosbot')
    file = 'urdf/rosbot.urdf.xacro'
    
    config_file_path = os.path.join(
            os.getenv('HOME'),
            'rosbot', 'src', 'rosbot', 'rviz', 'my_robot_config.rviz'
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
