# rosbot
# Descripción del Proyecto

Este repositorio alberga el desarrollo de un rover basado en la plataforma ROSbot Plus. 
Aquí se implementarán y probarán diversos algoritmos para navegación autónoma, teleoperación y percepción.

# Clonación del Repositorio

$ git clone https://github.com/MBG021/rosbot

# Incluir en la carpeta oculta de .gazebo una carpeta llamada rosbot, dentro de la cual incluiremos nuestra carpeta meshes y el archivo model.config

# Instalación de Dependencias

Actualiza el sistema e instala las dependencias necesarias:

$ sudo apt update

$ sudo apt upgrade

$ sudo apt install ros-humble-joint-state-publisher*

$ sudo apt install ros-humble-xacro

$ sudo apt install ros-humble-gazebo-*

$ sudo apt install ros-humble-ackermann-*

$ sudo apt install ros-humble-rosgraph*

$ sudo apt install ros-humble-control*

$ sudo apt install ros-humble-ros2*

$ pip install modern_robotics

# Configuración del Entorno

Agrega la siguiente línea a tu archivo .bashrc para asegurarte de que el entorno ROS esté correctamente configurado:

source /usr/share/gazebo/setup.bash

source /usr/share/gazebo-11/setup.bash

echo $ROS_PACKAGE_PATH

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ruta/a la carpeta/models/en tu work space/

# Compilación del Proyecto

Navega al directorio del proyecto y compila con colcon:

$ cd rosbot

$ colcon build

$ source install/setup.bash

# Visualización en RViz

Para visualizar el rover en RViz, ejecuta el siguiente comando:

$ ros2 launch rosbot rviz_robot.launch.py

# Simulación en Gazebo

Para simular el rover en Gazebo, usa el siguiente comando:

$ ros2 launch rosbot urdf_gz.launch.py

# Teleoperacion en gazebo

ventana 1: ros2 launch rosbot gz_controller.launch.py

ventana 2: ros2 run rosbot ack_controller.py 
