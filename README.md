# Rover Lab

Este repositorio contiene el código y los recursos necesarios para el desarrollo y simulación de un rover. El proyecto está diseñado para explorar conceptos de robótica, control y simulación.

## Contenido

- **my_first_simulation/**: Archivos realizados en el tutorial del simulador de Webots
- **my_package/**: Archivos realizados en el tutorial del simulador de Webots + ROS2 
- **rover/**: Carpeta que contiene el proyecto 

## Requisitos
- Ubuntu 22.04 LTS: https://releases.ubuntu.com/jammy/ 
- ROS2 Humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html 
- Simulador Webots + ROS2: https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Simulation-Webots.html 
- Python y C
- rqt_image_view (herramienta de ros2 para ver imagenes que estan siendo publicadas en algun topico): sudo apt install ros-humble-rqt-image-view


## Instalación

1. Clona este repositorio:
    ```bash
    git clone https://github.com/tu_usuario/Rover_Lab.git
    cd Rover_Lab
    ```
2. Instala Webots
    

## Uso

1. Ejecuta el simulador:
    - cd Rover_Lab/
    - colcon build --symlink-install
    - source install/local_setup.bash
    - ros2 launch rover rover_launch.py
2. Para visualizar la camara: 
    - rqt_image_view
    - Seleccionar el topico camera/image_raw

## Contribuciones

¡Las contribuciones son bienvenidas! Por favor, abre un issue o envía un pull request con tus mejoras.
Para ello se les recomienda saber lo basico de ROS2 (que son los topicos, nodos, workspace), saber usar el simulador webots (sin ROS2) y saber usarlo con ROS2. 

##  Links tutoriales: 

ROS 2 (tutoriales basicos): https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html 
Webots: https://cyberbotics.com/doc/guide/tutorials 
Webots + ROS2: https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Simulation-Webots.html 

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo `LICENSE` para más detalles.