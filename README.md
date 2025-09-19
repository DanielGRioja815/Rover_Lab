# Rover Lab

Este repositorio contiene el código y los recursos necesarios para el desarrollo y simulación de un rover. El proyecto está diseñado para explorar conceptos de robótica, control y simulación.

## Contenido
- **rover/**: Carpeta que contiene el proyecto 

## Requisitos Mínimos Recomendados
    Hardware Base (Simulaciones Básicas)
        CPU: Procesador de 4 núcleos (Intel i5 o AMD Ryzen 5 equivalente)
        RAM: 8 GB DDR4
        GPU: Tarjeta gráfica dedicada con 2 GB VRAM (NVIDIA GTX 1050 o superior)
        Almacenamiento: 10 GB de espacio libre (SSD recomendado)

    Hardware Recomendado (Simulaciones Complejas + ROS 2)
        CPU: Procesador de 8 núcleos (Intel i7/i9 o AMD Ryzen 7/9)
        RAM: 16-32 GB DDR4/DDR5
        GPU: Tarjeta gráfica dedicada con 4-8 GB VRAM (NVIDIA RTX 2060 o superior)
        Almacenamiento: SSD NVMe de 500 GB+

    Requisitos de Software
        Sistema Operativo: 
            Ubuntu 22.04 LTS (menos recomendado) o Ubuntu 24.04 LTS (Recomendado)

    Webots 
        Versión 2023b o superior (compatible con ROS 2)
        Instalación desde repositorio oficial

    ROS 2 Humble (Ubuntu 22.04 LTS) o Jazzy (Ubuntu 24.04 LTS)

## Instalación
1. Ubuntu 24.04 LTS: https://releases.ubuntu.com/noble/

2. ROS2 Jazzy: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- Webots(ejecutar en terminal): 
    wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo gpg --dearmor -o /usr/share/keyrings/cyberbotics.gpg
    echo "deb [signed-by=/usr/share/keyrings/cyberbotics.gpg] https://cyberbotics.com/debian/ binary-amd64/" | sudo tee /etc/apt/sources.list.d/cyberbotics.list
    sudo apt update && sudo apt upgrade
    sudo apt install webots
    webots

3. Simulador Webots + ROS2: https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html 

4. rqt_image_view (herramienta de ros2 para ver imagenes que estan siendo publicadas en algun topico): 
    sudo apt update && sudo apt upgrade
    sudo apt install ros-jazzy-rqt-image-view

5. Python y C (Recomendación usar Visual Studio Code): 
    sudo apt update && sudo apt upgrade -y
    sudo apt install software-properties-common apt-transport-https wget -y
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
    sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
    sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
    rm -f packages.microsoft.gpg
    sudo apt update
    sudo apt install code -y

6. Clona este repositorio:
    ```bash
    git clone https://github.com/tu_usuario/Rover_Lab.git
    cd Rover_Lab
    ```
    
## Uso
1. Ejecuta el simulador:
    - cd Rover_Lab/
    - colcon build --symlink-install
    - source install/local_setup.bash
    - ros2 launch rover rover_launch.py

2. Para visualizar la camara: 
    - ros2 run rqt_image_view rqt_image_view
    - Selecciona el topico: camera/image_raw

## Contribuciones

¡Las contribuciones son bienvenidas! Por favor, abre un issue o envía un pull request con tus mejoras.
Para ello se les recomienda saber lo basico de ROS2 (que son los topicos, nodos, workspace), saber usar el simulador webots (sin ROS2) y saber usarlo con ROS2. 

##  Links tutoriales: 

ROS 2 (tutoriales basicos): https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html 
Webots: https://cyberbotics.com/doc/guide/tutorials 
Webots + ROS2: https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Simulation-Webots.html

## Licencia

Este proyecto está bajo la licencia MIT. Consulta el archivo `LICENSE` para más detalles.