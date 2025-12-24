cmake_minimum_required(VERSION 3.8)
project(my_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(webots_ros2_driver REQUIRED)  

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Instala otros recursos si los tienes (URDF, mundos de Webots, etc.)
install(DIRECTORY
  urdf 
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()