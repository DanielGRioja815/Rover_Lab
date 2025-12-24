import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/danielgrioja/Proyectos/Rover_Lab/ros2_ws/install/my_package'
