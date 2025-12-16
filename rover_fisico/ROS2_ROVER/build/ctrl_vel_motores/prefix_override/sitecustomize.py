import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/javier/Rover_Lab/rover_fisico/ROS2_ROVER/install/ctrl_vel_motores'
