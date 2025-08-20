import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/danielgrioja/LAB/Rover_Lab/install/rover_ros2'
