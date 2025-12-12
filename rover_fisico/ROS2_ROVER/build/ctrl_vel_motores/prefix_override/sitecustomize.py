import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/javier/ROVER_V1/install/ctrl_vel_motores'
