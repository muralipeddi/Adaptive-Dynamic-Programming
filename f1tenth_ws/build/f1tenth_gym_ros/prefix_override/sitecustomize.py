import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/can-02/Desktop/f1tenth_ws/install/f1tenth_gym_ros'
