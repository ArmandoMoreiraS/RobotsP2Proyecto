import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/armando30/robotsprj/install/ur5e_conveyor_bringup'
