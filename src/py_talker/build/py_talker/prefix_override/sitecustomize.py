import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bilbo/ros2_ws/src/py_talker/install/py_talker'
