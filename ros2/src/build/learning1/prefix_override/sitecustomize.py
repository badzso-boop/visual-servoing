import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/g/University/5. Semester/Projektmunka/visual_servoing_ws/ros2/src/install/learning1'
