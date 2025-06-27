import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jasdeep_renny/welding_path_ws/install/geodesic_path_planner'
