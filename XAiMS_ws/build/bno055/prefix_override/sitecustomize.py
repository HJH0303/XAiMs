import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aims_xavier/XAiMs/XAiMS_ws/install/bno055'
