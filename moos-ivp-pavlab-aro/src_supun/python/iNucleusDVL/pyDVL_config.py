#!/usr/bin/env python

config_dict = {
    "conn_Serial": True,    # Boolean: True -> use serial, False -> use TCP
    "port": "/dev/ttyUSB0", # String: Specify Serial port. 
    "use_hostname": True,  # Boolean: use hostname (with serial) vs. IP 
    "serialnum": "300085",  # String: serial num for hostname (above)
    "IP": "169.254.15.123", # String: IP Address alt. used for TCP 
    "logging": False,       # Boolean: True -> enable logging.
}

# port documentation from Nortek is WRONG. Must have leading / for /dev...
# serial num can be found on device
