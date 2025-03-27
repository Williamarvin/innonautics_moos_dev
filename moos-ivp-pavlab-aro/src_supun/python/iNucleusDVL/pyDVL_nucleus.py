#!/usr/bin/env python

# Nucleus DVL Driver - Mark Franklin - 7/20/23
# https://github.com/nortekgroup/nucleus_driver/tree/main/nucleus_driver

from nucleus_driver import NucleusDriver
import pymoos
import time
import atexit # used to end measurement etc when program quits

import pyDVL_config as cfg # config file with dict of user-spec

class pynucleus_driver(object):
    def __init__(self):
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
        ###parameters###
        self.moos_app_name = 'pynucleus_driver'
        self.time_warp = 1
        self.server_host = 'localhost'
        self.server_port = 9022 # for oak, since config setting is not setup yet
        
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

        self.moos_connected = False
        self.CMD = "" # init. var used for receiving user-posted commands
        
        ''' Initialize Python-MOOS Communications '''
        self.mooscomms = pymoos.comms()
        self.mooscomms.set_on_connect_callback(self.moos_on_connect)

        self.mooscomms.add_active_queue('CMD_queue', self.moos_on_CMD)
        self.mooscomms.add_message_route_to_active_queue('CMD_queue', 'DVL_CMD')
          
        self.mooscomms.run(self.server_host, self.server_port, self.moos_app_name)
        pymoos.set_moos_timewarp(self.time_warp)

        self.driver = NucleusDriver() # init. driver

        atexit.register(self.disconnect_sensor) # calls disconnect func upon shutdown

        self.engage_comms() # moved to init to solve serial comms with commands

    def moos_on_connect(self):
        ''' On connection to MOOSDB, register for desired MOOS variables (allows for * regex) e.g. register('variable', 'community', 'interval')
        self.mooscomms.register('NODE_*_PING','NODE_*',0) '''
        self.moos_connected = True
        self.mooscomms.register('DVL_CMD', 0) # user can pass commands to driver
                 
        # begins measurement upon startup
        self.driver.start_measurement()

        # optionally enables logging with default path (for ./launch.sh management)
        if cfg.config_dict["logging"] == True:
            print("LOGGING ENABLED")
            PATH = "./logs"
            self.driver.set_log_path(path=PATH)
            self.driver.start_logging()


        return True

    def run(self):
        while True:
            if self.moos_connected:
                self.iterate()
                time.sleep(0.01)
             

    def iterate(self):
        # Reads and renders (print and Notify) packet
        # If statement bc frequency (script vs. DVL) discrep. -> "Packet= NONE"
        packet = self.driver.read_packet()  
        if len(str(packet)) >= 20:
            self.mooscomms.notify("DVL_PACKET", str(packet), pymoos.time())
            print(packet)
            print("\n")
            
    # Shutdown procedure initiated by atexit
    def disconnect_sensor(self):
        self.driver.stop_logging()
        self.driver.stop()
        self.driver.disconnect()
        # shut down?
        print("sensor disconnected")

    def engage_comms(self):
        # Takes in config data to select connection type and implement

        #Serial setup
        if cfg.config_dict["conn_Serial"] == True:
            # Nucleus documentation missing leading slash on "/dev/ttyUSB0"
            SERIAL_PORT = cfg.config_dict["port"] 
            try: # try/except for user config. errors
                self.driver.set_serial_configuration(port=SERIAL_PORT)
                self.driver.connect(connection_type='serial')
            except:
                raise Exception("ERROR: port must be type string, check if correct value and precede by /")

        # TCP Setup
        elif cfg.config_dict["conn_Serial"] == False:
            # Can use hostname (with serial #) or IP address. See manual!
            if cfg.config_dict["use_hostname"] == True:
                TCP_HOST = "NORTEK-" + cfg.config_dict["serialnum"] + ".local"
                print("TCP_HOST******* =", TCP_HOST)
            elif cfg.config_dict["use_hostname"] == False:
                TCP_HOST = cfg.config_dict["IP"]
                print("TCP_HOST******* =", TCP_HOST)
            else:
                raise Exception("ERROR:use_hostname must be type bool")
            try:
                self.driver.set_tcp_configuration(host=TCP_HOST)
                self.driver.connect(connection_type='tcp')
            except:
                raise Exception("ERROR: serialnum/IP must be type string, check if correct value")
        else:
            raise Exception("ERROR:conn_Serial must be type bool")

    def raise_command(self, cont):
        self.driver.stop() # stop driver to relay command
        command = self.CMD  # Refer to the Nucleus documentation for more commands
        if command == "START": # Doesn't handle START/STOP commands -> handle here
            self.driver.start_measurement()
        elif command != "STOP": #If user wants STOP, stops above and doesn't restart
            response = self.driver.send_command(command=command)
            self.mooscomms.notify("DVL_RESPONSE", str(response), pymoos.time()) 
            print("CMD_RESPONSE", response) 
            print("\n")
            # Restarts measurement if user wants continuous
            if cont:
                self.driver.start_measurement() 
        return True

    def moos_on_CMD(self, msg):
        self.CMD = msg.string() # string?
        #  Allows user to query continuous command with "~" -> starts after response
        if self.CMD[0] == "~":
            self.CMD = self.CMD[1:]
            return self.raise_command(True)
        return self.raise_command(False)
        
if __name__ == '__main__':
    sim = pynucleus_driver()
    sim.run()
    
