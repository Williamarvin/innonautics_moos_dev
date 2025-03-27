#!/usr/bin/env python3

'''
This is a stub for a module launching AIS tracks in the
Virtual Ocean Testbed

Original    (MATLAB): Henrik Schmidt, Nov. 2020
Translation (Python): Henrik Schmidt, Jan. 2021
'''

import numpy as np
import pymoos
import logging

# pymoos doesn't currently bind to CMOOSApp method
# for config parsing; use this for convenience
from lib_pylamss import moos_config
from random import seed
from random import gauss

# Use uPlot's Ansi module for color-coding logs
from lib_pylamss.Ansi import Ansi as ansi


logger = logging.getLogger(__name__)


class NAV_STUB:
  '''
  This is a stub for a module creating track solutions for Hydroman
  '''

  def __init__(self,*args,**kwargs):

    # Default server settings (override via config)
    self.server_host = 'localhost'
    self.server_port = 9000
    self.time_warp   = 1

    # Process args and read config
    self.name = args[2]
    self.cfg = {'config':args[1],
                'apptick':4,
                'std_x': 15,
                'std_y': 15,
                'track_interval': 30
               }
    self.read_config()

    # Set random number variables

    self.std_x = self.cfg['std_x']
    self.std_y = self.cfg['std_y']
    self.track_delta = self.cfg['track_interval'] # 30 seconds between track solutions
    
    seed(1)
    # Initialize some variables
    self.last_track = 0

    self.start_time = None;
    self.lat_origin = None;
    self.lon_origin = None;
    self.true_x = None;
    self.true_y = None;

    self.got_stuff = False;

    # self.ownship_id = 0 # use None as 'empty'

    self.vars_to_reg = ['DB_TIME',
                        'LAT_ORIGIN',
                        'LONG_ORIGIN',
                        'HYDROMAN_IFS_X',
                        'HYDROMAN_IFS_Y']

    # Log pymoos app's configuration:
    self.log_config()

    # start pymoos.comms, with callbacks
    pymoos.set_moos_timewarp(self.time_warp)
    self.pmc = pymoos.comms()
    self.pmc.set_on_connect_callback(self.on_connect)
    self.pmc.set_on_mail_callback(self.on_new_mail)

    self.pmc.run(self.server_host,self.server_port,self.name)
    self.pmc.wait_until_connected(2000)


  def iterate(self):

    if self.got_stuff:
      if (self.current_time >= self.last_track+self.track_delta):

        r_lat = gauss(0,1)
        r_lon = gauss(0,1)
        est_x = self.true_x + r_lon * self.std_x
        est_y = self.true_y + r_lat * self.std_y

        track_msg = ('@PB[lamss.hydroman.protobuf.HydromanInternal]'
                    + ' track_update {'
                    + ' time: %12.1f' %(self.current_time)
                    + ' vehicle_x: %f' %(est_x)
                    + ' vehicle_y: %f' %(est_y)
                    + ' vehicle_x_stdev: %f' %(self.std_x)
                    + ' vehicle_y_stdev: %f' %(self.std_y)
                    + ' vehicle_corr_xy: 0 }'
                    )

        logger.info('Time : %f' % (self.current_time-self.start_time))
        logger.info('Posting track solution x: %f' %(est_x)
                    + ' y: %f' %(est_y))
        self.pmc.notify('HYDROMAN_ICEX_TRACK_UPDATE',track_msg)
        self.last_track = self.current_time


  def handle_mail(self,msg):

    if msg.is_name('DB_TIME'):
      self.current_time = msg.double()
      if self.start_time==None:
        self.start_time = self.current_time

    elif msg.is_name('LAT_ORIGIN'):
      self.lat_origin = msg.double()

    elif msg.is_name('LONG_ORIGIN'):
      self.lon_origin = msg.double()

    elif msg.is_name('HYDROMAN_IFS_X'):
      self.true_x = msg.double()

    elif msg.is_name('HYDROMAN_IFS_Y'):
      self.true_y = msg.double()

    self.got_stuff = all([x!=None for x in [self.start_time,
                                            self.lat_origin,
                                            self.lon_origin,
                                            self.true_x,
                                            self.true_y]])


  # =================================================
  # moos_config parser in lib_pylamss for convenience
  def read_config(self):
    moos_config.read_config(self,self.cfg['config'],self.name)
  def log_config(self):
    moos_config.log_config(self)

  # ======================
  # pymoos.comms callbacks
  def on_connect(self):
    for var in self.vars_to_reg:
      logger.info(self.name +' Register for '+var)
      self.pmc.register(var,0)
    return True
  def on_new_mail(self):
    for msg in self.pmc.fetch():
      self.handle_mail(msg)
    return True


# ==================================================
# ==================================================
# This section defines how the program should behave
# when called directly, rather than as an import to
# another program
if __name__=='__main__':

  import time
  import sys

  # Logger should be configured by 'primary' program
  logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s | %(name)s.%(funcName)-16s %(levelname)-8s : %(message)s',
    datefmt='[%Y-%m-%d %H:%M:%S]')
  logging.Formatter.converter = time.gmtime

  # create instance of our class
  stub = NAV_STUB(*sys.argv)

  # time.sleep(1)

  # call iterate based on a crude apptick interpretation
  while True:
      stub.iterate()
      time.sleep(1.0/stub.cfg['apptick'])


