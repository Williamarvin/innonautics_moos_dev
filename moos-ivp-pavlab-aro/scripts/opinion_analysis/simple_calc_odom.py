#!/usr/bin/env python3
print("Starting simple calc")

import csv
import matplotlib.pyplot as plt
from os import listdir
from os.path import isfile, join
import numpy as np
from math import cos, sin, acos, sqrt

# pass arguments

# pass arguments
import argparse

data_dir = 'unset'

parser = argparse.ArgumentParser("simple_calc_odm")
parser.add_argument("data_dir", help="the directory of the data file. It ends with /data.", type=str)
args = parser.parse_args()
data_dir = args.data_dir

files_to_process = [f for f in listdir(data_dir) if isfile(join(data_dir, f))]

data_dict = {}
for filename in files_to_process:
    rows = []
    with open(data_dir + "/" + filename, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            rows.append(row)

    # cannot be duplicate files in the system
    # remove the .klog
    keyname = filename[:-5]
    data_dict[keyname] = rows

    
for key in data_dict:
    print(key)

time_nav_x = []
nav_x = []
for row in data_dict['NAV_X']:
    time_nav_x.append(float(row[0]))
    nav_x.append(float(row[1]))


time_nav_y = []
nav_y = []
for row in data_dict['NAV_Y']:
    time_nav_y.append(float(row[0]))
    nav_y.append(float(row[1]))


if (len(nav_x) != len(nav_y)):
    print('Error NAV_X and NAV_Y not the same length')
    quit()


odom = 0.0
for i in range(len(nav_x)-1):

    
    # find the closest nav_y to i
    # found the best_j_for_i_plus_1
    closest_time_nav_y = 100000.0
    best_j = 0
    closest_time_nav_y_for_i_plus_1 = 100000.0
    best_j_for_i_plus_1 = 0
    for j in range(len(nav_y)-1):
        dt = abs(time_nav_x[i] - time_nav_y[j])
        if (dt < closest_time_nav_y):
            closest_time_nav_y = dt
            best_j = j

        dt2 = abs(time_nav_x[i+1] - time_nav_y[j])
        if (dt2 < closest_time_nav_y_for_i_plus_1):
            closest_time_nav_y_for_i_plus_1 = dt2
            best_j_for_i_plus_1 = j
    
       

    dx = nav_x[i] - nav_x[i+1]
    dy = nav_y[best_j] - nav_y[best_j_for_i_plus_1]
    ds = np.sqrt(dx**2 + dy**2)
    print('time diff = ' + str(closest_time_nav_y))
    print('ds = ' + str(ds))
    print('odom = ' + str(odom))
    odom += ds
    

print('Total Odom = ' + str(odom) )


