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

parser = argparse.ArgumentParser("simple_calc_data")
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
    

start_time = 192
end_time = 680

#state_arr = np.array(data_dict['COMPASS_HEADING_RAW'])
state_arr = np.array(data_dict['NAV_HEADING'])
state_time = state_arr[:,0]
state_time = [float(ele) for ele in state_time]
state_val  = state_arr[:,1]
state_val  = [float(ele) for ele in state_val]

errors = []

for row in data_dict['DESIRED_HEADING']:
    time = float(row[0])
    val  = float(row[1])

    if ((time < start_time) or (time > end_time)):
        continue

    # find the closest index in the state
    idx = min(range(len(state_time)), key=lambda i: abs(state_time[i]-time))
    #print('delta t = ' + str(abs(state_time[idx]-time)))

    #print('time is = ' + str(time))
    #print('val is  = ' + str(val))
    #print('state time found = ' + str(state_time[idx]))
    #print('state val found = ' + str(state_val[idx]))
    
    theta1 = val * np.pi / 180.0
    theta2 = state_val[idx] * np.pi / 180.0
    
    #print('theta1 = ' + str(theta1) + ' theta2 = ' + str(theta2))

    dot_prod = cos(theta1) * cos(theta2) + sin(theta1) * sin(theta2)

    if (dot_prod > 1):
        dot_prod = 1.0
    if (dot_prod < -1):
        dot_prod = -1.0
        
    error = acos(dot_prod) * 180.0 / np.pi
    
    errors.append(error)


    
sqrd_errors =[ele*ele for ele in errors]
rmse = sqrt(sum(sqrd_errors)/len(errors))
rmsd = np.sqrt(np.mean((np.array(errors) - rmse) ** 2, axis=0))

ave = np.average(errors)
var = np.var(errors)

print('Error Statistics: Ave = ' + str(ave) + ', Var = ' + str(var))
print('                  Min = ' + str(min(errors)) + ', Max = ' + str(max(errors)))
print('                  RMS = ' + str(rmse) + ', Std = ' + str(rmsd) )
    


fig, axs = plt.subplots()

axs.hist(sqrd_errors, bins=100)
axs.set_xlabel('Heading Error (deg)')
axs.grid(True)
axs.set_title('Heading error hist for ' + data_dir)

plt.show()
