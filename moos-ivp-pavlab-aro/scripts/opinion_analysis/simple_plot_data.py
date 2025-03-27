#!/usr/bin/env python3
print("Starting plotting")

import csv
import matplotlib.pyplot as plt
from os import listdir
from os.path import isfile, join
import numpy as np

# pass arguments
import argparse

data_dir = 'unset'

parser = argparse.ArgumentParser("simple_plot_data")
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

    
# plot all keys
fig, ax = plt.subplots()

for keyname, data in data_dict.items():
    print('plotting keyname = ' + keyname)
    data_arr = np.array(data)
    times = data_arr[:,0]
    times = [float(ele) for ele in times]
    vals  = data_arr[:,1]
    vals  = [float(ele) for ele in vals]
    
    ax.plot(times, vals, label=keyname)

    
ax.legend()
ax.set_xlabel('Time Since Mission Started (s)')
ax.grid(True)
ax.set_title(data_dir)
plt.savefig(data_dir + 'quick_plot.pdf')
plt.show()
