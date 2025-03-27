#!/usr/bin/env python3
print("Starting simple calc")

import csv
import matplotlib.pyplot as plt
from os import listdir
from os.path import isfile, join
import numpy as np

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


