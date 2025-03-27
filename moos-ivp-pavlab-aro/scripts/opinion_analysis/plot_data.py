#!/usr/bin/env python3
print("Starting plotting")

import csv
import ternary
import matplotlib.pyplot as plt
from os import listdir
from os.path import isfile, join
from plot_data_helpers import parseOpinionLine, clipTuple, tupleInRange
from collections import OrderedDict
from math import sqrt


data_dir = "data"
files_to_process = [f for f in listdir(data_dir) if isfile(join(data_dir, f))]

attention_files = {}
opinion_files = {}
contacts_files = {}

for filename in files_to_process:
    rows = []
    with open(data_dir + "/" + filename, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            rows.append(row)

    if "attention" in filename:
        attention_files[filename] = rows

    if "opinions" in filename:
        opinion_files[filename] = rows

    if "contacts" in filename:
        contacts_files[filename] = rows

# sort the dict
attention_files = OrderedDict(sorted(attention_files.items()))
opinion_files = OrderedDict(sorted(opinion_files.items()))
contacts_files = OrderedDict(sorted(contacts_files.items()))


########################################################
# Plot opinions in ternary plot
# from https://github.com/marcharper/python-ternary
#agent_color = {'abe':'lightsteelblue', 'ben':'moccasin', 'cal':'limegreen', 'deb':'salmon', 'eve':'violet', 'fin':'peru', 'gil':'pink', 'hix':'yellow', 'max':'limegreen', 'ned':'pink', 'oak':'yellow'}
#agent_end   = {'abe':'tab:blue', 'ben':'tab:orange', 'cal':'tab:green', 'deb':'tab:red', 'eve':'tab:purple', 'fin':'tab:brown', 'gil':'deeppink', 'hix':'tab:olive', 'max':'tab:green', 'ned':'deeppink', 'oak':'tab:olive'}
agent_color = {'abe':'tab:blue',
               'ben':'tab:orange',
               'cal':'tab:green',
               'deb':'tab:red',
               'eve':'tab:purple',
               'fin':'tab:brown',
               'gil':'tab:pink',
               'hix':'tab:gray',
               'ike':'lime',
               'jim': 'red',
               'kim': 'springgreen',
               'lou': 'gold', 
               'max':'tab:olive',
               'ned':'tab:cyan',
               'oak':'tab:green'}


agent_end =   {'abe':'tab:blue',
               'ben':'tab:orange',
               'cal':'tab:green',
               'deb':'tab:red',
               'eve':'tab:purple',
               'fin':'tab:brown',
               'gil':'tab:pink',
               'hix':'tab:gray',
               'ike':'lime',
               'jim': 'red',
               'kim': 'springgreen',
               'lou': 'gold', 
               'max':'tab:olive',
               'ned':'tab:cyan',
               'oak':'tab:green'}



fontsize = 12
figure, tax = ternary.figure(scale=9.0)
range_lim = [-7.0, 14.0]
axis_limits = {'b':range_lim,'l':range_lim,'r':range_lim}
tax.set_axis_limits(axis_limits)
tax.boundary()
tax.gridlines(multiple=1, color="black")

saved_opinion_names = ['not_active', 'not_active', 'not_active']

# plot data for each record
for filename, data_list in opinion_files.items():
    points = []
    times  = []
    for line in data_list:
         time, v_name, group, opinion_names, opinion_vals_tpl = parseOpinionLine(line)
         points.append(opinion_vals_tpl)
         times.append(time)

         if ( (opinion_names[0] != 'not_active') and (opinion_names[1] != 'not_active') and (opinion_names[2] != 'not_active') ):
             saved_opinion_names[0] = opinion_names[0]
             saved_opinion_names[1] = opinion_names[1]
             saved_opinion_names[2] = opinion_names[2]

    # clip the data 
    points_clipped = []
    for point in points:
        if (tupleInRange(point, range_lim[0], range_lim[1]) ):
            points_clipped.append( point )
        else:
            points_clipped.append( clipTuple(point, range_lim[0], range_lim[1]) )

    # Plot the data
    conv_points = tax.convert_coordinates(points_clipped)
    
    tax.plot(conv_points, linewidth=1.0, label=v_name, color=agent_color[v_name])
    
# again plot the end points on top. 
for filename, data_list in opinion_files.items():
    points = []
    times  = []
    for line in data_list:
         time, v_name, group, opinion_names, opinion_vals_tpl = parseOpinionLine(line)
         points.append(opinion_vals_tpl)
         times.append(time)
    
    # Plot the data
    conv_points = tax.convert_coordinates(points)
    tax.plot([conv_points[-1],conv_points[-1]], marker="o", markersize=10,  color=agent_end[v_name])


    
tax.get_ticks_from_axis_limits(1)
tax.set_custom_ticks(offset=0.017)
tax.bottom_axis_label(saved_opinion_names[0].capitalize(),  fontsize=fontsize)
tax.right_axis_label(saved_opinion_names[1].capitalize(),   fontsize=fontsize, offset=0.12)
tax.left_axis_label(saved_opinion_names[2].capitalize(),   fontsize=fontsize, offset=0.12)
tax.get_axes().axis('off')
tax.clear_matplotlib_ticks()

tax.legend()
tax.savefig("OpinionTrajectories.pdf", dpi=1200)
print("Finished generating plot: " + "OpinionTrajectories.pdf")
tax.show()



###############################################################
# Plot attention and connection degree from all agents in single plot

fig2, (ax1, ax2) = plt.subplots(2, sharex=True)

for filename, data_list in attention_files.items():
    name = filename[:3]
    times = []
    atten_mag = []
    for pair in data_list:
        times.append(float(pair[0])/60.0)
        atten_mag.append(float(pair[1]))
    
    ax1.plot(times, atten_mag, label=name, color=agent_color[name])

ax1.set_ylabel('Attention (u)', fontsize=20)
ax1.tick_params(axis='both', labelsize=18)
ax1.grid()
ax1.legend(fontsize=16)

for filename, data_list in contacts_files.items():
    name = filename[:3]
    times = []
    deg_of_vertex = []
    for line in data_list:

        #if (name == 'deb'):
        #    print(line)

        if (len(deg_of_vertex) > 0):
           deg_of_vertex.append(deg_of_vertex[-1])
           times.append(float(line[0])/60.0)

        times.append(float(line[0])/60.0)
        
        if (len(line) == 2):
            if (line[1] == ''):
                deg_of_vertex.append(0.0)
            else:
                deg_of_vertex.append(len(line) - 1.0)
        else:  
            deg_of_vertex.append(len(line) - 1.0)

    ax2.plot(times, deg_of_vertex, label=name, color=agent_color[name])

    #if (name == 'deb'):
    #    print(deg_of_vertex)

ax2.set_ylabel('Deg of Vertex/Vehicle', fontsize=20)
ax2.set_xlabel('Time Since Mission Start (min)', fontsize=20)
ax2.set_xlim(0,130)
ax2.tick_params(axis='both', labelsize=18)
ax2.grid()

fig2.savefig("AttentionTrajectories.pdf", dpi=1200)
print("Finished generating plot: " + "AttentionTrajectories.pdf")
plt.show()
    
    

########################################################################
## Plot magnitude of opinion for each vehicle

fig3, ax3 = plt.subplots()

# plot data for each record
for filename, data_list in opinion_files.items():
    op_mag = []
    times  = []
    for line in data_list:
        time, v_name, group, opinion_names, opinion_vals_tpl = parseOpinionLine(line)
        
        mag = 0.0
        for op_val in opinion_vals_tpl:
            mag = mag + op_val*op_val
        
        op_mag.append(sqrt(mag))
        times.append(float(time))


        #print(type(time))
        #print(type(mag))
    ax3.plot(times, op_mag, linewidth=1.0, label=v_name, color=agent_color[v_name])

ax3.set_ylabel('Magnitude of Opinions', fontsize=20)
ax3.set_xlabel('Time Since Mission Start (min)', fontsize=20)
#ax3.set_xlim(0,130)
ax3.tick_params(axis='both', labelsize=18)
ax3.grid()
ax3.legend(fontsize=16)


fig3.savefig("OpinionMagnitudes.pdf", dpi=1200)
print("Finished generating plot: " + "OpinionMagnitudes.pdf")
plt.show()
