#!/usr/bin/env python3
print("Generating animations")

import csv
from os import listdir, makedirs, remove
from os.path import isfile, join, isdir, splitext
import ternary
import imageio

from plot_data_helpers import parseOpinionLine


data_dir = "data"
images_dir = "data/animation_img"
files_to_process = [f for f in listdir(data_dir) if isfile(join(data_dir, f))]

attention_files = {}
opinion_files = {}

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

# get all data with time stamps
all_states = [] # list where ( time as a float, v_name as a string, raw points as a list)

for filename, data_list in opinion_files.items():
    for line in data_list:
        time, v_name, group, opinion_names, opinion_vals_tpl = parseOpinionLine(line)
        all_states.append( (float(time), v_name, opinion_vals_tpl ) )

all_states = sorted(all_states)
max_window = 0.4  # max time window before moving to another frame.
                  # If another vehicle has data in this window, it will be 
                  # plotted in the same frame. 


# Plot opinions in ternary plot
# from https://github.com/marcharper/python-ternary
agent_color = {'abe':'lightsteelblue', 'ben':'moccasin', 'cal':'limegreen', 'deb':'salmon', 'eve':'violet', 'fin':'peru', 'gil':'pink', 'hix':'yellow', 'max':'limegreen', 'ned':'pink', 'oak':'yellow'}
agent_end   = {'abe':'tab:blue', 'ben':'tab:orange', 'cal':'tab:green', 'deb':'tab:red', 'eve':'tab:purple', 'fin':'tab:brown', 'gil':'deeppink', 'hix':'tab:olive', 'max':'tab:green', 'ned':'deeppink', 'oak':'tab:olive'}
fontsize = 12
axis_limits = {'b':[-8, 16],'l':[-8,16],'r':[-8,16]}


points = {}
count = 0
results_dir = images_dir +"/all"
if not isdir(results_dir):
    makedirs(results_dir)

indx = 0
while indx <  len(all_states):

    # Find all the points here and up to the
    # window in the future.
    time = all_states[indx][0]
    v_name = all_states[indx][1]
    raw_points = all_states[indx][2]

    if v_name not in points:
        # first time seeing this vehicle name
        points[v_name] = []
        
    points[v_name].append(raw_points)
    #print('indx='+ str(indx) + ' adding v_name = ' + v_name)
        
    search_indx = 1
    while indx + search_indx < len(all_states):
        # is the next state within the window?
        time_jump = abs(all_states[indx+search_indx][0] - time)

        #print('time jump to next = ' + str(time_jump))
        if ( time_jump < max_window):
            # add these point to be plotted here
            if all_states[indx+search_indx][1] not in points:
                # first time seeing this vehicle name
                points[all_states[indx+search_indx][1]] = []
                
            points[all_states[indx+search_indx][1]].append(all_states[indx+search_indx][2])
            #print('indx='+ str(indx) + ' and search_indx = ' + str(search_indx) + ' adding v_name = ' + all_states[indx+search_indx][1])
            search_indx += 1
        else:
            # the next jump is too far, we are done here
            search_indx -= 1
            break                        
        
    indx+=1+search_indx

    # plot all we have so far
    figure, tax = ternary.figure(scale=9.0)
    tax.set_axis_limits(axis_limits)
    tax.boundary()
    tax.gridlines(multiple=1, color="black")
    tax.get_ticks_from_axis_limits(1)
    tax.set_custom_ticks(offset=0.017)
    tax.get_axes().axis('off')
    tax.clear_matplotlib_ticks()
  
    for vv_name, raw_points in points.items():
         # Plot this frame
        conv_points = tax.convert_coordinates(raw_points)
        tax.plot(conv_points, linewidth=1.0, label=vv_name, color=agent_color[vv_name])

    for vv_name, raw_points in points.items():
         # Plot this frame
        conv_points = tax.convert_coordinates(raw_points)
        tax.plot([conv_points[-1],conv_points[-1]], marker="o", markersize=5,  color=agent_end[vv_name])

    tax.bottom_axis_label(opinion_names[0],  fontsize=fontsize)
    tax.right_axis_label(opinion_names[1],   fontsize=fontsize, offset=0.12)
    tax.left_axis_label(opinion_names[2],   fontsize=fontsize, offset=0.12)
    tax.legend(loc='upper right')
    
    # build the filename
    image_name = "{}".format(count)
    tax.savefig(results_dir + "/" + image_name + ".png")
    tax.close()
    count = count+1



    
print("Finished writing images to: " + results_dir)
    

# now make a movie from these images
images_to_process = [f for f in listdir(results_dir) if isfile(join(results_dir, f))]
images_to_process = sorted(images_to_process, key=lambda x: int(splitext(x)[0]))

with imageio.get_writer('All_vehicles'+'.mp4', mode='I') as writer:
    for imagename in images_to_process:
        image = imageio.imread(results_dir + '/' + imagename)
        writer.append_data(image)
        remove(results_dir + '/' + imagename)

print("Finished writing animation to: " + 'All_vehicles' + '.mp4')
        

