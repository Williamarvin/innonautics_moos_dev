#!/bin/bash
#---------------------------------------------------------------
#   Script: update_params.sh
#  Mission: beta_opinion
#   Author: Tyler Paine
#   LastEd: Dec 2024
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

mkdir -p "results"
# base URL of the file to download
url="https://oceanai.mit.edu/monte/results/tpaine/GCID_learn/policy_iter2/eval_"

# Name of the  file
filename="results.csv"

declare -a RUN=("cent" "param1" "param2" "param3"
                "param4" "param5" "param6" "param7"
		"param8" 
               )


for RNAME in "${RUN[@]}"
do
    total_url="${url}${RNAME}_job/${filename}"
    target_loc="results/${RNAME}.csv"

    wget -O "${target_loc}" "${total_url}"

    # Check if download was successful
    if [ $? -eq 0 ]; then
	echo "File ${total_url} downloaded successfully!"	
    else
	echo "File ${total_url} download failed."
    fi
done



./process_mission2.py results
./gen_param_set.py params/params_cent.txt
