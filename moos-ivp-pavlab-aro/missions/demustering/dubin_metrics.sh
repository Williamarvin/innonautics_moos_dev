#!/bin/bash -e
# set -x
#---------------------------------------------------------------
#   Script: dubin_post_process.sh
#   Author: Filip Stromstad
#   LastEd: 2024-Aug-15

# Ensure a folder path is provided
if [[ -z "$1" ]]; then
    echo "Error: No folder path provided. Please specify the folder containing the log files."
    exit 1
fi

FOLDER=$1

# Ensure the folder exists
if [[ ! -d "$FOLDER" ]]; then
    echo "Error: Folder '$FOLDER' does not exist."
    exit 1
fi

# Get the name of the last folder in the path
MISSION_NAME=$(basename $FOLDER)

# Check if there exist a .json file in FOLDER
if [[ ! -f "${FOLDER}/${MISSION_NAME}.json" ]]; then
    echo "No .json file found in folder '${FOLDER}'. Running dubin_post_processing.sh"
    # ./dubin_post_process.sh "${FOLDER}"
    ./batch_dubin_post_process.sh "${FOLDER}"
fi


# BASE_FOLDER=$(basename $FOLDER)
# if [[ "$BASE_FOLDER" == batch_* ]]; then
#     # Do nothing
#     echo "Batch folder detected. Skipping MATLAB plotting"
# else
#     # check if the results folder already exists
#     if [[ ! -d "results/${MISSION_NAME}" ]]; then
#         mkdir -p results/${MISSION_NAME}
#         MATLAB_PATH="/Applications/MATLAB_R2023b.app/bin/matlab"
#         MATLAB_SCRIPT_PATH="/Users/filipts/Documents/Thesis/Demustering-MATLAB/Data/plot_trajectories.m"
#         $MATLAB_PATH -nodisplay -nosplash -nodesktop -r "run('$MATLAB_SCRIPT_PATH'); exit;"
#         # $MATLAB_PATH -nosplash -nodesktop -r "run('$MATLAB_SCRIPT_PATH'); exit;"
#         echo "Running dubin_metrics_calc"
#     else 
#         echo "results/${MISSION_NAME} already exists. Skipping MATLAB plotting"
#     fi
# fi
echo "Skipping MATLAB plotting"


g++ -std=c++11 -o dubin_metrics_calc dubin_metrics_calc.cpp \
    -I/Users/filipts/moos-ivp/ivp/src/lib_geometry \
    -I/Users/filipts/moos-ivp/ivp/src/lib_mbutil \
    -L/Users/filipts/moos-ivp/lib \
    -lgeometry \
    -lmbutil \
    # -lapputil \
    # -lpthread



./dubin_metrics_calc \
    --mission-name "${MISSION_NAME}" \
    --folder "${FOLDER}" \