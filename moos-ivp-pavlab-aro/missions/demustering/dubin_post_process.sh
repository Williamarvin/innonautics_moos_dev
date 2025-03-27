#!/bin/bash -e
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

rm -rf processed_logs

echo "Post-processing Dubin paths"

START_TIMES=()
END_TIMES=()
NUM_RUNS=1
SIMULATION=1

# Dynamically retrieve the names from folders starting with LOG_
NAMES=($(ls -d ${FOLDER}/LOG_* | sed 's|^.*/LOG_||;s/_.*//'))

# If names are empty --> real life data
if [[ ${#NAMES[@]} -eq 0 ]]; then
    NAMES=($(ls -d ${FOLDER}/${MISSION_NAME}-*/ | sed "s|^.*/${MISSION_NAME}-||;s/-.*//"))
    SIMULATION=0
fi

NAMES=(${NAMES[@]/SHORESIDE/})
NAMES=(${NAMES[@]/SHORE/})

echo "Names: ${NAMES[@]}"

PERIODIC_VARS=("NAV_X" "NAV_Y" "NAV_HEADING" "NAV_SPEED" "DESIRED_SPEED" "COMPASS_HEADING" "DESIRED_THRUST" "PYDIR_THRUST_L" "PYDIR_THRUST_R")
NON_PERIODIC_VARS=("DUBIN_UPDATE")

# Create a temp directory for the processed logs
mkdir -p processed_logs

####################################################################
# Get the start and end times for the demustering from the shore log
####################################################################

SHORE_VARS=("DEMUSTER_BEGIN" "DEMUSTER_COMPLETE")
shore_log=processed_logs/temp_SHORESIDE.alog

# If simulation...
if [[ $SIMULATION -eq 1 ]]; then
    aloggrep ${FOLDER}/LOG_SHORESIDE*/*.alog ${SHORE_VARS[@]} -sd ${shore_log}
else
    aloggrep ${FOLDER}/${MISSION_NAME}-SHORE*/*.alog ${SHORE_VARS[@]} -sd ${shore_log}
fi

# aloggrep ${FOLDER}/LOG_SHORESIDE*/*.alog ${SHORE_VARS[@]} -sd ${shore_log}
while read -r line; do
    # Read the time, event, source, and value from the log file
    time=$(echo "$line" | awk '{print $1}')
    event=$(echo "$line" | awk '{print $2}')

    if [[ "$event" == "DEMUSTER_BEGIN" && $in_demuster -eq 0 ]]; then
        # Capture the begin time
        begin_time="$time"
        in_demuster=1
    elif [[ "$event" == "DEMUSTER_COMPLETE" && $in_demuster -eq 1 ]]; then
        # Capture the complete time and print the pair
        end_time="$time"
        # Add the start and end times to the list
        START_TIMES+=("$begin_time")
        END_TIMES+=("$end_time")
        in_demuster=0
    elif [[ "$event" == "DEMUSTER_BEGIN" && $in_demuster -eq 1 ]]; then
        # Skip if a begin is found before a complete
        echo "Warning: Found another DEMUSTER_BEGIN before DEMUSTER_COMPLETE. Skipping..."
        begin_time="$time" # Testing to see if this works
    elif [[ "$event" == "DEMUSTER_COMPLETE" && $in_demuster -eq 0 ]]; then
        # Skip if a complete is found without a corresponding begin
        echo "Warning: Found DEMUSTER_COMPLETE without a preceding DEMUSTER_BEGIN. Skipping..."
    fi
done < "$shore_log"

TIME_START_SHORE=$(sed -n '4p' "$shore_log" | awk '{print $3}')
TIME_START_DIFFS=()
NUM_RUNS=${#START_TIMES[@]}

# rm ${shore_log}

####################################################################
# Process the logs for each vehcile
####################################################################

for NAME in "${NAMES[@]}"; do
    # Create a directory for the processed logs
    mkdir -p processed_logs/${NAME}

    # Process the log file for each name
    echo "Processing ${NAME} logs..."

    # If simulation...
    if [[ $SIMULATION -eq 1 ]]; then
        LOG_PATH=${FOLDER}/LOG_${NAME}*/*.alog
    else
        LOG_PATH=${FOLDER}/${MISSION_NAME}-${NAME}*/*.alog
    fi
    aloggrep ${LOG_PATH} ${PERIODIC_VARS[@]} ${NON_PERIODIC_VARS[@]} -sd processed_logs/${NAME}/temp_${NAME}.alog
    START_TIME=$(sed -n '4p' ${LOG_PATH} | awk '{print $3}')
    TIME_DIFF=$(echo "$START_TIME - $TIME_START_SHORE" | bc -l)
    TIME_START_DIFFS+=("$TIME_DIFF")

    for ((i = 1; i < NUM_RUNS+1; i++)); do
        SYNHCED_START=$(echo "${START_TIMES[i-1]} - $TIME_DIFF" | bc -l)
        SYNHCED_END=$(echo "${END_TIMES[i-1]} - $TIME_DIFF" | bc -l)
        alogclip processed_logs/${NAME}/temp_${NAME}.alog processed_logs/${NAME}/${NAME}_${i}.alog ${SYNHCED_START} ${SYNHCED_END}
        # alogclip processed_logs/${NAME}/temp_${NAME}.alog processed_logs/${NAME}/${NAME}_${i}.alog ${START_TIMES[i-1]} ${END_TIMES[i-1]}
    done

    rm processed_logs/${NAME}/temp_${NAME}.alog
done

echo "Names: ${NAMES[@]}"
echo "Start times: ${START_TIMES[@]}"
echo "End times: ${END_TIMES[@]}"
echo "Periodic variables: ${PERIODIC_VARS[@]}"
echo "Non-periodic variables: ${NON_PERIODIC_VARS[@]}"
echo "Number of runs: ${NUM_RUNS}"
echo "Mission name: ${MISSION_NAME}"
echo "Time differences: ${TIME_START_DIFFS[@]}"


####################################################################
# Process the data and output the results as a json file
####################################################################

g++ -std=c++11 -o dubin_data dubin_data.cpp
./dubin_data \
    --start-times "${START_TIMES[@]}" \
    --end-times "${END_TIMES[@]}" \
    --names "${NAMES[@]}" \
    --periodic-variables "${PERIODIC_VARS[@]}" \
    --non-periodic-variables "${NON_PERIODIC_VARS[@]}" \
    --mission-name "${MISSION_NAME}" \
    --shore-start-time "${TIME_START_SHORE}" \
    --folder "${FOLDER}" \


echo "Done post-processing Dubin paths"
# rm -rf processed_logs

# mkdir -p results/${MISSION_NAME}
# MATLAB_PATH="/Applications/MATLAB_R2023b.app/bin/matlab"
# MATLAB_SCRIPT_PATH="/Users/filipts/Documents/Thesis/Demustering/Data/data_test.m"
# $MATLAB_PATH -nodisplay -nosplash -nodesktop -r "run('$MATLAB_SCRIPT_PATH'); exit;"
# # $MATLAB_PATH -nosplash -nodesktop -r "run('$MATLAB_SCRIPT_PATH'); exit;"
