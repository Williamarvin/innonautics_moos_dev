#!/bin/bash -e
# set -x
#---------------------------------------------------------------
#   Script: dubin_post_process.sh
#   Author: Filip Stromstad
#   LastEd: 2024-Aug-15

echo "Post-processing demustering missions"

if [[ -z "$1" ]]; then
    echo "Error: No folder path provided. Please specify the folder containing the log files."
    exit 1
fi

INPUT_FOLDER=$1

if [[ ! -d "$INPUT_FOLDER" ]]; then
    echo "Error: Folder '$INPUT_FOLDER' does not exist."
    exit 1
fi

# Get the name of the last folder in the path
BASE_FOLDER=$(basename $INPUT_FOLDER)
MISSION_NAMES=()
BATCH=0

if [[ "$BASE_FOLDER" == batch_* ]]; then
    BATCH=1
    for dir in "$INPUT_FOLDER"/*/; do
        MISSION_NAMES+=("$(basename "$dir")")
    done
    # Remove the mission name: "/failed_attempts"
    MISSION_NAMES=(${MISSION_NAMES[@]/failed_attempts/})
else
    MISSION_NAMES=($(basename $INPUT_FOLDER))
fi

# Create a temp directory for the processed logs
rm -rf processed_logs
mkdir -p processed_logs


ALL_START_TIMES=()
ALL_END_TIMES=()
NUM_MISSIONS=${#MISSION_NAMES[@]}
SIMULATION=1
PERIODIC_VARS=("NAV_X" "NAV_Y" "NAV_HEADING" "NAV_SPEED" "DESIRED_SPEED" "COMPASS_HEADING" "DESIRED_THRUST" "PYDIR_THRUST_L" "PYDIR_THRUST_R")
NON_PERIODIC_VARS=("DUBIN_UPDATE" "DEADLOCKED")
SHORE_VARS=("DEMUSTER_BEGIN" "DEMUSTER_COMPLETE")


TOTAL_RUNS=0
# TIME_START_SHORE_FIRST=0
TIME_START_SHORE_FIRST=0
# Repeat for each MISSION_NAME in MISSION_NAMES
for MISSION_NAME in "${MISSION_NAMES[@]}"; do
    echo "Processing mission: ${MISSION_NAME}"
    MISSION_START_TIMES=()
    MISSION_END_TIMES=()


    if [[ $BATCH -eq 1 ]]; then
        FOLDER=${INPUT_FOLDER}/${MISSION_NAME}
    else
        FOLDER=${INPUT_FOLDER}
    fi
    
    # Dynamically retrieve the names from folders starting with LOG_
    NAMES=($(ls -d ${FOLDER}/LOG_* | sed 's|^.*/LOG_||;s/_.*//'))

    # If names are empty --> real life data
    if [[ ${#NAMES[@]} -eq 0 ]]; then
        NAMES=($(ls -d ${FOLDER}/${MISSION_NAME}-*/ | sed "s|^.*/${MISSION_NAME}-||;s/-.*//"))
        SIMULATION=0
    fi

    NAMES=(${NAMES[@]/SHORESIDE/})
    NAMES=(${NAMES[@]/SHORE/})


    ####################################################################
    # Get the start and end times for the demustering from the shore log
    ####################################################################

    shore_log=processed_logs/temp_SHORESIDE.alog

    # If simulation...
    if [[ $SIMULATION -eq 1 ]]; then
        aloggrep ${FOLDER}/LOG_SHORESIDE*/*.alog ${SHORE_VARS[@]} -sd ${shore_log} 1> /dev/null
    else
        aloggrep ${FOLDER}/${MISSION_NAME}-SHORE*/*.alog ${SHORE_VARS[@]} -sd ${shore_log} 1> /dev/null
    fi

    TIME_START_SHORE=$(sed -n '4p' "$shore_log" | awk '{print $3}')
    if [[ "$TIME_START_SHORE_FIRST" == "0" ]]; then
        TIME_START_SHORE_FIRST=$(sed -n '4p' "$shore_log" | awk '{print $3}')
    fi
    ABSOLUTE_OFFSET=$(echo "$TIME_START_SHORE - $TIME_START_SHORE_FIRST" | bc -l)

    in_demuster=0
    while read -r line; do
        # Read the time, event, source, and value from the log file
        time=$(echo "$line" | awk '{print $1}')
        event=$(echo "$line" | awk '{print $2}')

        if [[ "$event" == "DEMUSTER_BEGIN" && $in_demuster -eq 0 ]]; then
            # Capture the begin time
            begin_time="$time"
            in_demuster=1
        elif [[ "$event" == "DEMUSTER_COMPLETE" && $in_demuster -eq 1 ]]; then
            end_time="$time"
            MISSION_START_TIMES+=("$begin_time")
            MISSION_END_TIMES+=("$end_time")
            in_demuster=0

            ABSOLUTE_START_TIME=$(echo "$begin_time + $ABSOLUTE_OFFSET" | bc -l)
            ABSOLUTE_END_TIME=$(echo "$end_time + $ABSOLUTE_OFFSET" | bc -l)
            ALL_START_TIMES+=("$ABSOLUTE_START_TIME")
            ALL_END_TIMES+=("$ABSOLUTE_END_TIME")
        elif [[ "$event" == "DEMUSTER_BEGIN" && $in_demuster -eq 1 ]]; then
            # echo "Warning: Found another DEMUSTER_BEGIN before DEMUSTER_COMPLETE. Skipping..."
            begin_time="$time" # Testing to see if this works
        elif [[ "$event" == "DEMUSTER_COMPLETE" && $in_demuster -eq 0 ]]; then
            echo "Warning: Found DEMUSTER_COMPLETE without a preceding DEMUSTER_BEGIN. Skipping..."
        fi
    done < "$shore_log"
    rm ${shore_log}


    NUM_RUNS=${#MISSION_START_TIMES[@]}
    # echo "Number of runs: ${NUM_RUNS}"
    if [[ $NUM_RUNS -eq 0 ]]; then
        echo "Error: No demustering events found in the shore log. Skipping..."
        continue
    fi

    ####################################################################
    # Process the logs for each vehcile
    ####################################################################

    for NAME in "${NAMES[@]}"; do
        # Create a directory for the processed logs
        mkdir -p processed_logs/${NAME}

        # If simulation...
        if [[ $SIMULATION -eq 1 ]]; then
            LOG_PATH=${FOLDER}/LOG_${NAME}*/*.alog
        else
            LOG_PATH=${FOLDER}/${MISSION_NAME}-${NAME}*/*.alog
            echo "Processing ${NAME} logs..."
        fi
        aloggrep ${LOG_PATH} ${PERIODIC_VARS[@]} ${NON_PERIODIC_VARS[@]} -sd processed_logs/${NAME}/temp_${NAME}.alog  1> /dev/null
        START_TIME=$(sed -n '4p' ${LOG_PATH} | awk '{print $3}')
        TIME_DIFF=$(echo "$START_TIME - $TIME_START_SHORE" | bc -l)

        for ((i = 1; i < NUM_RUNS+1; i++)); do
            TOTAL_RUN_NUMBER=$(echo "${TOTAL_RUNS} + ${i}" | bc -l)
            SYNHCED_START=$(echo "${MISSION_START_TIMES[i-1]} - $TIME_DIFF" | bc -l) 
            SYNHCED_END=$(echo "${MISSION_END_TIMES[i-1]} - $TIME_DIFF" | bc -l)
            alogclip processed_logs/${NAME}/temp_${NAME}.alog processed_logs/${NAME}/${NAME}_${TOTAL_RUN_NUMBER}.alog ${SYNHCED_START} ${SYNHCED_END}
        done

        rm processed_logs/${NAME}/temp_${NAME}.alog
    done
    TOTAL_RUNS=$(echo "${TOTAL_RUNS} + ${NUM_RUNS}" | bc -l)
done


# echo "Names: ${NAMES[@]}"
# echo "Start times: ${ALL_START_TIMES[@]}"
# echo "End times: ${ALL_END_TIMES[@]}"
# echo "Periodic variables: ${PERIODIC_VARS[@]}"
# echo "Non-periodic variables: ${NON_PERIODIC_VARS[@]}"
# echo "Number of runs: ${TOTAL_RUNS}"
# echo "Mission name: ${MISSION_NAME}"
echo "JSON name: ${BASE_FOLDER}"


####################################################################
# Process the data and output the results as a json file
####################################################################

g++ -std=c++11 -o dubin_data dubin_data.cpp
./dubin_data \
    --start-times "${ALL_START_TIMES[@]}" \
    --end-times "${ALL_END_TIMES[@]}" \
    --names "${NAMES[@]}" \
    --periodic-variables "${PERIODIC_VARS[@]}" \
    --non-periodic-variables "${NON_PERIODIC_VARS[@]}" \
    --json-name "${BASE_FOLDER}" \
    --shore-start-time "${TIME_START_SHORE_FIRST}" \
    --folder "${INPUT_FOLDER}" \
    --mission_names "${MISSION_NAMES[@]}" \


echo "Done post-processing Dubin paths"
# rm -rf processed_logs

# mkdir -p results/${MISSION_NAME}
# MATLAB_PATH="/Applications/MATLAB_R2023b.app/bin/matlab"
# MATLAB_SCRIPT_PATH="/Users/filipts/Documents/Thesis/Demustering-MATLAB/Data/data_test.m"
# $MATLAB_PATH -nodisplay -nosplash -nodesktop -r "run('$MATLAB_SCRIPT_PATH'); exit;"
# # $MATLAB_PATH -nosplash -nodesktop -r "run('$MATLAB_SCRIPT_PATH'); exit;"
