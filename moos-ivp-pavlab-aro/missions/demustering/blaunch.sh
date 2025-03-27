#!/bin/bash -e
# set -x
#--------------------------------------------------------------
#   Script: blaunch.sh  (Batch Launch)
#   Author: Filip Stromstad
#     Date: 2024-Aug-30
#--------------------------------------------------------------


TIME_WARP=1
MISSION_TIME=700
#If a MOOS process is no longer reporting an uptime, in realtime, we track the process time to bring things down just in case
TRIALS=10
POST_PROCESS_SCRIPT_PATH="" 
LAUNCH_ARGS=""
RE_RUN="true"
RE_TRY="false"
OWT=""
VNUM=8

idx=0
while [[ idx -lt $# ]]; do
    idx=$((idx+1))
    ARGI=${!idx}
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "Not yet!"
	exit 0;
    # elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI:0:4}" = "--t=" ]; then
        TRIALS="${ARGI#--t=*}"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then
        LAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:7}" = "--vnum=" ]; then
        LAUNCH_ARGS+=" $ARGI"
        VNUM="${ARGI#--vnum=}"
    elif [ "${ARGI:0:8}" = "--rerun=" ]; then
        RE_RUN="${ARGI#--rerun=*}"
    elif [ "${ARGI:0:8}" = "--retry=" ]; then
        RE_TRY="${ARGI#--retry=*}"
    elif [ "${ARGI:0:6}" = "--owt=" ]; then
        OWT="${ARGI#--owt=*}"
    else 
	echo "$ME: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

PROCESS_TIME=$(( 10 + $MISSION_TIME / $TIME_WARP))
CONFIG_FILE="config.txt"
CONFIGS=()

# If OWT is set, add OWT to CONFIGS
if [ -n "$OWT" ]; then
    # CONFIGS+=("$(basename "$OWT" .json)")
    CONFIGS+=("TEST")
    data_all=$(jq -c '.data_all[] | to_entries | map({vehicle: .key, NAV_HEADING: .value.NAV_HEADING[0], NAV_X: .value.NAV_X[0], NAV_Y: .value.NAV_Y[0]})' "$OWT")
    TRIALS=$(jq '.data_all | length' "$OWT")
fi


# Read through each line in the file "config.txt" and generate the configuration conditions
while IFS= read -r line || [[ -n "$line" ]]; do
    # Each line is on the form: "--var_name=[start:step:end]"
    # If first character is not a "-", skip the line
    if [ "${line:0:1}" != "-" ]; then
        continue
    fi

    if [[ "$line" == *"["*"]"* ]]; then
        var_name=$(echo "$line" | cut -d '=' -f 1)
        range=$(echo "$line" | cut -d '[' -f 2 | cut -d ']' -f 1)
        
        start=$(echo "$range" | cut -d ':' -f 1)
        step=$(echo "$range" | cut -d ':' -f 2)
        end=$(echo "$range" | cut -d ':' -f 3)

        i=$start    
        while (( $(echo "$i <= $end" | bc -l) )); do
            CONFIGS+=("${var_name}=$i")
            i=$(echo "$i + $step" | bc -l)
        done
    else
        CONFIGS+=("$line")
    fi
done < "$CONFIG_FILE"

# Verify the input parameters before staring the batch
echo "You are about to run a batch with:"
echo "  - ${#CONFIGS[@]} configurations"
echo "  - ${TRIALS} trials per configuration"
echo "  - For a total of $(( ${#CONFIGS[@]} * ${TRIALS} )) missions"
echo "  - This will take approximately $(( ${#CONFIGS[@]} * ${TRIALS} * $MISSION_TIME / $TIME_WARP / 60 )) minutes ($(( ${#CONFIGS[@]} * ${TRIALS} * $MISSION_TIME / $TIME_WARP / 3600 )) hours)"
echo "  - launch args: $LAUNCH_ARGS"
echo "Are you sure you want to continue? [y/n]"
read -r response
if [ "$response" != "y" ]; then
    echo "Exiting..."
    exit 0
fi



idx=0
config_idx=0
t_start=$(date +%s)
ktm

# Create a folder for the batch, if not retrying
if [ "$RE_TRY" = "false" ]; then
    BATCH_NAME="batch_$(date +"%y%m%d-%H%M")"
    mkdir -p ./logs_simulation/$BATCH_NAME
    mkdir -p ./logs_simulation/$BATCH_NAME/failed_attempts
    cp $CONFIG_FILE ./logs_simulation/$BATCH_NAME
else
    BATCH_NAME=$RE_TRY
fi




# Get the name of the first config
FIRST_CONFIG=""


for CONFIG in "${CONFIGS[@]}"; do
    config_idx=$((config_idx+1))
    #get the config name without the preceeding "--" and replace the "=" with "_"
    config_name=$(echo $CONFIG | cut -c 3- | sed 's/=/_/g')

    if [ $config_idx -eq 1 ]; then
        FIRST_CONFIG=$config_name
    fi

    trial_idx=0
    failed_attempts=0
    # for k in $(seq 1 $TRIALS); do
    while [ $trial_idx -lt $TRIALS ]; do
        idx=$((idx+1))
        trial_idx=$((trial_idx+1))
        # ktm
        #Set timers
        mission_duration=0
        # mission_start=$(date +%s)
        mission_name=$(date +"%y%m%d-%H%M%S")_${config_name}_trial_${trial_idx}
        mission_failed=false

        t_now=$(date +%s)
        duration=$((t_now-t_start))

        percent_done=$(echo "scale=2; $idx / (${#CONFIGS[@]} * ${TRIALS}) * 100" | bc -l)
        
        echo "${percent_done}% done: config ${config_idx}/${#CONFIGS[@]} (${CONFIG}), trial ${trial_idx}/${TRIALS} - ${duration} seconds elapsed"

        # If retrying, get the name of the previous mission
        if [ "$RE_TRY" != "false" ]; then
            existing_mission_folder=$(ls -d logs_simulation/$BATCH_NAME/* 2>/dev/null | grep "_${config_name}_trial_${trial_idx}$" || true)
            # If mission already exists, skip it
            if [ -n "$existing_mission_folder" ]; then
                echo "Mission already exists, skipping..."
                continue
            fi
        fi

        # run_number=$((trial_idx - 1))
        # NUM_VEHICLES=$(jq --argjson run "$run_number" '.data_all[$run] | length' "$OWT")
        # VEHICLE_START_POS=$(jq -r --argjson run "$run_number" '
        # .data_all[$run] | to_entries | 
        # map("\(.value.NAV_X[0]),\(.value.NAV_Y[0]),\(.value.NAV_HEADING[0])") | 
        # join(";")' "$OWT")
        # ./launch.sh $LAUNCH_ARGS $TIME_WARP --vnum=$NUM_VEHICLES --vstart="$VEHICLE_START_POS" --batch=$BATCH_NAME --mission_name=$mission_name&> /dev/null &

        # if this is the first CONFIG...
        if [ $config_idx -eq 1 ]; then
            ./launch.sh $LAUNCH_ARGS $CONFIG $TIME_WARP --rand --batch=$BATCH_NAME --mission_name=$mission_name&> /dev/null &
        else
            if [ "$RE_RUN" = "true" ]; then
                FIRST_CONFIG_TRIAL=$(ls -d logs_simulation/$BATCH_NAME/*_"${FIRST_CONFIG}"_trial_"${trial_idx}")
                ./launch.sh $LAUNCH_ARGS $CONFIG $TIME_WARP --rerun=$FIRST_CONFIG_TRIAL --batch=$BATCH_NAME --mission_name=$mission_name&> /dev/null &
            else
                ./launch.sh $LAUNCH_ARGS $CONFIG $TIME_WARP --rand --batch=$BATCH_NAME --mission_name=$mission_name&> /dev/null &
            fi
        fi

        pid_l=$!
        disown $pid_l # Detach the process to avoid the 'Terminated' message
        
        #Let things bring themselves up
        # sleep 15

        # Wait until the expected log folder appears
        elapsed=0

        while [ ! -d "logs_simulation/$BATCH_NAME/$mission_name" ]; do
            sleep 5
            elapsed=$((elapsed+5))
            echo "Waiting for mission folder to appear... $elapsed"
        done

        # Wait for vnum+1 subfolders to appear in the mission folder
        expected_subfolders=$((VNUM + 1))
        echo "Waiting for $expected_subfolders subfolders to appear..."
        elapsed=0

        while true; do
            actual_subfolders=$(find "logs_simulation/$BATCH_NAME/$mission_name" -mindepth 1 -maxdepth 1 -type d | wc -l)
            
            if [ "$actual_subfolders" -ge "$expected_subfolders" ]; then
                break
            fi
            
            sleep 4
            elapsed=$((elapsed+4))
            echo "Waiting for $expected_subfolders subfolders to appear... Currently: $actual_subfolders (Elapsed: $elapsed sec)"
        done

        sleep 25
        echo "Mission $mission_name started"
        mission_start=$(date +%s)

        #Start the mission by poking the DB
        uPokeDB targ_shoreside.moos \
                            DEPLOY_ALL=true \
            MOOS_MANUAL_OVERRIDE_ALL=false \
                            RETURN_ALL=false \
                    STATION_KEEP_ALL=false \
                        DEMUSTER_ALL=false \
                        DEMUSTERING=true \
                    DEMUSTER_ASSIGN=true \
                    DEMUSTER_CONFIG=type_circle \
                        DEMUSTER_BEGIN=true \
            &> /dev/null &

        DONE="false"

        #State monitoring machine, sustaining checks before bringing the mission down
        while [ "${DONE}" = "false" ] ; do 

            sleep 3
            t_now=$(date +%s)
            mission_duration=$((t_now-mission_start))
            echo "   Mission Duration: $mission_duration"
            #1) Has the process time for this session ran over? This would imply a hanging process or application, if it has been exceeded, we cut it
            if [ $mission_duration -gt $PROCESS_TIME ] ; then
                echo "   Process TimeOut" 
                DONE="true"
                # Get the most recent mission log folder and move it to the failed attempts folder
                # mv $(ls -td ./logs_simulation/$BATCH_NAME/*/ | head -n 1) ./logs_simulation/$BATCH_NAME/failed_attempts/
                mission_failed=true
                break
            fi

            #2) Have we received a QUIT_MISSION queue
            echo "   Checking for mission completion"
            if uQueryDB targ_shoreside.moos --condition="DEMUSTER_COMPLETE == true" --wait=3 >& /dev/null ; then 
                echo "   Mission Complete"
                DONE="true"
                break
                #3) Have we been running over the allotted expected mission time?
                # TODO: Ask Ray why include this and the one above?
            # elif uQueryDB targ_shoreside.moos         \
            #         --condition="DB_UPTIME >= $MISSION_TIME" --wait=2 >& /dev/null ; then 
            #     echo "   Mission TimeOut" 
            #     DONE="true"
            #     mission_failed=true
            #     break
            fi
        done

        
        #Make sure every single process is brought down
        sleep 8
        kill $pid_l
        nuke_moos $pid_l &
        ktm &> /dev/null

        sleep 2

        # Get the folder of the last mission
        newest_mission=logs_simulation/$BATCH_NAME/$mission_name
        echo "Post processing $newest_mission"
        ./log_strip.sh $newest_mission

        ktm &> /dev/null
        sleep 2

        echo "Mission $mission_name complete"

        # If the mission failed, move the mission to the failed attempts folder
        if [ "$mission_failed" = true ]; then
            failed_attempts=$((failed_attempts+1))
            mv $newest_mission ./logs_simulation/$BATCH_NAME/failed_attempts/
            
            if [ $failed_attempts -le 3 ]; then
                idx=$((idx-1))
                trial_idx=$((trial_idx-1))
                continue
            else
                echo "Mission failed 3 times, skipping..."
                failed_attempts=0
            fi
        else 
            failed_attempts=0
        fi
                
    done
done


# BUG: Anythign after this point is not executed
echo "All missions complete!"


# Post processing on the batch folder
./batch_dubin_post_process.sh logs_simulation/$BATCH_NAME

# Calculate dubin_metrics on the batch folder
./dubin_metrics.sh logs_simulation/$BATCH_NAME