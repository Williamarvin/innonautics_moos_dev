#!/bin/bash -e
# set -x
#---------------------------------------------------------------
#   Script: launch.sh
#  Mission: alpha_heron
#   Author: Mike Benjamin
#   LastEd: 2021-Jun-08
#---------------------------------------------------------------

# Set up a trap to catch termination signals
trap "echo 'Terminating...'; kill -- -$$" SIGINT SIGTERM
# Define a convenience function for producing terminal
# debugging/status output depending on the verbosity.
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#---------------------------------------------------------------
#  Part 1: Set global var defaults
#---------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
NUM_VEHICLES=8
RANDOM_HEADING=0
VEHICLE_NAMES=("abe" "ben" "cal" "deb" "eve" "fin" "max" "ned" "oak" "pip")
VEHICLE_NAMES+=("xaa" "xab" "xac" "xad" "xae" "xaf" "xag" "xah" "xai" "xaj" "xak" "xal" "xam" "xan" "xao" "xap")
VEHICLE_NAMES+=("xba" "xbb" "xbc" "xbd" "xbe" "xbf" "xbg" "xbh" "xbi" "xbj" "xbk" "xbl" "xbm" "xbn" "xbo" "xbp")
VEHICLE_NAMES+=("xca" "xcb" "xcc" "xcd" "xce" "xcf" "xcg" "xch" "xci" "xcj" "xck" "xcl" "xcm" "xcn" "xco" "xcp")
VEHICLE_NAMES+=("xda" "xdb" "xdc" "xdd" "xde" "xdf" "xdg" "xdh" "xdi" "xdj" "xdk" "xdl" "xdm" "xdn" "xdo" "xdp")
VEHICLE_NAMES+=("xea" "xeb" "xec" "xed" "xee" "xef" "xeg" "xeh" "xei" "xej" "xek" "xel" "xem" "xen" "xeo" "xep")
VEHICLE_NAMES+=("xfa" "xfb" "xfc" "xfd" "xfe" "xff" "xfg" "xfh" "xfi" "xfj" "xfk" "xfl" "xfm" "xfn" "xfo" "xfp")
VEHICLE_NAMES+=("xga" "xgb" "xgc" "xgd" "xge" "xgf" "xgg" "xgh" "xgi" "xgj" "xgk" "xgl" "xgm" "xgn" "xgo" "xgp")
VEHICLE_NAMES+=("xha" "xhb" "xhc" "xhd" "xhe" "xhf" "xhg" "xhh" "xhi" "xhj" "xhk" "xhl" "xhm" "xhn" "xho" "xhp")
VEHICLE_START_POS=("4,-4,90" "20,-1,100" "14,-6,240" "27,2,210" "17,0,180" "4,-8,90" "20,-5,100" "14,-10,240" "27,-2,210" "17,-4,180")

BATCH_NAME=""
VLAUNCH_ARGS=""
SLAUNCH_ARGS=""
MISSION_NAME=""
RE_RUN_MISSION=""
TURN_RADIUS="5"
MIN_DIFFICULTY=0
MIN_INITIAL_DISTANCE=7
FORMATION_MARGIN=15
CIRCLE_SEGMENT_DEG=180
CIRCLE_RADIUS=0
MODE="decluster" #DECLUSTER, SIMULTANEOUS, SEQUENTIAL, DEPLOY


MTASC="no"
MTASC_SUBNET="192.168.7"
USE_CACHE=""
SHOREIP="localhost"


# REGION="-509838,10440:174329,585000:427750,496100:283500,-88600:-310500,-379600"
# VLAUNCH_ARGS="--auto --region=$REGION "
# SLAUNCH_ARGS="--auto --region=$REGION "


#---------------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#---------------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME [SWITCHES] [time_warp]                                          "
        echo "  --help, -h                  Show this help message                "
        echo "  --just_make, -j             Just make targ files, no launch       "
        echo "  --verbose, -v               Verbose output, confirm before launch "
        echo "  --vnum=<number>             Number of vehicles to launch          "
        echo "  --rand, -r                  Randomize vehicle starting positions  "
        echo "  --nogui, -n                 Launch without GUI                    "
        echo "  --batch=<name>              Batch name for the mission            "
        echo "  --mission_name=<name>       Mission name                          "
        echo "  --rerun=<folder>            Re-run a mission                      "
        echo "  --turnRadius=<radius>       Turn radius for vehicles              "
        echo "  --projectFirst              Project 1st points in front of the veh"
        echo "  --assAlg=<algorithm>        Assignment algorithm                  "
        echo "  --assMet=<metric>           Assignment metric                     "
        echo "  --turnInPlace               Turn in place                         "
        echo "  --headDistWeight=<weight>   Heading distance weight               "
        echo "  --tempBlockLimit=<limit>    Temporary block limit                 "
        echo "  --useDynamicSpeed=<bool>    Use dynamic speed                     "
        echo "  --dthMin=<min>              Minimum distance to hold              "
        echo "  --fhType=<type>             Fast heuristic type                   "
        echo "  --mode=<mode>               Mission mode: decluster, simultaneous, sequential"
        echo "                                                        "
        echo "Options For launches on MTASC machines:                 "
        echo "  --mtasc, -m                                           " 
        echo "    Launch vehicles in the MTASC cluster                " 
        echo "  --shoreip=<ipaddr>                                    " 
        echo "    IP address where nodes can expect to find shoreside "
        echo "  --cache, -c                                           " 
        echo "    Dont confirm IP address of a pablo before launching "
        echo "    a mission. Instead, rely on the ~/.pablos_ipfs file "
        echo "    for a cached mapping of pablo names to IP addresses."
        echo "    Without this flag, the pablo name/IP will first be  "
        echo "    confirmed by contacting the pablo. Longer to launch."
        echo "  --logclean, -l                                        " 
        echo "    Clean (remove) all log files prior to launch        "
        exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [[ "${ARGI}" =~ --vnum=([0-9]+) ]]; then
        NUM_VEHICLES=${BASH_REMATCH[1]}
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="-j"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="--verbose"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then
        SLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI}" = "--rand" -o "${ARGI}" = "-r" ]; then
        RANDOM_HEADING=1
    elif [ "${ARGI:0:8}" = "--batch=" ]; then
        BATCH_NAME="${ARGI#--batch=*}"
    elif [ "${ARGI:0:15}" = "--mission_name=" ]; then
        MISSION_NAME="${ARGI#--mission_name=*}"
    elif [ "${ARGI:0:8}" = "--rerun=" ]; then
        RE_RUN_MISSION="${ARGI#--rerun=*}"
    elif [ "${ARGI:0:13}" = "--turnRadius=" ]; then
        TURN_RADIUS="${ARGI#--turnRadius=*}"
    elif [ "${ARGI:0:15}" = "--projectFirst=" ]; then
        SLAUNCH_ARGS+=" $ARGI"
        VLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:9}" = "--assAlg=" ]; then
        SLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:9}" = "--assMet=" ]; then
        SLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:17}" = "--headDistWeight=" ]; then
        SLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:17}" = "--tempBlockLimit=" ]; then
        VLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:14}" = "--turnInPlace=" ]; then
        SLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:18}" = "--useDynamicSpeed=" ]; then
        VLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:9}" = "--dthMin=" ]; then
        VLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:9}" = "--fhType=" ]; then
        SLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:7}" = "--mode=" ]; then
        MODE="${ARGI#--mode=*}"
    elif [ "${ARGI:0:9}" = "--vstart=" ]; then
        IFS=';' read -r -a VEHICLE_START_POS <<< "${ARGI#--vstart=}"
    # MTASC options
    elif [ "${ARGI}" = "--mtasc" -o "${ARGI}" = "-m" ]; then
	    MTASC="yes"
    elif [ "${ARGI:0:10}" = "--shoreip=" ]; then
        SHOREIP="${ARGI#--shoreip=*}"
    elif [ "${ARGI}" = "--cache" -o "${ARGI}" = "-c" ]; then
        USE_CACHE="--cache"
    elif [ "${ARGI}" = "--logclean" -o "${ARGI}" = "-l" ]; then
	    VLAUNCH_ARGS+=" $ARGI"
    else
        echo "$ME: Bad arg:" $ARGI "Exit Code 1."
        exit 1
    fi
done


#-------------------------------------------------------------
# Part 4: Generate random starting positions, speeds and vnames
#-------------------------------------------------------------
echo "Picking starting positions and speeds"

if [ ! -f "$HOME/.pablo_names" ]; then
    echo "$ME: Could not find ~/.pablo_names. Exit Code 2."
    exit 2
fi

if [ "${RANDSTART}" = "true" -o  ! -f "vpositions.txt" ]; then
    ./pickpos.sh $NUM_VEHICLES
fi
MINSPD=1
MAXSPD=2
if [ "${RANDSTART}" = "true" -o  ! -f "vspeeds.txt" ]; then
    pickpos --amt=$NUM_VEHICLES --spd=$MINSPD:$MAXSPD > vspeeds.txt
fi

# vehicle names are always deterministic in alphabetical order
pickpos --amt=$NUM_VEHICLES --vnames  > vnames.txt

# VEHPOS=(`cat vpositions.txt`)
# SPEEDS=(`cat vspeeds.txt`)
# VNAMES=(`cat vnames.txt`)
PABLOS=(`cat ~/.pablo_names`)  # only need in mtasc mode


#-------------------------------------------------------------
# Part 5: In MTASC mode, the shore IP address must be: 
#         (1) a non-localhost IP address. 
#         (2) a currently active IP interface for this machine
#         (3) an IP address on the local MTASC network
#-------------------------------------------------------------
if [ "$MTASC" = "yes" ]; then
    vecho "Verifying Shoreside IP Address for MTASC mode"

    ADDR=""
    if [ "$SHOREIP" = "localhost" ]; then
        ADDR=`ipmatch.sh --match=$MTASC_SUBNET`
    else
	    ADDR=`ipmatch.sh --match=$SHOREIP`
    fi
    
    if [ "$ADDR" != "" ]; then
	SHOREIP="$ADDR"
    else
	echo "Provide active Shore IP addr on MTASC network. Exit Code 3."
	exit 3
    fi

    verify_ssh_key.sh --pablo    
fi

vecho "SHORE IPAddress: $SHOREIP"


#---------------------------------------------------------------
#  Part 3: Generate starting positions
#---------------------------------------------------------------

if [ "$RANDOM_HEADING" -eq 1 ]; then

    generate_random_number() {
        local min=$1
        local max=$2
        echo $((RANDOM % (max - min + 1) + min))
    }

    generate_random_heading() {
        echo $((RANDOM % 361))
    }

    calculate_distance() {
        local x1=$1
        local y1=$2
        local x2=$3
        local y2=$4
        echo "sqrt(($x2 - $x1) * ($x2 - $x1) + ($y2 - $y1) * ($y2 - $y1))" | bc -l
    }

    generate_random_point_in_radius() {
        local radius=$1
        local anchor_x=$2
        local anchor_y=$3

        # Generate two random numbers between 0 and 1 using /dev/urandom
        local rand1=$(od -An -N4 -tu4 /dev/urandom | tr -d ' ' | awk '{print $1 / 4294967295}')
        local rand2=$(od -An -N4 -tu4 /dev/urandom | tr -d ' ' | awk '{print $1 / 4294967295}')

        # Ensure rand1 and rand2 are single-line values
        rand1=$(echo "$rand1" | tr -d '\n')
        rand2=$(echo "$rand2" | tr -d '\n')

        # Compute random angle and distance using awk
        awk -v r="$radius" -v x="$anchor_x" -v y="$anchor_y" -v u1="$rand1" -v u2="$rand2" '
            BEGIN {
                pi = 3.14159265359
                angle = u1 * 2 * pi
                distance = sqrt(u2) * r
                printf "%.10f,%.10f\n", x + distance * cos(angle), y + distance * sin(angle)
            }
        '
    }


    anchor_x=15
    anchor_y=-35
    min_distance=$MIN_INITIAL_DISTANCE
    attempt_limit=5
    initial_radius=$min_distance

    g++ -std=c++11 -o dubin_metrics_calc_clustered dubin_metrics_calc_clustered.cpp \
        -I/Users/filipts/moos-ivp/ivp/src/lib_geometry \
        -I/Users/filipts/moos-ivp/ivp/src/lib_mbutil \
        -L/Users/filipts/moos-ivp/lib \
        -lgeometry \
        -lmbutil \

    while true; do
        VEHICLE_START_POS=()
        rand_formation_radius=$initial_radius

        for ((i = 0; i < NUM_VEHICLES; i++)); do
            attempts=0
            while true; do
                # Generate a random point within the current radius
                random_point=$(generate_random_point_in_radius $rand_formation_radius $anchor_x $anchor_y)
                IFS=',' read -r x y <<< "$random_point"


                # Check validity of the new position
                valid=true
                for pos in "${VEHICLE_START_POS[@]}"; do
                    IFS=',' read -r px py _ <<< "$pos"
                    distance=$(calculate_distance $x $y $px $py)
                    if (( $(echo "$distance < $min_distance" | bc -l) )); then
                        valid=false
                        break
                    fi
                done

                # If valid, add the position and break out of the loop
                if $valid; then
                    heading=$(generate_random_heading)
                    VEHICLE_START_POS+=("${x},${y},${heading}")
                    break
                fi

                # If too many attempts, increase the radius and reset attempts
                attempts=$((attempts + 1))
                if (( attempts > attempt_limit )); then
                    rand_formation_radius=$((rand_formation_radius + 1))
                    # rand_formation_radius=$((rand_formation_radius * 2))
                    attempts=0
                fi
            done
        done

        output_file="vehicle_data_temp.txt"
        > "$output_file" # Clear previous file contents

        for pos in "${VEHICLE_START_POS[@]}"; do
            IFS=',' read -r x y heading <<< "$pos"
            echo "$x $y $heading" >> "$output_file"
        done

        metric_result=$(./dubin_metrics_calc_clustered)
        echo "$ME: Clustered metric: $metric_result"

        if (( $(echo "$metric_result >= $MIN_DIFFICULTY" | bc -l) )); then
            # Delete temorary file and c++ executable
            rm -f "$output_file"
            rm -f dubin_metrics_calc_clustered
            break
        fi

    done


elif [ -n "$RE_RUN_MISSION" ]; then    
    echo "$ME: Re-running mission $RE_RUN_MISSION"
    VEHICLE_START_POS=()

    rm -rf $RE_RUN_MISSION/temp_nav_logs
    mkdir -p $RE_RUN_MISSION/temp_nav_logs
    
    # Loop through all subfolders in the RE_RUN_MISSION folder and echo the *.alog filename
    COUNTER=0
    for folder in "$RE_RUN_MISSION"/*; do
        if [ -d "$folder" ]; then
            if [[ "$folder" == *SHORE* ]]; then
                continue
            elif [[ "$folder" == *temp_nav_logs* ]]; then
                continue
            fi

            # Check if there is an .alog file in the folder
            for alog_file in "$folder"/*.alog; do
                if [ -f "$alog_file" ]; then
                    let COUNTER=COUNTER+1
                    alogpath=$(realpath $alog_file)
                    aloggrep ${alogpath} NAV_X NAV_Y NAV_HEADING -sd $RE_RUN_MISSION/temp_nav_logs/${COUNTER}.alog 1> /dev/null
                    alogpath_pruned=$RE_RUN_MISSION/temp_nav_logs/${COUNTER}.alog

                    first_nav_x=""
                    first_nav_y=""
                    first_nav_heading=""

                    while IFS= read -r line; do
                        time=$(echo "$line" | awk '{print $1}')
                        variable=$(echo "$line" | awk '{print $2}')
                        value=$(echo "$line" | awk '{print $4}')

                        if [[ "$variable" == "NAV_X" && -z "$first_nav_x" ]]; then
                            first_nav_x=$value
                        elif [[ "$variable" == "NAV_Y" && -z "$first_nav_y" ]]; then
                            first_nav_y=$value
                        elif [[ "$variable" == "NAV_HEADING" && -z "$first_nav_heading" ]]; then
                            first_nav_heading=$value
                        fi

                        # Break if all variables are found
                        if [[ -n "$first_nav_x" && -n "$first_nav_y" && -n "$first_nav_heading" ]]; then
                            break
                        fi
                    done < <(tail -n +5 "$alogpath_pruned")  # Skip the first 4 lines of the file

                    start_pos="${first_nav_x},${first_nav_y},${first_nav_heading}"
                    VEHICLE_START_POS+=("${start_pos}")
                fi
            done
        fi
    done

    rm -rf $RE_RUN_MISSION/temp_nav_logs
    NUM_VEHICLES=$COUNTER
fi



# if mission name is not provided, generate a unique one
if [ -z "$MISSION_NAME" ]; then
    MISSION_NAME=$(date +"%y%m%d-%H%M%S")
fi

echo "$ME: Mission Name is $MISSION_NAME"
echo "$ME: Batch Name is $BATCH_NAME"
mkdir -p ./logs_simulation/$BATCH_NAME/$MISSION_NAME

#---------------------------------------------------------------
#  Part 5: Launch the shoreside
#---------------------------------------------------------------

# If circle radius is 0, set the circle radius to the formation margin
if [ "$CIRCLE_RADIUS" -eq 0 ]; then
    calculate_radius() {
        # Input arguments
        local N=$1        # Number of points
        local d=$2        # Distance between points
        local theta=$3    # Circle segment angle in degrees
        local pi=3.141592653589793

        # Check for invalid input
        if (( N < 2 )); then
            echo "Error: N (number of points) must be at least 2."
            return 1
        fi

        if (( $(echo "$theta == 0" | bc -l) )); then
            echo "Error: Theta (angle) must be greater than 0."
            return 1
        fi

        # Calculate radius
        local numerator=$(echo "($N) * $d * 180" | bc -l)
        local denominator=$(echo "$theta * $pi" | bc -l)
        local radius=$(echo "$numerator / $denominator" | bc -l)


        # Return radius as the result of the function
        # echo "$radius"
        min_radius=30
        if (( $(echo "$radius < $min_radius" | bc -l) )); then
            echo "$min_radius"
        else
            echo "$radius"
        fi
    }

    CIRCLE_RADIUS=$(calculate_radius $NUM_VEHICLES $FORMATION_MARGIN $CIRCLE_SEGMENT_DEG)
fi

echo "$ME: Launching Shoreside ..."
SLAUNCH_ARGS+=" --tr=$TURN_RADIUS"
SLAUNCH_ARGS+=" --circle_radius=$CIRCLE_RADIUS"
SLAUNCH_ARGS+=" --fm=$FORMATION_MARGIN"
SLAUNCH_ARGS+=" --mode=$MODE"
./launch_shoreside.sh --auto --noconfirm $VERBOSE $JUST_MAKE $TIME_WARP --mission_name=$MISSION_NAME --batch=$BATCH_NAME --sim $SLAUNCH_ARGS



#---------------------------------------------------------------
#  Part 6: Initialize and Launch the vehicles
#---------------------------------------------------------------

for ((i=1; i<=NUM_VEHICLES; i++))
do
    VNAME=${VEHICLE_NAMES[$((i-1))]}
    VPOS=${VEHICLE_START_POS[$((i-1))]}
    VLAUNCH_ARGS_COPY=$VLAUNCH_ARGS
    VLAUNCH_ARGS_COPY+=" --auto --noconfirm --sim --vname=$VNAME --index=$i --start=$VPOS --mission_name=$MISSION_NAME --batch=$BATCH_NAME"
    VLAUNCH_ARGS_COPY+=" --use_compass=false"
    VLAUNCH_ARGS_COPY+=" --tr=$TURN_RADIUS"
    VLAUNCH_ARGS_COPY+=" --shore=$SHOREIP"
    VLAUNCH_ARGS_COPY+=" --mode=$MODE"
    VLAUNCH_ARGS_COPY+=" $VERBOSE $JUST_MAKE $TIME_WARP"
    # VLAUNCH_ARGS_COPY+=" --shore=192.168.1.10" #TEMP
    echo "$ME: Launching $VNAME ..."
    echo "Arguments: $VLAUNCH_ARGS_COPY"

    # ./launch_vehicle.sh $VLAUNCH_ARGS_COPY

    if [ "${MTASC}" != "yes" ]; then
        # sleep 0.5
	    ./launch_vehicle.sh $VLAUNCH_ARGS_COPY 	
    else
        sleep 0.5
        SSH_OPTIONS="-o StrictHostKeyChecking=no -o LogLevel=QUIET "
        SSH_OPTIONS+="-o ConnectTimeout=1 -o ConnectionAttempts=1 "
        SSH_OPTIONS+="-o BatchMode=yes "

        PNAME=${PABLOS[$ARRAY_INDEX]}
        IP=`find_pablo_ip_by_name.sh $USE_CACHE $PNAME | tr -d '\r' `
        
        echo "pablo[$PNAME] IP is: $IP"
        VLAUNCH_ARGS_COPY+=" --mission=$MISSION"
        ssh student2680@$IP $SSH_OPTIONS mlaunch.sh $VLAUNCH_ARGS_COPY &
    fi
done



#---------------------------------------------------------------
# Part 7: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_shoreside.moos
kill -- -$$