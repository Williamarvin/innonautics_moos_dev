#!/bin/bash -e
#-------------------------------------------------------------- 
#   Script: launch_vehicle.sh                                    
#  Mission: ufld_alpha
#   Author: Michael Benjamin  
#     Date: April 2021
#--------------------------------------------------------------
#  Part 1: Declare global var defaults
#--------------------------------------------------------------
ME=`basename "$0"`
GRN='\033[0;32m'
NC='\033[0m' # No Color

TIME_WARP=${TIME_WARP:-1}
JUST_MAKE="no"
VERBOSE=${VERBOSE:-"no"}
AUTO_LAUNCHED=${AUTO_LAUNCHED:-"no"}
CMD_ARGS=""

IP_ADDR="localhost"
MOOS_PORT=${MOOS_PORT:-"9001"}
PSHARE_PORT=${PSHARE_PORT:-"9201"}

SHORE_IP=${SHORE_IP:-"localhost"}
SHORE_PSHARE=${SHORE_PSHARE:-"9200"}
VNAME=${VNAME:-"abe"}
INDEX=${INDEX:-"1"}

REGION=${REGION:-"pavlab"}
START_POS=${START_POS:-"0,0"}
BHV_MFILE="meta_vehicle.bhv"

DEFAULT_POLAR="0,0:28.8,40.95:47.1,67.57:64.1,82.25:89.3,84.11:110,90:135.7,81.18:149.4,77.47:160.2,59.63:167.6,38.29"

#DEFAULT_POLAR="0,0:14.2,33.42:21.1,54.77:45,65:90,80:110,90:"
#DEFAULT_POLAR+="136.5,87.85:150,83:166.8,65.42:180,50"
POLAR=${POLAR:-$DEFAULT_POLAR}

#--------------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#--------------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                        "
	echo "                                                 " 
	echo "Options:                                         "
	echo "  --help, -h                                     " 
	echo "    Print this help message and exit             "
	echo "  --just_make, -j                                " 
	echo "    Just make targ files, but do not launch      "
	echo "  --verbose, -v                                  " 
	echo "    Verbose output, confirm before launching     "
        echo "  --auto, -a                                     "
        echo "     Auto-launched by a script.                  "
        echo "     Will not launch uMAC as the final step.     "
	echo "                                                 "
	echo "  --ip=<localhost>                               " 
	echo "    Force pHostInfo to use this IP Address       "
	echo "  --mport=<9001>                                 "
	echo "    Port number of this vehicle's MOOSDB port    "
	echo "  --pshare=<9201>                                " 
	echo "    Port number of this vehicle's pShare port    "
	echo "                                                 "
	echo "  --shore=<localhost>                            " 
	echo "    IP address location of shoreside             "
	echo "  --vname=<abe>                                  " 
	echo "    Name of the vehicle being launched           " 
	echo "  --index=<1>                                    " 
	echo "    Index for setting MOOSDB and pShare ports    "
	echo "  --start=<X,Y>     (default is 0,0)             " 
	echo "    Start position chosen by script launching    "
	echo "    this script (to ensure separation)           "
	echo "  --mission-pt2pt, -mp                           " 
	echo "    Use the Point-to-Point mision                "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes" 

    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:7}" = "--mport" ]; then
	MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"

    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHORE_IP="${ARGI#--shore=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--index=" ]; then
        INDEX="${ARGI#--index=*}"
	
    elif [ "${ARGI:0:8}" = "--start=" ]; then
        START_POS="${ARGI#--start=*}"
    elif [ "${ARGI}" = "--pavlab" -o "${ARGI}" = "-p" ]; then
        REGION="pavlab"
    elif [ "${ARGI}" = "--forest" -o "${ARGI}" = "-f" ]; then
        REGION="forest_lake"

    elif [ "${ARGI}" = "--mission-pt2pt" -o "${ARGI}" = "-mp" ]; then
        BHV_MFILE="meta_vehicle_pp.bhv"
	
    else 
	echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
	exit 1
    fi
done

MOOS_PORT=`expr $INDEX + 9000`
PSHARE_PORT=`expr $INDEX + 9200`
     
#---------------------------------------------------------------
#  Part 3: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" -o "${CONFIRM}" = "yes" ]; then 
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "CONFIRM =       [${CONFIRM}]      "
    echo "----------------------------------"
    echo "MOOS_PORT =     [${MOOS_PORT}]    "
    echo "PSHARE_PORT =   [${PSHARE_PORT}]  "
    echo "IP_ADDR =       [${IP_ADDR}]      "
    echo "----------------------------------"
    echo "SHORE_IP =      [${SHORE_IP}]     "
    echo "SHORE_PSHARE =  [${SHORE_PSHARE}] "
    echo "VNAME =         [${VNAME}]        "
    echo "INDEX =         [${INDEX}]        "
    echo "----------------------------------"
    echo "START_POS =     [${START_POS}]    "
    echo "REGION =        [${REGION}]       "
    echo "BHV_FILE =      [${BHV_MFILE}]    "
fi

if [ "${CONFIRM}" = "yes" ]; then 
    echo -n "Hit any key to continue with launching"
    read ANSWER
fi


#--------------------------------------------------------------
#  Part 4: Create the .moos and .bhv files. 
#--------------------------------------------------------------
NSFLAGS="-s -f"
if [ "${AUTO}" = "" ]; then
    NSFLAGS="-i -f"
fi

nsplug meta_vehicle.moos targ_$VNAME.moos $NSFLAGS WARP=$TIME_WARP \
       PSHARE_PORT=$PSHARE_PORT     VNAME=$VNAME                   \
       START_POS=$START_POS         SHORE_IP=$SHORE_IP             \
       SHORE_PSHARE=$SHORE_PSHARE   MOOS_PORT=$MOOS_PORT           \
       IP_ADDR=$IP_ADDR             REGION=$REGION                 \
       POLAR=$POLAR                 

nsplug $BHV_MFILE targ_$VNAME.bhv $NSFLAGS VNAME=$VNAME \
       START_POS=$START_POS         REGION=$REGION            \
       POLAR=$POLAR

if [ ${JUST_MAKE} = "yes" ] ; then
    echo "Files assembled; nothing launched; exiting per request."
    exit 0
fi

#--------------------------------------------------------------
#  Part 5: Launch the processes
#--------------------------------------------------------------
echo -e "$GRN Launching $VNAME MOOS Community. WARP=$TIME_WARP $NC"
pAntler targ_$VNAME.moos >& /dev/null &
echo -e "$GRN Done Launching the $VNAME MOOS Community $NC"

#---------------------------------------------------------------
#  Part 6: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#---------------------------------------------------------------
# Part 7: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_$VNAME.moos
kill -- -$$
