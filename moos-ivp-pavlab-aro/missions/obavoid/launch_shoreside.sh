#!/bin/bash 
#------------------------------------------------------------ 
#   Script: launch_shoreside.sh    
#  Mission: obavoid
#   Author: M.Benjamin
#   LastEd: May 26 2024
#------------------------------------------------------------
#  Part 1: Set convenience functions for producing terminal
#          debugging output, and catching SIGINT (ctrl-c).
#------------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }
on_exit() { echo; echo "Halting all apps"; kill -- -$$; }
trap on_exit SIGINT

#------------------------------------------------------------ 
#  Part 2: Set global variable default values
#------------------------------------------------------------ 
ME=`basename "$0"`
CMD_ARGS=""
TIME_WARP=1
JUST_MAKE="no"
VERBOSE=""
AUTO_LAUNCHED="no"
LAUNCH_GUI="yes"

IP_ADDR="localhost"
MOOS_PORT="9000"
PSHARE_PORT="9200"

VNAMES=""

#custom
DRESET="false"
POINTS="false"
SEP="10"
ENC="25"


#------------------------------------------------------------ 
#  Part 3: Check for and handle command-line arguments
#------------------------------------------------------------ 
for ARGI; do
    echo $ARGI
    CMD_ARGS+="${ARGI} "
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS] [time_warp]                     " 
	echo "                                               "
	echo "Options:                                       "
	echo "  --help, -h         Show this help message    " 
	echo "  --just_make, -j    Only create targ files    " 
	echo "  --verbose, -v      Verbose, confirm launch   "
	echo "                                               "
        echo "  --auto, -a                                   "
        echo "    Auto-launched by a script.                 "
        echo "    Will not launch uMAC as the final step.    "
        echo "  --nogui, -n                                  "
        echo "    Headless mode - no pMarineViewer etc       "
	echo "                                               "
	echo "  --ip=<localhost>                             "
	echo "    Force pHostInfo to use this IP Address     "
	echo "  --mport=<9000>                               "
	echo "    Port number of this vehicle's MOOSDB port  "
	echo "  --pshare=<9200>                              "
	echo "    Port number of this vehicle's pShare port  "
        echo "  --vnames=<vnames>                            "
        echo "    Colon-separate list of all vehicle names   "
	echo "                                               "
	echo "Options:                                       "
	echo "  --dynamic_reset, -d                          " 
	echo "    Enable dynamice resetting of obstacles     "
	echo "    in the obstacle simulator                  "
	echo "  --enc=<25>                                   " 
	echo "    Number of encounters in headless mission   "
	echo "  --sep=<dist>                                 " 
	echo "    Min sep between obstacles (Default 10)     "
	echo "  --points, -p                                 " 
	echo "    Enable the simulated sensor points mode    "
	echo "    in the obstacle simulator                  "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then
        LAUNCH_GUI="no"

    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:7}" = "--mport" ]; then
	MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"
    elif [ "${ARGI:0:9}" = "--vnames=" ]; then
        VNAMES="${ARGI#--vnames=*}"

    elif [ "${ARGI}" = "--dynamic_reset" -o "${ARGI}" = "-d" ]; then
	DRESET="true"
    elif [ "${ARGI:0:6}" = "--enc=" ]; then
        ENC="${ARGI#--enc=*}"
    elif [ "${ARGI:0:6}" = "--sep=" ]; then
        SEP="${ARGI#--sep=*}"
    elif [ "${ARGI}" = "--points" -o "${ARGI}" = "-p" ]; then
	POINTS="true"

    else
	echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
	exit 1
    fi
done

#------------------------------------------------------------ 
#  Part 4: If not auto_launched (likely running in the field),
#          and the IP_ADDR has not be explicitly set, try to get
#          it using the ipaddrs.sh script. 
#------------------------------------------------------------ 
if [ "${AUTO_LAUNCHED}" = "no" -a "${IP_ADDR}" = "localhost" ]; then
    MAYBE_IP_ADDR=`ipaddrs.sh --blunt`
    if [ $? = 0 ]; then
	IP_ADDR=$MAYBE_IP_ADDR
    fi
fi

#------------------------------------------------------------ 
#  Part 5: If verbose, show vars and confirm before launching
#------------------------------------------------------------ 
if [ "${VERBOSE}" = "yes" ]; then 
    echo "=================================="
    echo "  launch_shoreside.sh SUMMARY     "
    echo "=================================="
    echo "$ME                               "
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "JUST_MAKE =     [${JUST_MAKE}]    "
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "----------------------------------"
    echo "IP_ADDR =       [${IP_ADDR}]      "
    echo "MOOS_PORT =     [${MOOS_PORT}]    "
    echo "PSHARE_PORT =   [${PSHARE_PORT}]  "
    echo "LAUNCH_GUI =    [${LAUNCH_GUI}]   "
    echo "----------------------------------"
    echo "VNAMES =        [${VNAMES}]       "
    echo "--------------------------(Custom)"
    echo "DRESET =        [${DRESET}]       "
    echo "ENC =           [${ENC}]          "
    echo "SEP =           [${SEP}]          "
    echo "POINTS =        [${POINTS}]       "
    echo -n "Hit any key to continue launch "
    read ANSWER
fi

#------------------------------------------------------------ 
#  Part 6: Create the shoreside mission file
#------------------------------------------------------------ 
NSFLAGS="--strict --force"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="--interactive --force"
fi

nsplug meta_shoreside.moos targ_shoreside.moos $NSFLAGS WARP=$TIME_WARP \
       IP_ADDR=$IP_ADDR             MOOS_PORT=$MOOS_PORT    \
       PSHARE_PORT=$PSHARE_PORT     LAUNCH_GUI=$LAUNCH_GUI  \
       VNAMES=$VNAMES                                       \
       DRESET=$DRESET               TEST_ENCOUNTERS=$ENC    \
       POINTS=$POINTS               SEP=$SEP

if [ "${JUST_MAKE}" = "yes" ]; then
    echo "$ME: Targ files made; exiting without launch."
    exit 0
fi

#------------------------------------------------------------ 
#  Part 7: Launch the shoreside MOOS community
#------------------------------------------------------------ 
echo "Launching Shoreside MOOS Community. WARP="$TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &
echo "Done Launching Shoreside Community"

#------------------------------------------------------------ 
#  Part 8: If launched from script, we're done, exit now
#------------------------------------------------------------ 
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#------------------------------------------------------------ 
# Part 9: Launch uMAC until the mission is quit
#------------------------------------------------------------ 
uMAC targ_shoreside.moos

trap "" SIGINT
kill -- -$$

