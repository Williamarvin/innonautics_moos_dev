#!/bin/bash
#-------------------------------------------------------------- 
#   Script: launch_vehicle.sh                                    
#  Mission: cloverrun_pavlab
#   Author: Mike Benjamin
#   LastEd: Oct 2022
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#--------------------------------------------------------------
#  Part 2: Declare global var defaults
#--------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
VERBOSE="no"
AUTO_LAUNCHED="no"
CMD_ARGS=""

REGION="pavlab"
IP_ADDR="localhost"
MOOS_PORT="9001"
PSHARE_PORT="9201"

SHORE_IP="192.168.1.115"
SHORE_PSHARE="9200"
VNAME="aux"
INDEX="50"
XMODE=""


#--------------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
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
	echo "  --nomediate, -nm                               " 
	echo "    No use of pMediator for inter-vessel comms   "
        echo "  --auto, -a                                     "
        echo "     Auto-launched by a script.                  "
        echo "     Will not launch uMAC as the final step.     "
	echo "                                                 "
	echo "  --region=<region>                              " 
	echo "    Name of the region of operation              "
	echo "                                                 "
	echo "  --sim, -s                                      "
	echo "    simulation mode                              "
	echo "  --ip=<localhost>                               " 
	echo "    Force pHostInfo to use this IP Address       "
	echo "  --mport=<9001>                                 "
	echo "    Port number of this vehicle's MOOSDB port    "
	echo "  --pshare=<9201>                                " 
	echo "    Port number of this vehicle's pShare port    "
	echo "                                                 "
	echo "  --shore=<localhost>                            " 
	echo "    IP address location of shoreside             "
	echo "  --shore_pshare=<9200>                          " 
	echo "    Port on which shoreside pShare is listening  "
	echo "  --vname=<aux>                                  " 
	echo "    Name of the community being launched         " 
	echo "  --index=<1>                                    " 
	echo "    Index for setting MOOSDB and pShare ports    "
	echo "                                                 "
	exit 0;
	
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI}" = "--nomediate" -o "${ARGI}" = "-nm" ]; then
	MEDIATED="no"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes" 
    elif [ "${ARGI}" = "--logclean" -o "${ARGI}" = "-l" ]; then
	LOG_CLEAN="yes"

    elif [ "${ARGI:0:9}" = "--region=" ]; then
        REGION="${ARGI#--region=*}"

    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:7}" = "--mport" ]; then
	MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"

    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHORE_IP="${ARGI#--shore=*}"
    elif [ "${ARGI:0:15}" = "--shore_pshare=" ]; then
        SHORE_PSHARE="${ARGI#--shore_pshare=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--color=" ]; then
        COLOR="${ARGI#--color=*}"
    elif [ "${ARGI:0:8}" = "--index=" ]; then
        INDEX="${ARGI#--index=*}"
    elif [ "${ARGI}" = "--sim" -o "${ARGI}" = "-s" ]; then
        XMODE="SIM"
        echo "Simulation mode ON."
    else
	echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
	exit 1
    fi
done

#--------------------------------------------------------------
#  Part 3: Use INDEX for Other Settings
#--------------------------------------------------------------

MOOS_PORT=`expr $INDEX + 9000`
PSHARE_PORT=`expr $INDEX + 9200`

if [ "${XMODE}" = "SIM" ]; then
    IP_ADDR="127.0.0.1"
    IP_ADDR="127.0.0.1"
fi


#---------------------------------------------------------------
#  Part 4: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" ]; then 
    echo "======================================================"
    echo "        launch_aux.sh SUMMARY                         "
    echo "======================================================"
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "JUST_MAKE =     [${JUST_MAKE}]    "
    echo "----------------------------------"
    echo "MOOS_PORT =     [${MOOS_PORT}]    "
    echo "PSHARE_PORT =   [${PSHARE_PORT}]  "
    echo "IP_ADDR =       [${IP_ADDR}]      "
    echo "REGION =        [${REGION}]       "
    echo "----------------------------------"
    echo "SHORE_IP =      [${SHORE_IP}]     "
    echo "SHORE_PSHARE =  [${SHORE_PSHARE}] "
    echo "VNAME =         [${VNAME}]        "
    echo "INDEX =         [${INDEX}]        "
    echo "----------------------------------"
    echo "XMODE =         [${XMODE}]        "
    echo -n "Hit any key to continue with launching ${VNAME}"
    read ANSWER
fi


#-------------------------------------------------------
#  Part 5: If Log clean before launch, do it now. 
#          In MTASC missions, remote cleaning is essential.
#-------------------------------------------------------
if [ "$LOG_CLEAN" = "yes" ]; then
    vecho "Cleaning local Log Files"
    rm -rf LOG* XLOG* MOOSLog*
fi


#--------------------------------------------------------------
#  Part 6: Create the .moos and .bhv files. 
#--------------------------------------------------------------
NSFLAGS="-s -f"
if [ "${AUTO}" = "" ]; then
    NSFLAGS="-i -f"
fi

nsplug meta_aux.moos targ_$VNAME.moos $NSFLAGS WARP=$TIME_WARP \
       PSHARE_PORT=$PSHARE_PORT     VNAME=$VNAME               \
       SHORE_IP=$SHORE_IP           XMODE=$XMODE               \
       SHORE_PSHARE=$SHORE_PSHARE   MOOS_PORT=$MOOS_PORT       \
       IP_ADDR=$IP_ADDR             REGION=$REGION             \                      

       
if [ ${JUST_MAKE} = "yes" ] ; then
    echo "Files assembled; nothing launched; exiting per request."
    exit 0
fi


#--------------------------------------------------------------
#  Part 6: Launch the processes
#--------------------------------------------------------------

echo "Launching $VNAME MOOS Community. WARP="$TIME_WARP
pAntler targ_${VNAME}.moos >& /dev/null &
echo "Done Launching $CNAME MOOS Community"

#---------------------------------------------------------------
#  Part 7: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#---------------------------------------------------------------
# Part 8: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_$VNAME.moos
kill -- -$$
