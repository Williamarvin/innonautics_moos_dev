#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch_shoreside.sh    
#  Mission: ufld_saxis
#   Author: Michael Benjamin   
#   LastEd: January 2021
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#-------------------------------------------------------------- 
#  Part 2: Set Global variables
#-------------------------------------------------------------- 
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
VERBOSE=""
AUTO_LAUNCHED="no"
CMD_ARGS=""
#REGION="-100,-100:100,-100:100,-300:-100,-300"
REGION="0,-15:150,-15:150,-165:0,-165"
IP_ADDR="localhost"
MOOS_PORT="9000"
PSHARE_PORT="9200"
MAP="MIT_SP.tif"
VNAMES=""
DETECTION_RADIU=1000
HD=""
ADD_SPEED="1.0"
AMT=9
CHOICE="--choice=sp"
ORIGIN="plug_origin_warp1.moos"
#---------------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#---------------------------------------------------------------
for ARGI; do
    CMD_ARGS+="${ARGI} "
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS] [time_warp]                       " 
	echo "                                                 "
	echo "Options:                                         "
        echo "  --help, -h                                     "
        echo "    Display this help message                    "
	echo "  --just_make, -j                                "
	echo "    Just make target files. Do not launch.       "
        echo "  --verbose, -v                                  "
        echo "    Increase verbosity                           "
        echo "  --auto, -a                                     "
        echo "     Auto-launched by a script.                  "
        echo "     Will not launch uMAC as the final step.     "

	echo "  --ip=<localhost>                               " 
	echo "    Force pHostInfo to use this IP Address       "
	echo "  --mport=<9000>                                 "
	echo "    Port number of this vehicle's MOOSDB port    "
	echo "  --pshare=<9200>                                " 
	echo "    Port number of this vehicle's pShare port    "
	echo "                                                 "
	echo "  --region=Polygon                               " 
	echo "    Set the polygon cover region                 "
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
    elif [ "${ARGI:0:5}" = "--hd=" ]; then
        HD="${ARGI#--hd=*}"
    elif [ "${ARGI:0:8}" = "--cross=" ]; then
        CROSS="${ARGI#--cross=*}"
    elif [ "${ARGI:0:7}" = "--mport" ]; then
	    MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:7}" = "--drad=" ]; then
	    DETECTION_RADIU=${ARGI#--drad=*}
    elif [ "${ARGI:0:6}" = "--map=" ]; then
	    MAP=${ARGI#--map=*}
    elif [ "${ARGI:0:6}" = "--amt=" ]; then
	    AMT=${ARGI#--amt=*}
    elif [ "${ARGI:0:5}" = "--adv" ]; then
	    ADV="true"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"

    elif [ "${ARGI:0:9}" = "--vnames=" ]; then
	VNAMES="${ARGI#--vnames=*}"
    elif [ "${ARGI:0:9}" = "--region=" ]; then
        REGION="${ARGI#--region=*}"
    elif [ "${ARGI:0:9}" = "--origin=" ]; then
        ORIGIN="${ARGI#--origin=*}"
    else 
	echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
	exit 1
    fi
done
#---------------------------------------------------------------
#  Part 4: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" ]; then 
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}"
    echo "TIME_WARP =     [${TIME_WARP}]"
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "MOOS_PORT =     [${MOOS_PORT}]"
    echo "PSHARE_PORT =   [${PSHARE_PORT}]"
    echo "IP_ADDR =       [${IP_ADDR}]"
    echo "VNAMES=         [${VNAMES}]"
    echo -n "Hit any key to continue with launching"
    read ANSWER
fi

#-------------------------------------------------------
#  Part 5: Create the .moos and .bhv files. 
#-------------------------------------------------------
D1R=$((1+$DETECTION_RADIU))
D2R=$((2+$DETECTION_RADIU))
nsplug meta_shoreside.moos targ_shoreside.moos -i -f WARP=$TIME_WARP \
       IP_ADDR=$IP_ADDR         PSHARE_PORT=$PSHARE_PORT             \
       MOOS_PORT=$MOOS_PORT     VNAMES=$VNAMES                       \
       COVER_REGION=$REGION     DETECTION_RADIUS=$DETECTION_RADIU    \
       D1=$D1R                  D2=$D2R                              \
       MAP=$MAP                 ORIGIN=$ORIGIN                       \
       HD=$HD          

if [ ${JUST_MAKE} = "yes" ]; then
    echo "Files assembled; nothing launched; exiting per request."
    exit 0
fi

#-------------------------------------------------------
#  Part 6: Launch the processes
#-------------------------------------------------------
echo "Launching $VNAME MOOS Community. WARP=" $TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &
echo "Done Launching Shoreside Community"


#-------------------------------------------------------
#  Part 6: Launch the Simulated Adversary
#-------------------------------------------------------

if [ "${ADV}" = "true" ]; then 
    IN=`expr $AMT + 1`
    AMPORT=`expr $IN + 9000`
    APPORT=`expr $IN + 9200`
    AD_SPEED="--adspeed=$ADD_SPEED"
    ./launch_adversary.sh $AD_SPEED  --origin=$ORIGIN $CHOICE --pshare=$APPORT --mport=$AMPORT
fi
#---------------------------------------------------------------
#  Part 7: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#---------------------------------------------------------------
# Part 8: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_shoreside.moos
kill -- -$$
