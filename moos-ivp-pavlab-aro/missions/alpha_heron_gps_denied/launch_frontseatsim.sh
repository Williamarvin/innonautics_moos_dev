#!/bin/bash -e
#--------------------------------------------------------------
#   Script: launch_frontseatsim.sh                              
#   Author: Michael Benjamin                                 
#     Date: April 2020
#   Modified by Supun Randeni on 08/06/2021                                       
#--------------------------------------------------------------

#----------------------------------------------------------
#  Part 1: Set Exit actions and declare global var defaults
#----------------------------------------------------------

TIME_WARP=1
AUTO_LAUNCHED="no"

IP_ADDR="127.0.0.1"
MOOS_PORT="9080"
FRONTSEAT_PORT="29500"

VNAME=""
INDEX="1"

START_POS="0,0,180"
COMMS_TYPE="server"


REGION="pavlab"

#-------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	echo "launch_frontseatsim.sh [SWITCHES]                    "
	echo "  --help, -h                                  " 
	echo "    Display this help message                 " 
	echo "  --just_make, -j                             "
        echo "    Just make targ files, but do not launch   "
        echo "  --port=<port>                               "
        echo "    Port number of the socket (29500 default) "
        echo "  --client, -c                                "
        echo "    Launch as a client (Client is default)    "
        echo "  --server, -s                                "
        echo "    Launch as a server                        "
        echo "  --ip=<ipaddr>                               " 
        echo "    If a client, set IP address of target     "
        echo "    server. (127.0.0.1 is default)            "
	echo "                                              "
	echo "Example: (both on same machine)               "
	echo "$ ./launch_frontseatsim.sh -s --port=29555           "
	exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--start=" ]; then
        START_POS="${ARGI#--start=*}"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then
        MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:7}" = "--port=" ]; then
        FRONTSEAT_PORT="${ARGI#--port=*}"
    elif [ "${ARGI}" = "--client" -o "${ARGI}" = "-c" ]; then
        COMMS_TYPE="client"
    elif [ "${ARGI}" = "--server" -o "${ARGI}" = "-s" ]; then
        COMMS_TYPE="server"

    elif [ "${ARGI}" = "--evan" -o "${ARGI}" = "-E" ]; then
        VNAME="evan"
    elif [ "${ARGI}" = "--felix" -o "${ARGI}" = "-F" ]; then
        VNAME="felix"
    elif [ "${ARGI}" = "--gus" -o "${ARGI}" = "-G" ]; then
        VNAME="gus"
    elif [ "${ARGI}" = "--hal" -o "${ARGI}" = "-H" ]; then
        VNAME="hal"
    elif [ "${ARGI}" = "--ida" -o "${ARGI}" = "-I" ]; then
        VNAME="ida"
    elif [ "${ARGI}" = "--jing" -o "${ARGI}" = "-J" ]; then
        VNAME="jing"
    elif [ "${ARGI}" = "--kirk" -o "${ARGI}" = "-K" ]; then
        VNAME="kirk"
    elif [ "${ARGI}" = "--luke" -o "${ARGI}" = "-L" ]; then
        VNAME="luke"

    else 
	echo "Bad Argument: " $ARGI
	exit 0
    fi
done

#--------------------------------------------------------------
#  Part 3: Check for VNAME. Use INDEX for Other Settings
#--------------------------------------------------------------

if [ "${VNAME}" = "evan" ]; then
    INDEX=5
elif [ "${VNAME}" = "felix" ]; then
    INDEX=6
elif [ "${VNAME}" = "gus" ]; then
    INDEX=7
elif [ "${VNAME}" = "hal" ]; then
    INDEX=8
elif [ "${VNAME}" = "ida" ]; then
    INDEX=9
elif [ "${VNAME}" = "jing" ]; then
    INDEX=10
elif [ "${VNAME}" = "kirk" ]; then
    INDEX=11
elif [ "${VNAME}" = "luke" ]; then
    INDEX=12
fi

MOOS_PORT=`expr $INDEX + 9080`
FRONTSEAT_PORT=`expr $INDEX + 29500`

#-------------------------------------------------------
#  Part 3: Create the .moos file
#-------------------------------------------------------

nsplug  meta_frontseatsim.moos  targ_frontseatsim.moos -f WARP=$TIME_WARP \
	IP_ADDR=$IP_ADDR MOOS_PORT=$MOOS_PORT FRONTSEAT_PORT=$FRONTSEAT_PORT    \
    START_POS=$START_POS COMMS_TYPE=$COMMS_TYPE REGION=$REGION    \ 	

if [ "${JUST_MAKE}" = "yes" ] ; then
    exit 0
fi

#-------------------------------------------------------
#  Part 4: Launch
#-------------------------------------------------------
echo "Launching frontseatsim MOOS Community"
pAntler targ_frontseatsim.moos >& /dev/null &
echo "Done Launching frontseatsim MOOS Community"

#---------------------------------------------------------------
#  Part 5: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

uMAC targ_frontseatsim.moos
kill -- -$$
