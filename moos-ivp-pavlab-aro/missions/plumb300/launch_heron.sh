#!/bin/bash -e
#--------------------------------------------------------------
#   Script: launch_heron.sh                              
#   Author: Michael Benjamin                                 
#     Date: April 2020                                     
#--------------------------------------------------------------

#----------------------------------------------------------
#  Part 1: Set Exit actions and declare global var defaults
#----------------------------------------------------------
trap "kill -- -$$" EXIT SIGTERM SIGHUP SIGINT SIGKILL
IP_ADDR="127.0.0.1"
MPORT="9001"
PORT="29500"
COMMS_TYPE="server"

#-------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	echo "launch_heron.sh [SWITCHES]                    "
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
	echo "$ ./launch_heron.sh -s --port=29555           "
	exit 0
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="yes"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then
        MPORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:7}" = "--port=" ]; then
        PORT="${ARGI#--port=*}"
    elif [ "${ARGI}" = "--client" -o "${ARGI}" = "-c" ]; then
        COMMS_TYPE="client"
    elif [ "${ARGI}" = "--server" -o "${ARGI}" = "-s" ]; then
        COMMS_TYPE="server"
    else 
	echo "Bad Argument: " $ARGI
	exit 0
    fi
done

#-------------------------------------------------------
#  Part 3: Create the .moos file
#-------------------------------------------------------
nsplug  meta_heron.moos  targ_heron.moos -f         \
	IP_ADDR=$IP_ADDR MPORT=$MPORT PORT=$PORT    \
	COMMS_TYPE=$COMMS_TYPE

if [ "${JUST_MAKE}" = "yes" ] ; then
    exit 0
fi

#-------------------------------------------------------
#  Part 4: Launch
#-------------------------------------------------------
echo "Launching heron MOOS Community"
pAntler targ_heron.moos >& /dev/null &

uMAC targ_heron.moos --proc=uSimHeron
