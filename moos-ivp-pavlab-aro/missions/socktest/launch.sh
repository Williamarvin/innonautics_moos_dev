#!/bin/bash -e
#--------------------------------------------------------------
#   Script: launch_sock_comms.sh                              
#   Author: Michael Benjamin                                 
#     Date: April 2020                                     
#--------------------------------------------------------------

#--------------------------------------------------------------
#  Part 1: Set Global variables          
#--------------------------------------------------------------
IP_ADDR="127.0.0.1"
MPORT="9000"
PORT="29500"
COMMS_TYPE="client"
MSG="hello"

#-------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	echo "launch_sock_comms.sh [SWITCHES]               "
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
        echo "  --msg=<string>                              " 
        echo "    Message to send to receiver               " 
        echo "  --ip=<ipaddr>                               " 
        echo "    If a client, set IP address of target     "
        echo "    server. (127.0.0.1 is default)            "
	echo "                                              "
	echo "Example: (both on same machine)               "
	echo "$ ./launch_sock_comms.sh                      "
	echo "$ ./launch_sock_comms.sh -s --mport=9001      "
	echo "                                              "
	echo "Example: (separate machines)                  "
	echo "$ ./launch_sock_comms.sh --ip=1.2.3.4         "
	echo "$ ./launch_sock_comms.sh -s --mport=9001      "
	exit 0
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="yes"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then
        MPORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:7}" = "--port=" ]; then
        PORT="${ARGI#--port=*}"
    elif [ "${ARGI:0:6}" = "--msg=" ]; then
        MSG="${ARGI#--msg=*}"
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
nsplug  meta_sock_comms.moos  targ_sock_comms_$MPORT.moos -f  \
	IP_ADDR=$IP_ADDR MPORT=$MPORT PORT=$PORT              \
	COMMS_TYPE=$COMMS_TYPE MSG=$MSG 

if [ ${JUST_MAKE} = "yes" ] ; then
    exit 0
fi

#-------------------------------------------------------
#  Part 4: Launch
#-------------------------------------------------------
echo "Launching"
pAntler targ_sock_comms_$MPORT.moos >& /dev/null &

uMAC targ_sock_comms_$MPORT.moos --proc=uSockComms

kill -- -$$
