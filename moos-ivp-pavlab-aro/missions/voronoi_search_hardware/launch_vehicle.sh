#!/bin/bash -e
#-------------------------------------------------------------- 
#   Script: launch_vehicle.sh    
#  Mission: ufld_saxis
#   Author: Michael Benjamin   
#   LastEd: January 2021
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#-------------------------------------------------------------- 
#  Part 2: Set Global Variables
#-------------------------------------------------------------- 
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
VERBOSE=""
AUTO_LAUNCHED="no"
CMD_ARGS=""
SIM="plug_uSimMarineX.moos"
IP_ADDR="localhost"
MOOS_PORT="9201"
PSHARE_PORT="9201"
TIME_D="250"
SHORE_IP="localhost"
SHORE_PSHARE="9200"
VNAME="who"
DATA_D=""
INDEX="1"
SET_METHOD="center"
START_POS="0,0"  
SPEED="1.0"  
LOG_CLEAN="no"
MAX_SPD="2"
MTASC=""
#REGION="-100,-100:100,-100:100,-300:-100,-300"
REGION="0,-15:150,-15:150,-165:0,-165"
ORIGIN="plug_origin_warp1.moos"
SEARCH_M="ROTATE"
SIMULATION="false"
AR=2
CR=1
SPIN_RAD=120
VEC_NAME="SEARCH"
#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS] [time_warp]                              "
	echo "                                                        "
	echo "Options:                                                "
	echo "  --help, -h                                            " 
	echo "    Print this help message and exit                    "
	echo "  --verbose, -v                                         " 
	echo "    Increase verbosity                                  "
	echo "  --just_make, -j                                       " 
	echo "    Just make the targ*.moos and targ*.bhv file and exit"
	echo "  --region=Polygon                                      " 
	echo "    Set the polygon cover region                        "
	echo "  --vname=VNAME                                         " 
	echo "    Set the vheicle name (and MOOS community name)      "
	echo "  --index=INDEX                                         " 
	echo "    Set index for choosing MOOSDB and share listen port "
	echo "  --shoreip=IPADDR                                      " 
	echo "    The IP address where to which the vehicle will try  "
	echo "    to connect (via uFldNodeBroker)                     "
	echo "  --shore_pshare=PORT                                   " 
	echo "    Port on which shoreside pShare is listening         "
	echo "  --startpos=X,Y                                        " 
	echo "    A start position typically chosen by the script that"
	echo "    is launching this script (to ensure separation)     "
	echo "  --speed=meters/sec                                    " 
	echo "    The speed use for transiting/loitering              "
	echo "  --maxspd=meters/sec                                   " 
	echo "    The max speed capability of the vehicle, passed to  "
	echo "    uSimMarine for setting its thrust map               "
        echo "  --auto, -a                                            "
        echo "     Auto-launched by a script.                         "
        echo "     Will not launch uMAC as the final step.            "
        echo "  --mtasc, -m                                           "
        echo "     Mission is being remotely launched (e.g. MTASC)    "
	exit 0;
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes"
    elif [ "${ARGI}" = "--logclean" -o "${ARGI}" = "-l" ]; then
	LOG_CLEAN="yes"
    elif [ "${ARGI}" = "--mtasc" -o "${ARGI}" = "-m" ]; then
	MTASC="yes"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI}" = "--abe" -o "${ARGI}" = "-A" ]; then
        VNAME="abe"
	INDEX=14
    elif [ "${ARGI}" = "--ben" -o "${ARGI}" = "-B" ]; then
        VNAME="ben"
	INDEX=15
    elif [ "${ARGI}" = "--cal" -o "${ARGI}" = "-C" ]; then
        VNAME="cal"
	INDEX=16
    elif [ "${ARGI}" = "--deb" -o "${ARGI}" = "-D" ]; then
        VNAME="deb"
	INDEX=17
    elif [ "${ARGI}" = "--eve" -o "${ARGI}" = "-E" ]; then
        VNAME="eve"
	INDEX=18
    elif [ "${ARGI}" = "--fin" -o "${ARGI}" = "-F" ]; then
        VNAME="fin"
	INDEX=19
    elif [ "${ARGI}" = "--max" -o "${ARGI}" = "-M" ]; then
        VNAME="max"
	INDEX=20
    elif [ "${ARGI}" = "--ned" -o "${ARGI}" = "-N" ]; then
        VNAME="ned"
	INDEX=21
    elif [ "${ARGI}" = "--oak" -o "${ARGI}" = "-O" ]; then
        VNAME="oak"
    INDEX=22
    elif [ "${ARGI}" = "-MM" ]; then
        VNAME="cpr-m300-0020"
    INDEX=22
    elif [ "${ARGI}" = "-NN" ]; then
        VNAME="cpr-m300-0021"
    INDEX=15
    elif [ "${ARGI}" = "-OO" ]; then
        VNAME="cpr-m300-0022"
    INDEX=16
    elif [ "${ARGI:0:8}" = "--timed=" ]; then
        TIME_D="${ARGI#--timed=*}"
    elif [ "${ARGI:0:8}" = "--datad=" ]; then
        DATA_D="${ARGI#--datad=*}"
    elif [ "${ARGI:0:9}" = "--region=" ]; then
        REGION="${ARGI#--region=*}"
    elif [ "${ARGI:0:7}" = "--simx=" ]; then
        SIM="${ARGI#--simx=*}"
    elif [ "${ARGI:0:5}" = "--sim" ]; then
        SIMULATION="true"
    elif [ "${ARGI:0:7}" = "--capa=" ]; then
        AR="${ARGI#--capa=*}"
    elif [ "${ARGI:0:7}" = "--capr=" ]; then
        CR="${ARGI#--capr=*}"
    elif [ "${ARGI:0:7}" = "--spin=" ]; then
        SPIN_RAD="${ARGI#--spin=*}"
    elif [ "${ARGI:0:8}" = "--index=" ]; then
        INDEX="${ARGI#--index=*}"
    elif [ "${ARGI:0:8}" = "--smode=" ]; then
        SEARCH_M="${ARGI#--smode=*}"
    elif [ "${ARGI:0:8}" = "--start=" ]; then
        START_POS="${ARGI#--start=*}"
    elif [ "${ARGI:0:10}" = "--shoreip=" ]; then
        SHORE_IP="${ARGI#--shoreip=*}"
    elif [ "${ARGI:0:10}" = "--smethod=" ]; then
        SET_METHOD="${ARGI#--smethod=*}"
    elif [ "${ARGI:0:15}" = "--shore_pshare=" ]; then
        SHORE_PSHARE="${ARGI#--shore_pshare=*}"
    elif [ "${ARGI:0:8}" = "--speed=" ]; then
        SPEED="${ARGI#--speed=*}"
    elif [ "${ARGI:0:9}" = "--maxspd=" ]; then
        MAX_SPD="${ARGI#--maxspd=*}"
    elif [ "${ARGI:0:9}" = "--origin=" ]; then
        ORIGIN="${ARGI#--origin=*}"
    elif [ "${ARGI:0:9}" = "--vector=" ]; then
        VEC_NAME="${ARGI#--vector=*}"
    elif [ "${ARGI:0:7}" = "--drad=" ]; then
        DET_RAD="${ARGI#--drad=*}"
    else 
	echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
	exit 1
    fi
done

if [ -z $VNAME ]; then
    if [ "${XMODE}" = "SIM" ]; then
	VNAME="abe"
    else
	echo "No in-water vehicle selected. Exit Code 2."
	exit 2
    fi
fi

MOOS_PORT=`expr $INDEX + 9000`
PSHARE_PORT=`expr $INDEX + 9200`
FSEAT_IP="192.168.$INDEX.1"
IP_ADDR="192.168.$INDEX.100"
sss=3
start_plus=$(expr $INDEX*$sss | bc)
START_POS="$start_plus,-5"
#---------------------------------------------------------------
#  Part 3: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" ]; then 
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]"
    echo "TIME_WARP =     [${TIME_WARP}]"
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "MOOS_PORT =     [${MOOS_PORT}]"
    echo "PSHARE_PORT =   [${PSHARE_PORT}]"
    echo "IP_ADDR =       [${IP_ADDR}]"
    echo "----------------------------------"
    echo "SHORE_IP =      [${SHORE_IP}]"
    echo "SHORE_PSHARE =  [${SHORE_PSHARE}]"
    echo "VNAME =         [${VNAME}]"
    echo "INDEX =         [${INDEX}]"
    echo "----------------------------------"
    echo "MAX_SPD =       [${MAX_SPD}]"
    echo "START_POS =     [${START_POS}]"
    echo "START_HDG =     [${START_HDG}]"
    echo "START_SPD =     [${START_SPD}]"
    echo "----------------------------------"
    echo "VDEST_POS =     [${VDEST_POS}]"
    echo "VROLE =         [${VROLE}]"
    echo "WORM_HOLE =     [${WORM_HOLE}]"
    echo -n "Hit any key to continue with launching"
    read ANSWER
fi

#-------------------------------------------------------
#  Part 4: If Log clean before launch, do it now. 
#          In MTASC missions, remote cleaning is essential.
#-------------------------------------------------------
if [ "$LOG_CLEAN" = "yes" ]; then
    vecho "Cleaning local Log Files"
    rm -rf LOG* XLOG* MOOSLog*
fi
INDEX=1
if [ "${VNAME}" = "a" ]; then
    FSEAT_IP="192.168.14.1"
    IP_ADDR="192.168.14.100"
    INDEX=1
    START_POS="5,-5"
fi
if [ "${VNAME}" = "b" ]; then
    FSEAT_IP="192.168.15.1"
    IP_ADDR="192.168.15.100"
    INDEX=2
    START_POS="10,-5"
fi
if [ "${VNAME}" = "c" ]; then
    FSEAT_IP="192.168.16.1"
    IP_ADDR="192.168.16.100"
    INDEX=3
    START_POS="15,-5"
fi
if [ "${VNAME}" = "d" ]; then
    FSEAT_IP="192.168.17.1"
    IP_ADDR="192.168.17.100"
    INDEX=4
    START_POS="20,-5"
fi
if [ "${VNAME}" = "e" ]; then
    FSEAT_IP="192.168.18.1"
    IP_ADDR="192.168.18.100"
    INDEX=5
    START_POS="25,-5"
fi
if [ "${VNAME}" = "f" ]; then
    FSEAT_IP="192.168.19.1"
    IP_ADDR="192.168.19.100"
    INDEX=6
    START_POS="30,-5"
fi
if [ "${VNAME}" = "g" ]; then
    FSEAT_IP="192.168.20.1"
    IP_ADDR="192.168.20.100"
    INDEX=7
    START_POS="35,-5"
fi
if [ "${VNAME}" = "h" ]; then
    FSEAT_IP="192.168.21.1"
    IP_ADDR="192.168.21.100"
    INDEX=8
    START_POS="40,-5"
fi
if [ "${VNAME}" = "i" ]; then
    FSEAT_IP="192.168.22.1"
    IP_ADDR="192.168.22.100"
    INDEX=9
    START_POS="45,-5"
fi
if [ "${MOOS_PORT}" = "" ]; then
    MOOS_PORT=`expr $INDEX + 9000`
fi

if [ "${PSHARE_PORT}" = "" ]; then
    PSHARE_PORT=`expr $INDEX + 9200`
fi
#-------------------------------------------------------
#  Part 5: Create the .moos and .bhv files. 
#-------------------------------------------------------

# interactive nsplug only when not being remotely launched.
NSARGS="--force"
if [ "${MTASC}" = "" ]; then
    NSARGS+=" --interactive"
fi
if [ "${SIMULATION}" = "true" ]; then
    START_POS="45,-5"
nsplug meta_vehicle.moos targ_$VNAME.moos $NSARGS WARP=$TIME_WARP  \
       VNAME=$VNAME           START_POS=$START_POS                 \
       MOOS_PORT=$MOOS_PORT   SHARE_LISTEN=$LPORT                  \
       SHORE_IP=$SHORE_IP     COVER_REGION=$REGION                 \
       MAX_SPD=$MAX_SPD       SHORE_PSHARE=$SHORE_PSHARE           \
       IP_ADDR=$IP_ADDR       PSHARE_PORT=$PSHARE_PORT            \
       DATA_DIRECTORY="$DATA_D"  ORIGIN=$ORIGIN                   \
       SPEED="speed=$SPEED"    VECTOR=$VEC_NAME                     \
       SIMX=$SIM               TIME_DELAY=$TIME_D      \
       GOT_SIM=""            GOT_HARDWARE="//"

nsplug meta_vehicle.bhv targ_$VNAME.bhv $NSARGS   \
       VNAME=$VNAME         START_POS=$START_POS  \
       SPEED=$SPEED         COVER_REGION=$REGION  \
       AR=$AR               CR=$CR                \
       SPIN=$SPIN_RAD       SEARCH_MODE=$SEARCH_M  \
       RAN_NUM_GEN=$MOOS_PORT  SET_METHOD=$SET_METHOD
fi
if [ "${SIMULATION}" = "false" ]; then
SIM="plug_im300.moos"

nsplug meta_vehicle.moos targ_$VNAME.moos $NSARGS WARP=$TIME_WARP  \
       VNAME=$VNAME           START_POS=$START_POS                 \
       MOOS_PORT=$MOOS_PORT   SHARE_LISTEN=$LPORT                  \
       SHORE_IP=$SHORE_IP     COVER_REGION=$REGION                 \
       MAX_SPD=$MAX_SPD       SHORE_PSHARE=$SHORE_PSHARE           \
       IP_ADDR=$IP_ADDR       PSHARE_PORT=$PSHARE_PORT            \
       DATA_DIRECTORY="$DATA_D"  ORIGIN=$ORIGIN                   \
       SPEED="speed=$SPEED"    VECTOR=$VEC_NAME                     \
       SIMX=$SIM               TIME_DELAY=$TIME_D                  \
       GOT_SIM="//"            GOT_HARDWARE=""                    \
       FSEAT_IP=$FSEAT_IP

nsplug meta_vehicle.bhv targ_$VNAME.bhv $NSARGS   \
       VNAME=$VNAME         START_POS=$START_POS  \
       SPEED=$SPEED         COVER_REGION=$REGION  \
       AR=$AR               CR=$CR                \
       SPIN=$SPIN_RAD       SEARCH_MODE=$SEARCH_M  \
       RAN_NUM_GEN=$MOOS_PORT  SET_METHOD=$SET_METHOD
fi
if [ "${JUST_MAKE}" = "yes" ]; then
    vecho "JUST_MAKE is enabled - exiting now." 
    exit 0
fi

#-------------------------------------------------------
#  Part 6: Launch the vehicle mission
#-------------------------------------------------------
vecho "Launching $VNAME MOOS Community. WARP=" $TIME_WARP
pAntler targ_$VNAME.moos >& /dev/null &
vecho "Done Launching $VNAME MOOS Community"

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

