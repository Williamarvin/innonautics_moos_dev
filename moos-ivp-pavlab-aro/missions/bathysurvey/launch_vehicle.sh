#!/bin/bash
#-------------------------------------------------------------- 
#   Script: launch_vehicle.sh                                    
#  Mission: alpha_heron
#   Author: Michael Benjamin  
#     Date: June 2021
#--------------------------------------------------------------
#  Part 1: Declare global var defaults
#--------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE="no"
VERBOSE="no"
CONFIRM="yes"
AUTO_LAUNCHED="no"
CMD_ARGS=""

IP_ADDR="localhost"
MOOS_PORT="9001"
PSHARE_PORT="9201"

SHORE_IP="192.168.1.241"
SHORE_PSHARE="9200"
VNAME="ida"
ORIENTATION="north-south"
ORDER="normal"
INDEX="1"
XMODE="M300"
VMODE="HERON"

REGION="pavlab"
START_POS="0,0,180"
SPEED="1.2"
RETURN_POS="5,0"
MAXSPD="2.4"

GRID_START="-70,-55"
EXPLORE_END="0,0"

GRIDSIZE="SMALL"
PATHMODE="OTHER"
REWARD="0"

VNUM="2"
V_N="0"

#X_1="400"
#Y_1="-115"
#X_2="70"
#Y_2="-235"
#X_3="150"
#Y_3="-490"
#X_2="-80"
#Y_2="-290"
#X_3="10"
#Y_3="-530"
#X_4="480"
#Y_4="-370"

X_1="-75"
Y_1="-50"
X_2="-75"
Y_2="-220"
X_3="185"
Y_3="-220"
X_4="185"
Y_4="-50"


CELLSIZE="10"

GRIDFILE="1"
MIRROR="false"

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
	echo "  --noconfirm, -nc                               " 
	echo "    No confirmation before launching             "
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
	echo "  --shore_pshare=<9300>                          " 
	echo "    Port on which shoreside pShare is listening  "
	echo "  --vname=<abe>                                  " 
	echo "    Name of the vehicle being launched           " 
	echo "  --index=<1>                                    " 
	echo "    Index for setting MOOSDB and pShare ports    "
	echo "                                                 "
	echo "  --start=<X,Y>     (default is 0,0)             " 
	echo "    Start position chosen by script launching    "
	echo "    this script (to ensure separation)           "
	echo "  --speed=meters/sec                             " 
	echo "    The speed use for transiting/loitering       "
	echo "  --maxspd=meters/sec                            " 
	echo "    Max speed of vehicle (for sim and in-field)  "
	echo "                                                 "
	echo "  --abe,  -A  : abe vehicle.                     "
        echo "  --ben,  -B  : ben vehicle.                     "
        echo "  --cal,  -C  : cal vehicle.                     "
        echo "  --deb,  -D  : deb vehicle.                     "
	echo "  --eve,  -E  : eve vehicle.                     "
        echo "  --fin,  -F  : fin vehicle.                     "
	echo "  --max,  -M  : max vehicle.                     "
        echo "  --ned,  -N  : ned vehicle.                     "
	echo "  --oak,  -O  : oak vehicle.                     "
	echo "                                                 "
	echo "  --m200, -2   : Use iM200 interface             "
	echo "  --m300, -3   : Use iM300 interface             "
	echo "  --sim,  -s   : Simulation                      "
	echo "                                                 "
	echo "  --pablo      : Running on PABLO no Heron       "
	echo "                                                 "
	echo "  -north-south, -NS  : north-south lawnmower     "
        echo "  -east-west,   -EW  : east-west lawnmower       "
        echo "  -reverse,     -RV  : reverse waypoint order    "
	echo "                                                 "
        echo "  --smallgrid,  -SG  : small grid op region      "
        echo "  --fullgrid,   -FG  : full grid op region       "
	echo "                                                 "
        echo "  --lawnmode,   -LM  : run all lawnmowers        "
        echo "  --greedymode, -GM  : run all greedy search     "
	echo "  --othermode,  -OM  : Run PBACS                 "
	echo "                                                 "
	echo "  --start=0,0        : start position            "
        echo "  --gridstart=0,0    : first waypoint in grid    "
	echo "  --exploreend=0,0   : endpoint of explore phase "
	echo "                       (only used for PBACS)     "
	echo "                                                 "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI}" = "--noconfirm" -o "${ARGI}" = "-nc" ]; then
	CONFIRM="no"
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
    elif [ "${ARGI:0:15}" = "--shore_pshare=" ]; then
        SHORE_PSHARE="${ARGI#--shore_pshare=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--index=" ]; then
        INDEX="${ARGI#--index=*}"

    elif [ "${ARGI:0:8}" = "--start=" ]; then
        START_POS="${ARGI#--start=*}"
    elif [ "${ARGI:0:12}" = "--gridstart=" ]; then
        GRID_START="${ARGI#--gridstart=*}"
    elif [ "${ARGI:0:13}" = "--exploreend=" ]; then
        EXPLORE_END="${ARGI#--exploreend=*}"
	
    elif [ "${ARGI:0:8}" = "--speed=" ]; then
        SPEED="${ARGI#--speed=*}"
    elif [ "${ARGI:0:9}" = "--maxspd=" ]; then
        MAXSPD="${ARGI#--maxspd=*}"


    elif [ "${ARGI}" = "--abe" -o "${ARGI}" = "-A" ]; then
        VNAME="abe"
    elif [ "${ARGI}" = "--ben" -o "${ARGI}" = "-B" ]; then
        VNAME="ben"
    elif [ "${ARGI}" = "--cal" -o "${ARGI}" = "-C" ]; then
        VNAME="cal"
    elif [ "${ARGI}" = "--deb" -o "${ARGI}" = "-D" ]; then
        VNAME="deb"
    elif [ "${ARGI}" = "--eve" -o "${ARGI}" = "-E" ]; then
        VNAME="eve"
    elif [ "${ARGI}" = "--fin" -o "${ARGI}" = "-F" ]; then
        VNAME="fin"
    elif [ "${ARGI}" = "--max" -o "${ARGI}" = "-M" ]; then
        VNAME="max"
    elif [ "${ARGI}" = "--ned" -o "${ARGI}" = "-N" ]; then
        VNAME="ned"
    elif [ "${ARGI}" = "--oak" -o "${ARGI}" = "-O" ]; then
        VNAME="oak"

    elif [ "${ARGI}" = "--sim" -o "${ARGI}" = "-s" ]; then
        XMODE="SIM"
        echo "Simulation mode ON."

    elif [ "${ARGI}" = "--m300" -o "${ARGI}" = "-3" ]; then
	XMODE="M300"
    elif [ "${ARGI}" = "--m200" -o "${ARGI}" = "-2" ]; then
	XMODE="M200"

    elif [ "${ARGI}" = "--pablo" ]; then
	VMODE="PABLO"

    elif [ "${ARGI}" = "--north-south" -o "${ARGI}" = "-NS" ]; then
	ORIENTATION="north-south"
    elif [ "${ARGI}" = "--east-west" -o "${ARGI}" = "-EW" ]; then
	ORIENTATION="east-west"
    elif [ "${ARGI}" = "--reverse" -o "${ARGI}" = "-RV" ]; then
	ORDER="reverse"


    elif [ "${ARGI}" = "--smallgrid" -o "${ARGI}" = "-SG" ]; then
	GRIDSIZE="SMALL"
    elif [ "${ARGI}" = "--newgrid" -o "${ARGI}" = "-NG" ]; then
	GRIDSIZE="NEW"
    elif [ "${ARGI}" = "--fullgrid" -o "${ARGI}" = "-FG" ]; then
	GRIDSIZE="FULL"


    elif [ "${ARGI}" = "--lawnmode" -o "${ARGI}" = "-LM" ]; then
	PATHMODE="LAWN"
    elif [ "${ARGI}" = "--greedymode" -o "${ARGI}" = "-GM" ]; then
	PATHMODE="GREEDY"
    elif [ "${ARGI}" = "--othermode" -o "${ARGI}" = "-OM" ]; then
	PATHMODE="OTHER"

    elif [ "${ARGI}" = "--ucb" ]; then
	REWARD="1"
    elif [ "${ARGI}" = "--mvi" ]; then
	REWARD="0"
			 

    elif [ "${ARGI:0:7}" = "--vnum=" ]; then
        VNUM="${ARGI#--vnum=*}"
    elif [ "${ARGI:0:6}" = "--v_N=" ]; then
        V_N="${ARGI#--v_N=*}"

    elif [ "${ARGI:0:5}" = "--x1=" ]; then
        X_1="${ARGI#--x1=*}"
    elif [ "${ARGI:0:5}" = "--y1=" ]; then
        Y_1="${ARGI#--y1=*}"
    elif [ "${ARGI:0:5}" = "--x2=" ]; then
        X_2="${ARGI#--x2=*}"
    elif [ "${ARGI:0:5}" = "--y2=" ]; then
        Y_2="${ARGI#--y2=*}"
    elif [ "${ARGI:0:5}" = "--x3=" ]; then
        X_3="${ARGI#--x3=*}"
    elif [ "${ARGI:0:5}" = "--y3=" ]; then
        Y_3="${ARGI#--y3=*}"
    elif [ "${ARGI:0:5}" = "--x4=" ]; then
        X_4="${ARGI#--x4=*}"
    elif [ "${ARGI:0:5}" = "--y4=" ]; then
        Y_4="${ARGI#--y4=*}"
    elif [ "${ARGI:0:11}" = "--cellsize=" ]; then
        CELLSIZE="${ARGI#--cellsize=*}"

    elif [ "${ARGI:0:11}" = "--gridfile=" ]; then
        GRIDFILE="${ARGI#--gridfile=*}"
    elif [ "${ARGI}" = "--mirror" ]; then
        MIRROR="true"

    else
	echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
	exit 1
    fi
done

#--------------------------------------------------------------
#  Part 3: Check for VNAME. Use INDEX for Other Settings
#--------------------------------------------------------------
if [ -z $VNAME ]; then
    if [ "${XMODE}" = "SIM" ]; then
	VNAME="abe"
    else
	echo "No in-water vehicle selected. Exit Code 2."
	exit 2
    fi
fi

if [ "${VNAME}" = "abe" ]; then
    INDEX=14
elif [ "${VNAME}" = "ben" ]; then
    INDEX=15
elif [ "${VNAME}" = "cal" ]; then
    INDEX=16
elif [ "${VNAME}" = "deb" ]; then
    INDEX=17
elif [ "${VNAME}" = "eve" ]; then
    INDEX=18
elif [ "${VNAME}" = "fin" ]; then
    INDEX=19
elif [ "${VNAME}" = "max" ]; then
    INDEX=20
elif [ "${VNAME}" = "ned" ]; then
    INDEX=21
elif [ "${VNAME}" = "oak" ]; then
    INDEX=22
fi

if [ "${VMODE}" = "HERON" ]; then
    MOOS_PORT=`expr $INDEX + 9000`
    PSHARE_PORT=`expr $INDEX + 9200`
    FSEAT_IP="192.168.$INDEX.1"
    IP_ADDR="192.168.$INDEX.100"
fi

if [ "${VMODE}" = "PABLO" ]; then
    MOOS_PORT=`expr $INDEX + 9000`
    PSHARE_PORT=`expr $INDEX + 9200`
    FSEAT_IP="192.168.$INDEX.1"
    IP_ADDR="192.168.$INDEX.100"
fi

if [ "${XMODE}" = "SIM" ]; then
    if [ "${VMODE}" = "HERON" ]; then
        IP_ADDR="localhost"
        SHORE_IP="localhost"
    fi
fi


#---------------------------------------------------------------
#  Part 4: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" -o "${CONFIRM}" = "yes" ]; then 
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
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
    echo "FSEAT_IP =      [${FSEAT_IP}]     "
    echo "XMODE =         [${XMODE}]        "
    echo "----------------------------------"
    echo "START_POS =     [${START_POS}]    "
    echo "SPEED =         [${SPEED}]        "
    echo "MAXSPD =        [${MAXSPD}]       "
    echo -n "Hit any key to continue with launching"
    read ANSWER
fi


#--------------------------------------------------------------
#  Part 5: Create the .moos and .bhv files. 
#--------------------------------------------------------------
NSFLAGS="-s -f"
if [ "${AUTO}" = "" ]; then
    NSFLAGS="-i -f"
fi

nsplug meta_vehicle.moos targ_$VNAME.moos $NSFLAGS WARP=$TIME_WARP \
       PSHARE_PORT=$PSHARE_PORT     VNAME=$VNAME               \
       START_POS=$START_POS         SHORE_IP=$SHORE_IP         \
       SHORE_PSHARE=$SHORE_PSHARE   MOOS_PORT=$MOOS_PORT       \
       IP_ADDR=$IP_ADDR             REGION=$REGION             \
       FSEAT_IP=$FSEAT_IP           XMODE=$XMODE               \
       START_POS=$START_POS         MAXSPD=$MAXSPD             \
       GRIDSIZE=$GRIDSIZE           PATHMODE=$PATHMODE         \
       VNUM=$VNUM                   V_N=$V_N                   \
       X_1=$X_1                     Y_1=$Y_1                   \
       X_2=$X_2                     Y_2=$Y_2                   \
       X_3=$X_3                     Y_3=$Y_3                   \
       X_4=$X_4                     Y_4=$Y_4                   \
       CELLSIZE=$CELLSIZE           GRIDFILE=$GRIDFILE         \
       MIRROR=$MIRROR               REWARD=$REWARD

nsplug meta_vehicle.bhv targ_$VNAME.bhv $NSFLAGS VNAME=$VNAME  \
       SPEED=$SPEED                 ORIENTATION=$ORIENTATION   \
       ORDER=$ORDER                 GRID_START=$GRID_START     \
       GRIDSIZE=$GRIDSIZE           PATHMODE=$PATHMODE         \
       EXPLORE_END=$EXPLORE_END     
       
if [ ${JUST_MAKE} = "yes" ] ; then
    echo "Files assembled; nothing launched; exiting per request."
    exit 0
fi


#--------------------------------------------------------------
#  Part 6: Launch the processes
#--------------------------------------------------------------

echo "Launching $VNAME MOOS Community. WARP="$TIME_WARP
pAntler targ_${VNAME}.moos >& /dev/null &
echo "Done Launching $VNAME MOOS Community"

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
