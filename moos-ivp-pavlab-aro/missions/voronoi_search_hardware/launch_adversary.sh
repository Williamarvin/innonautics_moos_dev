#!/bin/bash 
#-------------------------------------------------------
#   Script: launch_vehicle.sh                       
#  Mission: lab_10_baseline
#-------------------------------------------------------
#  Part 1: Set global var defaults
#----------------------------------------------------------
TIME_WARP=1
JUST_MAKE="no"
HOSTNAME=$(hostname -s)
VNAME="ADVERSARY"
MINSPD=1.2
MAXSPD=1.3
IP_ADDR="localhost"
MOOS_PORT="9011"
PSHARE_PORT="9211"
SHORE_PSHARE="9200"
SHORE_IP="localhost"
GUI="yes"
CONF="no"
AUTO="no"
SPEED=1.0

#-------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "launch_vehicle.sh [SWITCHES]                     "
	echo "  --help, -h                                     " 
	echo "    Display this help message                    "
	echo "  --just_make, -j                                " 
	echo "    Just make targ files, but do not launch      "
	echo "                                                 "
	echo "  --vname=<vname>                                " 
	echo "    Name of the vehicle being launched           " 
	echo "                                                 "
	echo "  --shore=<ipaddr>                               "
	echo "  --shoreip=<ipaddr>                             "
	echo "    IP address of the shoreside                  "
	echo "                                                 "
	echo "  --mport=<port>(9001)                           "
	echo "    Port number of this vehicle's MOOSDB port    "
	echo "                                                 "
	echo "  --pshare=<port>(9201)                          " 
	echo "    Port number of this vehicle's pShare port    "
	echo "                                                 "
	echo "  --ip=<ipaddr>                                  " 
	echo "    Force pHostInfo to use this IP Address       "
	echo "                                                 "
	echo "  --nogui                                        " 
	echo "    Do not launch pMarineViewer GUI with vehicle "
	echo "  --auto,-a                                      " 
	echo "    Exit after launching. Do not launch uMAC     "
	echo "  --nc,-nc                                       " 
	echo "    No confirmation before launching             "
	exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--nc" -o "${ARGI}" = "-nc" ]; then
	CONF="no"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
	AUTO="yes"
    elif [ "${ARGI:0:8}" = "--shore=" ]; then
	SHORE_IP="${ARGI#--shore=*}"
	elif [ "${ARGI:0:9}" = "--origin=" ]; then
        ORIGIN="${ARGI#--origin=*}"
	elif [ "${ARGI:0:9}" = "--choice=" ]; then
        CHOICE="${ARGI#--choice=*}"
    elif [ "${ARGI:0:10}" = "--shoreip=" ]; then
	SHORE_IP="${ARGI#--shoreip=*}"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then
	MOOS_PORT="${ARGI#--mport=*}"
	elif [ "${ARGI:0:10}" = "--adspeed=" ]; then
	SPEED="${ARGI#--adspeed=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
	PSHARE_PORT="${ARGI#--pshare=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
	VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI}" = "--nogui" ]; then
	GUI="no"
    else
	echo "launch_ad.sh: Bad Arg: $ARGI Exit Code 1"
	exit 1
    fi
done

if [ "${CONF}" = "yes" ]; then 
    echo "PSHARE_PORT =  [${PSHARE_PORT}]"
    echo "MOOS_PORT =    [${MOOS_PORT}]"
    echo "IP_ADDR =      [${IP_ADDR}]"
    echo "VNAME =        [${VNAME}]"
    echo "SHORE_IP =     [${SHORE_IP}]"
    echo "SHORE_PSHARE = [${SHORE_PSHARE}]"
    echo "TIME_WARP =    [${TIME_WARP}]"
    echo "UMAC =         [${UMAC}]"
    echo -n "Hit any key to continue with launching"
    read ANSWER
fi

#-------------------------------------------------------
#  Part 3: Create the .moos and .bhv files. 
#-------------------------------------------------------
if [ "${CHOICE}" = "sp" ]; then 
	if [ "${RANDSTART}" = "true" -o  ! -f "vpositions1.txt" ]; then
		./pickpos.sh --iter="start"
	fi
	if [ "${RANDSTART}" = "true" -o  ! -f "vpositions2.txt" ]; then
		./pickpos.sh --iter="end"
	fi

	VEHPOS=(`cat vpositions1.txt`)
	ENDPOS=(`cat vpositions2.txt`)
fi
if [ "${CHOICE}" = "saxis" ]; then 
	if [ "${RANDSTART}" = "true" -o  ! -f "vpositionss1.txt" ]; then
		./pickpossaxis.sh --iter="start"
	fi
	if [ "${RANDSTART}" = "true" -o  ! -f "vpositionss2.txt" ]; then
		./pickpossaxis.sh --iter="end"
	fi

	VEHPOS=(`cat vpositionss1.txt`)
	ENDPOS=(`cat vpositionss2.txt`)
fi
START_POS=${VEHPOS[0]}
END=${ENDPOS[0]}
VNAME="ADVERSARY"
SPEED="${SPEED#speed=*}"



FULL_VNAME=$VNAME
WPT_COLOR="light_blue"

LOITER_POS="$END"
if [ "${CHOICE}" = "sp" ]; then
	SIMX="plug_uSimX1.moos"
	END_POS="300,-100"
	#NEW_P="150"
	#X_OFF="75"
	#MIN="-300"
	#MAX="-100"
	  	NEW_P=150
		X_OFF=75
		MIN=-165
		MAX=-15
		START_POS="x=-10.1,y=-30.4,heading=97"
fi
if [ "${CHOICE}" = "saxis" ]; then
	SIMX="plug_uSimX2.moos"
	END_POS="9000,2000"
	NEW_P="7200"
	X_OFF="1000"
	MIN="0"
	MAX="2000"
fi
nsplug meta_adversary.moos targ_$FULL_VNAME.moos -f WARP=$TIME_WARP  \
    PSHARE_PORT=$PSHARE_PORT     VNAME=$FULL_VNAME                 \
    START_POS=$START_POS         SHORE_IP=$SHORE_IP                \
    SHORE_PSHARE=$SHORE_PSHARE   VPORT=$MOOS_PORT                  \
    IP_ADDR=$IP_ADDR         	 ORIGIN=$ORIGIN                    \
	SIMX=$SIMX                   NEW_P=$NEW_P                      \
	X_OFF=$X_OFF                 MIN=$MIN                          \
	MAX=$MAX                     SPEED1=$SPEED

nsplug meta_adversary.bhv targ_$FULL_VNAME.bhv -f VNAME=$FULL_VNAME  \
    START_POS=$START_POS LOITER_POS=$LOITER_POS                    \
	SPEED_ENT=$SPEED     END_POS=$END_POS
   
#-------------------------------------------------------
#  Part 4: Launch the processes
#-------------------------------------------------------
echo "Launching $VNAME MOOS Community. WARP=" $TIME_WARP
pAntler targ_$VNAME.moos >& /dev/null &
echo "Done Launching $VNAME MOOS Community"

#-------------------------------------------------------
#  Part 5: If auto launched from a script, we're done
#-------------------------------------------------------


