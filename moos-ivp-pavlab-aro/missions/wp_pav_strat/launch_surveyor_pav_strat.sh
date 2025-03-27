#!/bin/bash
#---------------------------------------------------------------
#   Script: launch_surveyor_pav_strat.sh
#  Mission: wp_ctrl_test
#   Author: Tyler Paine + Mike Benjamin
#   LastEd: June 2024
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#---------------------------------------------------------------
#  Part 2: Set global var defaults
#---------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
VERBOSE=""
JUST_MAKE="no"
AUTO_LAUNCHED="no"
CMD_ARGS=""

IP_ADDR="localhost"
MOOS_PORT="9001"
PSHARE_PORT="9201"


SHORE_IP=localhost
SHORE_PSHARE="9300"

TRAIL_RANGE="3"
TRAIL_ANGLE="330"
VTEAM=""
VNAME=""
RNAME=""
VMODEL="M1_8"
START_POS=""
RETURN_POS="x=263,y=109"
RLA_ENABLE="no"

CID=000 #COMP ID
LOGPATH="./"
FLD_ANG=""
NSFLAGS="-f"

START_ACTION="PROTECT"
ENTRY=""

#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    CMD_ARGS+="${ARGI} "
    if [ $ARGI = "--help" -o $ARGI = "-H" -o $ARGI = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                   "
	echo "                                            "
	echo "Options (vehicle name):                     "
	echo "  --scott, -vs, -v1     Vehicle name is Sottt    "
	echo "  --thomas, -vt, -v2    Vehicle name is Thomas   "
	echo "  --ursula, -vu, -v3    Vehicle name is Ursula   "
	echo "  --valhalla, -vv, -v4  Vehicle name is Valhalla "
	echo "  --walter, -vw, -v5    Vehicle name is Walter   "
	echo "  --xavier, -vx, -v6    Vehicle name is Xavier   "
	echo "  --yolanda, -vy, -v7   Vehicle name is Yolanda  "
	echo "  --zach, -vz, -v8      Vehicle name is Zach     "
	echo "                                                 "
	echo "Options (vehicle role):                          "
	echo "  --red_one, -r1   Vehicle role is Red-One  "
	echo "  --red_two, -r2   Vehicle role is Red-Two  "
	echo "  --blue_one, -b1  Vehicle role is Blue-One "
	echo "  --blue_two, -b2  Vehicle role is Blue-Two "
	echo "                                            "
	echo "Options (other):                            "
	echo "  --role,           Startup roles: ATTACK/DEFEND _E/_MED."
	echo "  --sim, -s         Simulation mode."
	echo "  --shore=<localhost>                            "
        echo "    IP address location of shoreside             "
	echo "    Shortcut: --sip same as --shore=192.168.1.199"
	echo "  --ip=<localhost>                               " 
	echo "    Force pHostInfo to use this IP Address       "
	echo "  --start=<X,Y,H>                                " 
	echo "  --cid=            Competition ID (for log file)"
	echo "  --logpath=        Log path                     "
	echo "  --ang=<fld_ang>                                "
	echo "  --just_make, -j   Just make targ files.        "
	echo "  --help, -H,-h     Display this message.        "
	echo "  --verbose,-V,-v   Verbose launch.              "
	exit 0
    elif [ $ARGI = "-vs" -o $ARGI = "-v1" -o $ARGI = "--scott" ]; then
        FSEAT_IP=192.168.1.11;   VNAME="SCOTT";  IP_ADDR=192.168.1.12
    elif [ $ARGI = "-vt" -o $ARGI = "-v2" -o $ARGI = "--thomas" ]; then
        FSEAT_IP=192.168.1.21;   VNAME="THOMAS";  IP_ADDR=192.168.1.22
    elif [ $ARGI = "-vu" -o $ARGI = "-v3" -o $ARGI = "--ursula" ]; then
        FSEAT_IP=192.168.1.31;   VNAME="URSULA";  IP_ADDR=192.168.1.32
    elif [ $ARGI = "-vv" -o $ARGI = "-v4" -o $ARGI = "--valhalla" ]; then
        FSEAT_IP=192.168.1.41;   VNAME="VALHALLA";  IP_ADDR=192.168.1.42
    elif [ $ARGI = "-vw" -o $ARGI = "-v5" -o $ARGI = "--walter" ]; then
        FSEAT_IP=192.168.1.51;   VNAME="WALTER";  IP_ADDR=192.168.1.52
    elif [ $ARGI = "-vx" -o $ARGI = "-v6" -o $ARGI = "--xavier" ]; then
        FSEAT_IP=192.168.1.61;   VNAME="XAVIER";  IP_ADDR=192.168.1.62
    elif [ $ARGI = "-vy" -o $ARGI = "-v7" -o $ARGI = "--yolanda" ]; then
        FSEAT_IP=192.168.1.71;   VNAME="YOLANDA";  IP_ADDR=192.168.1.72
    elif [ $ARGI = "-vy" -o $ARGI = "-v8" -o $ARGI = "--zach" ]; then
        FSEAT_IP=192.168.1.81;   VNAME="ZACH";  IP_ADDR=192.168.1.82

    elif [ "${ARGI}" = "-r1" -o "${ARGI}" = "--red_one" ]; then
        PSHARE_PORT="9311";         VTEAM="red"
        RNAME="red_one";           MOOS_PORT="9011"               
    elif [ "${ARGI}" = "-r2" -o "${ARGI}" = "--red_two" ]; then
        PSHARE_PORT="9312";         VTEAM="red"
        RNAME="red_two";            MOOS_PORT="9012"               
    elif [ "${ARGI}" = "-r3" -o "${ARGI}" = "--red_three" ]; then
        PSHARE_PORT="9313";         VTEAM="red"
        RNAME="red_three";          MOOS_PORT="9013"               
    elif [ "${ARGI}" = "-b1" -o "${ARGI}" = "--blue_one" ]; then
        PSHARE_PORT="9315";         VTEAM="blue"
        RNAME="blue_one";           MOOS_PORT="9015"               
    elif [ "${ARGI}" = "-b2" -o "${ARGI}" = "--blue_two" ]; then
        PSHARE_PORT="9316";         VTEAM="blue"
        RNAME="blue_two";           MOOS_PORT="9016"               
    elif [ "${ARGI}" = "-b3" -o "${ARGI}" = "--blue_three" ]; then
        PSHARE_PORT="9317";         VTEAM="blue"
        RNAME="blue_three";         MOOS_PORT="9017"             

    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ $ARGI = "--just_build" -o $ARGI = "-J" -o $ARGI = "-j" ]; then
        JUST_MAKE="yes"
    elif [ "${ARGI}" = "--sim" -o "${ARGI}" = "-s" ]; then
        SIM="SIM"
    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHORE_IP="${ARGI#--shore=*}"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI}" = "--sip" -o "${ARGI}" = "-sip" ]; then
        SHORE_IP="192.168.1.199"
    elif [ "${ARGI:0:8}" = "--start=" ]; then
        START_POS="${ARGI#--start=*}"
    elif [ "${ARGI:0:7}" = "--role=" ]; then
        START_ACTION="${ARGI#--role=*}"
    elif [ "${ARGI:0:6}" = "--cid=" ]; then
        CID="${ARGI#--cid=*}"
        CID=$(printf "%03d" $CID)
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes" 
    elif [ "${ARGI:0:10}" = "--logpath=" ]; then
        LOGPATH="${ARGI#--logpath=*}"
    elif [ "${ARGI:0:6}" = "--ang=" ]; then
        FLD_ANG="${ARGI}"
    elif [ $ARGI = "--verbose" -o $ARGI = "-V" -o $ARGI = "-v" ]; then
	VERBOSE="yes"
	NSFLAGS+=" -i"
    else
        echo "$ME Bad Arg: [$ARGI]. Exit Code 1"
	exit 1
    fi
done

# give each info file unique name in case jerop.s is run in ~parallel
# when/if launching multiple vehicles in sim successively.
./jerop.sh $FLD_ANG > region_info"_$RNAME".txt
source region_info"_$RNAME".txt


if [ "${VTEAM}" = "red" ]; then
    OPFOR="blue"
    OPFOR_ZONE=$BLUE_ZONE
    HFLD=$HOME_IS_NORTH
elif [ "${VTEAM}" = "blue" ]; then
    OPFOR="red"
    OPFOR_ZONE=$RED_ZONE
    HFLD=$HOME_IS_SOUTH
fi


#---------------------------------------------------------------
#  Part 4: Ensure Mandatory Settings Specified
#---------------------------------------------------------------
if [ "${VNAME}" = "" -o "${RNAME}" = "" ]; then
    echo "VNAME and VROLE must be set. See --help "
    echo "RNAME =         [${RNAME}]              "
    echo "VNAME =         [${VNAME}]              "
    exit 1
fi

#---------------------------------------------------------------
#  Part 5: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" != "" ]; then 
    echo "=================================="
    echo "   launch_surveyor.sh SUMMARY     "
    echo "=================================="
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "JUST_MAKE =     [${JUST_MAKE}]    "
    echo "SIM =           [${SIM}]          "
    echo "NSFLAGS =       [$NSFLAGS]        "
    echo "FLD_ANG =       [${FLD_ANG}]      "
    echo "----------------------------------"
    echo "IP_ADDR =       [${IP_ADDR}]      "
    echo "MOOS_PORT =     [${MOOS_PORT}]    "
    echo "PSHARE_PORT =   [${PSHARE_PORT}]  "
    echo "----------------------------------"
    echo "SHORE_IP =      [${SHORE_IP}]     "
    echo "SHORE_PSHARE =  [${SHORE_PSHARE}] "
    echo "RNAME =         [${RNAME}]        "
    echo "VNAME =         [${VNAME}]        "
    echo "VTEAM =         [${VTEAM}]        "
    echo "----------------------------------"
    echo "FSEAT_IP =      [${FSEAT_IP}]     "
    echo "LOGPATH =       [${LOGPATH}]      "
    echo "----------------------------------"
    echo "REGION =        [${REGION}]       "
    echo "WANG =          [${WANG}]         "
    echo "DANG =          [${DANG}]         "
    echo "RED_ZONE =      [${RED_ZONE}]     "
    echo "BLUE_ZONE =     [${BLUE_ZONE}]    "
    echo -n "Hit any key to continue with launching ${VNAME}"
    read ANSWER
fi

echo "Assembling MOOS file targ_${RNAME}.moos"

#-------------------------------------------------------
#  Part 6: Create the .moos and .bhv files.
#-------------------------------------------------------
nsplug meta_pav_strat.moos targ_${RNAME}_pav_strat.moos $NSFLAGS WARP=$TIME_WARP  \
       PSHARE_PORT=$PSHARE_PORT    VNAME=$VNAME         \
       IP_ADDR=$IP_ADDR            SHORE_IP=$SHORE_IP   \
       SHORE_PSHARE=$SHORE_PSHARE  MOOS_PORT=$MOOS_PORT \
       VTEAM=$VTEAM                RNAME=$RNAME         \
       START_POS=$START_POS        VTYPE="heron"        \
       $SIM $HFLD $FLDS            FSEAT_IP=$FSEAT_IP   \
       FLD_ANG=$FLD_ANG


#-------------------------------------------------------
#  Part 7: Possibly exit now if we're just building† targ files
#-------------------------------------------------------
if [ ${JUST_MAKE} = "yes" ]; then
    echo "Vehicle targ files made. Nothing Launched per request"
    exit 0
fi

#-------------------------------------------------------
#  Part 8: Launch the processes
#-------------------------------------------------------

echo "Launching $RNAME MOOS Community "
pAntler targ_${RNAME}_pav_strat.moos >& /dev/null &

#---------------------------------------------------------------
#  Part 9: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

uMAC targ_${RNAME}_pav_strat.moos

sleep 2 # Give them a chance to exit with grace
echo "Killing all processes ..."
kill -- -$$
echo "Done killing processes."
