#!/bin/bash
#-----------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-----------------------------------------------------------
TIME_WARP=1
JUST_MAKE="no"
NUM_VEHICLES=11
CASE_F=case1
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	printf "%s [SWITCHES] [time_warp]   \n" $0
	printf "  --just_make, -j    \n"
	printf "  --help, -h         \n"
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_build" -o "${ARGI}" = "-j" ] ; then
	JUST_MAKE="yes"
    else
	printf "Bad Argument: %s \n" $ARGI
	exit 0
    fi
done

#-----------------------------------------------------------
#  Part 2: Create the .moos and .bhv files.
#-----------------------------------------------------------


SHORE_LISTEN="9300"
for ((i=1; i<=$NUM_VEHICLES; i++))
do
    VNAME="AGENT_"$i""
    x=$((15*i))
    x="$((400+x))"
    y=$((15*i))
    y="$((290-y))"
    START_POS="$x,$y"
    LOITER_POS="x=0,y=-75"
    SHARE_L=$((9300+i))
    VP=$((9000+i))
    nsplug meta_vehicle.moos targ_$VNAME.moos -f WARP=$TIME_WARP \
    VNAME=$VNAME      START_POS=$START_POS                       \
    VPORT="$VP"       SHARE_LISTEN="$SHARE_L"                    \
    VTYPE="kayak"     SHORE_LISTEN=$SHORE_LISTEN                 \
    COLOR="blue"

    nsplug meta_vehicle.bhv targ_$VNAME.bhv -f VNAME=$VNAME     \
    START_POS=$START_POS COLOR="blue" LOITER_POS=$LOITER_POS
done

nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
       VNAME="shoreside"  SHARE_LISTEN=$SHORE_LISTEN  VPORT="9000" \
       CASE_FILE=$CASE_F \

if [ ${JUST_MAKE} = "yes" ] ; then
    exit 0
fi

#-----------------------------------------------------------
#  Part 3: Launch the processes
#-----------------------------------------------------------
printf "Launching $SNAME MOOS Community (WARP=%s) \n"  $TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &
for ((i=1; i<=$NUM_VEHICLES; i++))
do
    VNAME="AGENT_$i"
    printf "Launching $VNAME MOOS Community (WARP=%s) \n" $TIME_WARP
    pAntler targ_$VNAME.moos >& /dev/null &
    printf "Done \n"
done

#-----------------------------------------------------------
#  Part 4: Launch uMAC and kill everything upon exiting uMAC
#-----------------------------------------------------------
uMAC targ_shoreside.moos
printf "Killing all processes ... \n"
kill %1 %2 %3 %4 %5 %6 %7 %8 %9 %10 %11 %12
printf "Done killing processes.   \n"
