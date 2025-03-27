#!/bin/bash
#---------------------------------------------------------------
#   Script: launch_surveyor_pav_strat3.sh
#  Mission: wp_ctrl_test
#   Author: Tyler Paine + Mike Benjamin
#   LastEd: June 2024

# Quick helper script to launch the additional apps for the pav
# strategy for all three vehicles

TIME_WARP=1

for ARGI; do
    if [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    fi
done

./clean.sh;
./launch_surveyor_pav_strat.sh -vu -b1 --auto $TIME_WARP
./launch_surveyor_pav_strat.sh -vv -b2 --auto $TIME_WARP
./launch_surveyor_pav_strat.sh -vx -b3 --auto $TIME_WARP

#./launch_surveyor_pav_strat.sh -vs -r1 --auto $TIME_WARP
#./launch_surveyor_pav_strat.sh -vt -r2 --auto $TIME_WARP
#./launch_surveyor_pav_strat.sh -vw -r3 --auto $TIME_WARP
