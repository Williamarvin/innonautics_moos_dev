#!/bin/bash

INVOCATION_ABS_DIR=`pwd`
BUILD_TYPE="None"
CMD_LINE_ARGS=""
BUILD_BOT_CODE_ONLY="OFF"
BUILD_SANDBOX_CODE="OFF"
FORCE_FULL_RASPI_BUILD=""

#-------------------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "build.sh [SWITCHES]                        "
	echo "Switches:                                  " 
	echo "  --help, -h                               " 
        echo "  --debug,   -d                            "
        echo "  --release, -r                            "
	echo "  --minrobot, -m                           "
	echo "    Only build minimal robot apps          "
	echo "  --minrobotx, -mx                         "
	echo "    Override min-robot default on Raspbian "
	echo "  --sandbox, -s                            "
	echo "    Build sandbox code                     "
	echo "                                           "
	echo "Notes:                                     "
	echo "o All other command line args will be passed as args "
	echo "  args to \"make\" when it is eventually invoked.    "
	echo "o For example -k will continue making upon a failure "
	echo "  is encountered in building one of the subdirs.     "
	echo "o For example -j2 will use a 2nd core in the build   "
	echo " if your machine has two cores. -j4 for quad core.   "
	exit 0;
    elif [ "${ARGI}" = "--debug" -o "${ARGI}" = "-d" ]; then
        BUILD_TYPE="Debug"
    elif [ "${ARGI}" = "--release" -o "${ARGI}" = "-r" ]; then
        BUILD_TYPE="Release"
    elif [ "${ARGI}" = "--minrobot" -o "${ARGI}" = "-m" ]; then
        BUILD_BOT_CODE_ONLY="ON"
    elif [ "${ARGI}" = "--minrobotx" -o "${ARGI}" = "-mx" ]; then
        FORCE_FULL_RASPI_BUILD="yes"
    elif [ "${ARGI}" = "--sandbox" -o "${ARGI}" = "-s" ]; then
        BUILD_SANDBOX_CODE="ON"
    else
	CMD_LINE_ARGS=$CMD_LINE_ARGS" "$ARGI
    fi
done


#-------------------------------------------------------------- 
#  Part 2: If this is Raspbian and minrobot not selected, and
#          no explicit override given with -mx, CONFIRM first
#-------------------------------------------------------------- 
command -v raspi-gpio
if [ "$?" = "0" -a "${BUILD_BOT_CODE_ONLY}" = "OFF" ]; then
    if [ ! "${FORCE_FULL_RASPI_BUILD}" = "yes" ]; then
	echo "Pi OS detected without --minrobotx or -mx selected."
	echo "[y] Continue with full build"
	echo "[M] Continue with minrobot build"
	echo -n "Continue? [y/M] "
	read ANSWER
	if [ ! "${ANSWER}" = "y" ]; then
	    BUILD_BOT_CODE_ONLY="ON"
	fi
    fi
fi
	
#-------------------------------------------------------------------
#  Part 3: Check if hydroman 2.0 is available, if so, build it first
#-------------------------------------------------------------------
pushd .. >& /dev/null
if [ -e HydroMAN ]; then
  echo "Found HydroMAN 2.0 external"
  pushd HydroMAN >& /dev/null
  echo "Building HydroMAN"
  ./build.sh $@
  popd >& /dev/null
fi
popd >& /dev/null


#-------------------------------------------------------------------
#  Part 4: Invoke the call to make in the build directory
#-------------------------------------------------------------------
mkdir -p build
cd build

cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE}                   \
      -DBUILD_SANDBOX_CODE=${BUILD_SANDBOX_CODE}         \
      -DBUILD_BOT_CODE_ONLY=${BUILD_BOT_CODE_ONLY}  ../

make ${CMD_LINE_ARGS}
cd ${INVOCATION_ABS_DIR}


