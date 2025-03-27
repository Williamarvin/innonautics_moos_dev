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

SHORE_IP=""
SHORE_PSHARE="9200"
VNAME=""
INDEX="1"

REGION="pavlab"

if [[ "$SYS" == "heron" ]]; then 
    START_POS="0,0,180"
elif [[ "$SYS" == "hydrolink" ]]; then
    START_POS="50,-70,0"
fi 

SPEED="1"
RETURN_POS="5,0"
MAXSPD="2"

ZONE=""