#!/bin/bash
#--------------------------------------------------------------
# Author: Kevin Becker
# Date: 04/16/2024
# Script: alog_note.sh
#--------------------------------------------------------------
# Part 1: Convenience functions, set variables
#--------------------------------------------------------------
ME=$(basename "$0")
VERBOSE=0
MOOS_VAR="ALOG_NOTE"
txtrst=$(tput sgr0)       # Reset
txtred=$(tput setaf 1)    # Red
txtgrn=$(tput setaf 2)    # Green
txtylw=$(tput setaf 3)    # Yellow
txtblu=$(tput setaf 4)    # Blue
txtltblu=$(tput setaf 75) # Light Blue
txtgry=$(tput setaf 8)    # Grey
txtul=$(tput smul)        # Underline
txtul=$(tput bold)        # Bold
vecho() {
    if [[ "$VERBOSE" -ge "$1" ]]; then
        echo ${txtgry}"$ME: $2" ${txtrst}
    fi
}
wecho() { echo ${txtylw}"$ME: $1" ${txtrst}; }
vexit() {
    echo ${txtred}"$ME: Error $2. Exit Code $2" ${txtrst}
    exit "$1"
}

#--------------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#--------------------------------------------------------------
for ARGI; do
    if [[ "${ARGI}" = "--help" || "${ARGI}" = "-h" ]]; then
        echo "$ME: [OPTIONS]                                       "
        echo "Sends timestamped notes/messages to shoreside MOOSDB. "
        echo "Options:                                              "
        echo "  --help, -h                                          "
        echo "    Display this help message                         "
        echo "  --verbose=num, -v=num or --verbose, -v              "
        echo "    Set verbosity                                     "
        echo "  --moos_var=              "
        echo "     Set moos variable name (default=ALOG_NOTE)       "
        exit 0
    elif [[ "${ARGI}" = "--moos_var="* ]]; then
        MOOS_VAR="${ARGI#*=}"
    elif [[ "${ARGI}" =~ "--verbose" || "${ARGI}" =~ "-v" ]]; then
        if [[ "${ARGI}" = "--verbose" || "${ARGI}" = "-v" ]]; then
            VERBOSE=1
        else
            VERBOSE="${ARGI#*=}"
        fi
    else
        print_string="$ARGI"
    fi
done

#--------------------------------------------------------------
#  Part 3: Find shoreside MOOSDB
#--------------------------------------------------------------
if [[ -f "targ_shoreside.moos" ]]; then
    vecho 2 "found targ_shoreside.moos"
    SHORE_MOOS="targ_shoreside.moos"
elif [[ -f "targ_shore.moos" ]]; then
    SHORE_MOOS="targ_shore.moos"
    vecho 2 "found targ_shore.moos"
else
    vexit 2 "No shoreside moos files found. Are you in the mission directory?"
fi

#--------------------------------------------------------------
#  Part 4: Wait for msgs to send and send them
#--------------------------------------------------------------
while [[ 1 ]]; do
    read -p "${txtrst}Waiting. Press enter to freeze time"
    TIMESTAMP=$(date +%s)
    read -p "${txtrst}Enter message for UTC=$TIMESTAMP: " print_string
    full_msg="$print_string (msg at UTC=$TIMESTAMP)"
    uPokeDB "$SHORE_MOOS" "$MOOS_VAR=$full_msg" >/dev/null 2>&1
    if [[ $? -ne 0 ]]; then
        vexit 5 "Error running $ uPokeDB \"$SHORE_MOOS\" \"$MOOS_VAR=$full_msg\". Is the mission running?"
    fi
    echo "$txtgry    Sent message $full_msg to $SHORE_MOOS as $MOOS_VAR"

done
