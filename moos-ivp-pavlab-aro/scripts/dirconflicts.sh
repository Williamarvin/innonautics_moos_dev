#!/bin/bash
#--------------------------------------------------------------
# Author: Kevin Becker
# Date: 09/22/2023
# Script: dirconflicts.sh
#--------------------------------------------------------------
# Part 1: Convenience functions, set variables
#--------------------------------------------------------------
ME=$(basename "$0")
VERBOSE=0
txtrst=$(tput sgr0)       # Reset
txtred=$(tput setaf 1)    # Red
txtgrn=$(tput setaf 2)    # Green
txtylw=$(tput setaf 3)    # Yellow
txtblu=$(tput setaf 4)    # Blue
txtltblu=$(tput setaf 75) # Light Blue
txtgry=$(tput setaf 8)    # Grey
txtul=$(tput smul)        # Underline
txtbld=$(tput bold)       # Bold

HAS_CONFLICT="no"
vecho() { if [[ "$VERBOSE" -ge "$1" ]]; then echo ${txtgry}"$ME: $2" ${txtrst}; fi; }
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
    echo " Checks for duplicate files in \$IVP_BEHAVIOR_DIRS"
    echo "Options:                                              "
    echo "  --help, -h                                          "
    echo "    Display this help message                         "
    echo "  --verbose=num, -v=num or --verbose, -v              "
    echo "    Set verbosity                                     "
    exit 0
  elif [[ "${ARGI}" =~ "--verbose" || "${ARGI}" =~ "-v" ]]; then
    if [[ "${ARGI}" = "--verbose" || "${ARGI}" = "-v" ]]; then
      VERBOSE=1
    else
      VERBOSE="${ARGI#*=}"
    fi
  else
    vexit "Bad Arg: $ARGI" 1
  fi
done

#--------------------------------------------------------------
#  Part 3:
#--------------------------------------------------------------

# Create a temporary file
tempfile="$(mktemp)"
touch "$tempfile"

# Split the PATH variable into an array of directories
IFS=':' read -ra DIRS <<<"$IVP_BEHAVIOR_DIRS"

# Iterate over each directory in IVP_BEHAVIOR_DIRS
for dir in "${DIRS[@]}"; do
  # Check if the directory exists and is readable
  if [[ -d "$dir" && -r "$dir" ]]; then
    # If so, iterate over each file in the directory
    for file in "$dir"/*; do
      # Check if the file is readable
      if [[ -r "$file" ]]; then
        filename=$(basename "$file")
        # If the filename is already in the tempfile, print both paths
        if grep -q "^$filename" "$tempfile"; then
          HAS_CONFLICT="yes"
          echo "${txtbld}${txtred}Duplicate file '$filename' found:${txtrst}"
          echo -n "   "
          grep "^$filename*" "$tempfile" | cut -d ' ' -f 2-
          echo "   $file"
        else
          # Add the file to the tempfile
          echo "$filename $file" >>"$tempfile"
        fi
      fi
    done
  fi
done

# Positive message to let user know this util worked
if [[ $HAS_CONFLICT != "yes" ]]; then
  echo "${txtbld}${txtgrn}No lib conflicts found in all \$IVP_BEHAVIOR_DIRS!"
fi

# Remove the temporary file
rm "$tempfile"
