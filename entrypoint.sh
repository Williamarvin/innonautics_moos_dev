#!/bin/bash

# Check the first argument passed to the container
if [ "$1" == "debug" ]; then
  echo "Entering debug mode..."
  # Keep the container running for debugging
  tail -f /dev/null
else
  echo "Launching the default application..."
  # Run the default application
  ./moos-ivp-pavlab-aro/missions/alpha_heron/launch_all.sh
fi