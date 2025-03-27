#!/bin/bash 

# This scipt updates the code and variables for the hew herons.
# Currently it pulls code from the forked repo, but hopefully
# this is changed going forward. 

# Change history:
# Tyler Paine July 2022 - Created for the first fleet buildup with the new
#                         front seats and the old IMU.  (before rev r1226)
# Tyler Paine May  2023 - Updated to use the new MicroStrain IMU, which requires
#                         an updated driver (nmea_navsat) and other package/launch
#                         file changes.
# Tyler Paine June 2023 - Another update to fix the error related to assumed GPS
#                         datum. 


echo "Initializing the new workspace"
mkdir /home/administrator/heron_ws
cd /home/administrator/heron_ws
wstool init src
catkin_init_workspace src
catkin_make

echo "Downloading the new code"
cd /home/administrator/heron_ws/src
git clone -b mit-fleet-workup https://github.com/painetyler/heron_robot.git
git clone -b mit-fleet-workup https://github.com/painetyler/um6.git
git clone -b melodic-devel https://github.com/painetyler/nmea_navsat_driver.git
git clone -b mit-fleet-workup https://github.com/painetyler/heron.git
# remove the source code for heron_description and heron_msg package
# so the binaries are used.  
rm -rf /home/administrator/heron_ws/src/heron/heron_description/
rm -rf /home/administrator/heron_ws/src/heron/heron_msgs/

echo "Installing dependances"
sudo apt-get install emacs                  # for editing
sudo apt-get install python-scipy           # for calibration script
sudo apt-get install python-matplotlib      # for calibration script
sudo apt install ros-melodic-roslint        # for um6
sudo apt-get install ros-melodic-catkin-virtualenv  # for nmea_navsat_driver 

echo "Building new code"
cd /home/administrator/heron_ws
catkin_make
source /home/administrator/heron_ws/devel/setup.bash


echo "Moving the old packages to home directory as backup"
# First check if there exists a directory with source code,
# If so, then rename it to keep all consistent.
if [ -d "/home/administrator/catkin_ws" ] 
then
    mv /home/administrator/catkin_ws /home/administrator/catkin_ws_orig
    echo "Found local source code in ~/catkin_ws and moved it off the path"
else
    echo "Looked for local source code directory ~/catkin_ws and didn't find it"
fi

sudo mv /opt/ros/melodic/share/um6 ~/um6.backup
sudo mv /opt/ros/melodic/share/heron_base ~/heron_base.backup
sudo mv /opt/ros/melodic/share/heron_bringup ~/heron_bringup.backup
sudo mv /opt/ros/melodic/share/heron_nmea ~/heron_nmea.backup
sudo mv /opt/ros/melodic/share/heron_control ~/heron_control.backup
sudo mv /opt/ros/melodic/share/nmea_navsat_driver ~/nmea_navsat_driver.backup

echo "Updating launch file"
sudo mv /etc/ros/melodic/ros.d/base.launch /etc/ros/melodic/ros.d/base.launch.backup
sudo cp /home/administrator/heron_ws/src/heron_robot/heron_base/launch/base.launch /etc/ros/melodic/ros.d/base.launch

echo "Adding environment variables"
echo "source /home/administrator/heron_ws/devel/setup.bash" | sudo tee -a /etc/ros/setup.bash > /dev/null 
echo "export HERON_MAGNETOMETER_ENABLED=true" | sudo tee -a /etc/ros/setup.bash > /dev/null 
echo "export MIT_CONFIG=true" | sudo tee -a /etc/ros/setup.bash  > /dev/null 
echo "export HERON_IMU_XYZ='-0.1397 0.0 0' " | sudo tee -a /etc/ros/setup.bash  > /dev/null
echo "export GPS_DATUM='42.358436, -71.087448' " | sudo tee -a /etc/ros/setup.bash  > /dev/null

# RPY Config for MicroStrain
echo "export HERON_IMU_RPY=='0.0 0.0 -1.5707963' " | sudo tee -a /etc/ros/setup.bash  > /dev/null 

echo "Done - Be sure to restart ROS for changes to take effect!"
