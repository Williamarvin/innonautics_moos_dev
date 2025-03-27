# To launch the other vehicles use the following commands in ~/moos-ivp-pavlab/missions/vsim300 (two terminals)

./launch_heron.sh --port=29501 --mport=9001 --vname=abe
./launch_heron.sh --port=29502 --mport=9003 --vname=ben

# AND launch with the following:
./launch.sh

# BE SURE TO USE THE SAME TIME WARP!!!


# To launch the aux community use the following:
./launch_aux.sh --shore=192.168.1.XXX --ip=192.168.1.XXX -<VB>

# where -VB is for "virtual ben" and -B is for the real ben vehicle 

# To launch the real vehicle you want to send info to
# the aux community.  The port number will stay the same, but use
# your ip address
./launch_vehicle.sh -X --shore=192.168.1.XXX --aux=<192.168.1.YYY:9250> 

# To launch a simulated vehicle use the main launch script
./launch.sh --amt=X --aux=<192.168.1.XXX:9250>


# To launch the real vehicle with the disturbance actuator include
# the --dcntrl argument.  So for a real vehicle with the aux community
# the command is 
./launch_vehicle.sh -X --shore=192.168.1.XXX --aux=<192.168.1.YYY:9250> --dctrl

# To launch two real vehicles and have them wait for the other to move
# into position before starting the legrun, then include the --2V argument.
# So for a real vehicle with the aux community, disturbance control
# and the waiting the command is 
./launch_vehicle.sh -X --shore=192.168.1.XXX --aux=<192.168.1.YYY:9250> --dctrl --2V
