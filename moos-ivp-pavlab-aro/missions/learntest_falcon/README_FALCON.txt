# To launch the other vehicles use the following commands in ~/moos-ivp-pavlab/missions/vsim300 (two terminals)

./launch_heron.sh --port=29501 --mport=9001 --vname=abe
./launch_heron.sh --port=29502 --mport=9003 --vname=ben

# To launch the aux community use the following
./launch_aux.sh --shore=192.168.1.XXX --ip=192.168.1.XXX
# and when launching the vehicle, and you want to send info to
# the aux community.  The port number will stay the same.
./launch_vehicle.sh -X --shore=192.168.1.XXX --aux=<192.168.1.XXX:9250> 
