# Clone the repository
```
git clone git@github.com:Williamarvin/Marine_automate.git
```

# Build the project
1. There are two folders needed to build
 
#### moos-ivp
```
cd moos-ivp
./build.sh
```

#### moos-ivp-pavlab
```
cd ..
cd moos-ivp-pavlab
./build.sh
```

# Edit ~/.bashrc file for permanent addition to path in linux, check for mac
linux
```
nano ~/.bashrc
```
mac
```
nano ~/.zshrc
```

## Create an alias to navigate easily
### Add these to .bashrc folder
```
alias    cdd='cd ..'
alias   cddd='cd ../..'
alias cddddd='cd ../../../..'
alias  cdddd='cd ../../..'
alias cda='cd /home/e/Marine_automate/moos-ivp/'
alias cdr='cd /home/e/Marine_automate/moos-ivp/ivp/missions'
alias cdb='cd /home/e/Marine_automate/moos-ivp-pavlab-aro/missions/alpha_heron'
```

## Add path permanately to system path
```
export PATH="/home/e/Marine_automate/moos-ivp/ivp/bin:/home/e/Marine_automate/moos-ivp-pavlab-aro/bin:$PATH"
```

## Update path
```
source ~/.bashrc
OR
source ~/.zshrc
```

# How to run mission file
## Navigate to mission folder

```
cd moos-ivp-pavlab-aro/missions/alpha_heron/
./launch_now.sh

To Clean all log messages
./clean.sh
```

# Run file on startup
Read more in startup.service
```
# sudo systemctl daemon-reload
# sudo systemctl enable startup.service
# sudo systemctl start startup.service
# sudo systemctl daemon-reload
```

# If all else fails
1. visit tutorial in: https://oceanai.mit.edu/2.680/pmwiki/pmwiki.php?n=Lab.Lab
2. Set up in mac or other inquires of windows: https://oceanai.mit.edu/2.680/pmwiki/pmwiki.php?n=Lab.ClassSetup

# Docker (experimental)
# Build
connect screen to docker ubuntu screen
```
xhost +local:docker
```
Run the following command if you are building project for the first time, or whenever you change pom.xml, docker-compose.yaml, or Dockerfile
```
docker build -t moos-app:latest .
```
cached
```
docker build --progress=plain -t moos-app:latest .
```
not cached
```
docker build --no-cache --progress=plain -t moos-app:latest .
```
# Debug
Execute into the terminal
```
docker run -d \
  -p 8080:8080 \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --name moos-debug \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  --privileged \
  moos-app debug
docker exec -it moos-debug bash
```
# Run
docker run -it --rm \
  -p 8080:8080 \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --name moos-app \
  moos-app
# Close container
To shut down container, run the following command
```
docker stop moos-app
docker stop moos-debug
```
# Once you’re finished with debugging and no longer need the container, remove it with:
```
docker rm moos-app
docker rm moos-debug
```