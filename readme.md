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
Add startup.service to the service to start
```
# sudo systemctl daemon-reload
# sudo systemctl enable startup.service
# sudo systemctl start startup.service
# sudo systemctl daemon-reload
```

# If all else fails
1. visit tutorial in: https://oceanai.mit.edu/2.680/pmwiki/pmwiki.php?n=Lab.Lab
2. Set up in mac or other inquires of windows: https://oceanai.mit.edu/2.680/pmwiki/pmwiki.php?n=Lab.ClassSetup