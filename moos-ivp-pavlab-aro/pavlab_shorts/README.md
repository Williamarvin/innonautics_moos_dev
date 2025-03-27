# pavlab_shorts

## By Raymond Turrisi (and maybe more)
```
____             _       _       ____  _                _       
|  _ \ __ ___   _| | __ _| |__   / ___|| |__   ___  _ __| |_ ___ 
| |_) / _` \ \ / / |/ _` | '_ \  \___ \| '_ \ / _ \| '__| __/ __|
|  __/ (_| |\ V /| | (_| | |_) |  ___) | | | | (_) | |  | |_\__ \
|_|   \__,_| \_/ |_|\__,_|_.__/  |____/|_| |_|\___/|_|   \__|___/
```
### About 

In this sub-repository I am including exclusively bash and python scripts, and shell environment modifiers (variables and aliases), which can optimize mundane tasks unique to the pavlab equipment and tools. I would encourage others to add things they have useful utilities to share as well, which don't have a clear home. For example, so far I have included scripts for distributing ssh keys to all the herons, checking if all the herons are online, and aliases for connecting to them via ssh. Will definitely be adding more! 

An example - check all the herons
```
$ herons_online
```

Check specific vehicles (abe ben cal and deb - frontseat and backseat)

```
$ herons_online a b c d
```


First you need to add an evironment variable to your ~/.bashrc or ~/.zshrc
```
export path_ps="$HOME/moos-ivp-pavlab/pavlab_shorts"
```

With this environment variable, you can then add all the groups or a specific group to your shell. 

To activate all the utilities grouped by vehicle or application (herons, sailbot, etc..), place the following in your .bashrc or .zshrc

```
eval "source $path_ps/setup_pavlab_shorts"
```

This calls the individual setup scripts for all the groups. If you want specific groups, you can execute the specific groups setup script instead. 

```
eval "source $path_ps/herons/setup_herons"
```