#!/bin/sh
### BEGIN INIT INFO
# Provides: start_pfeffer_now
# Required-Start: $remote_fs $syslog
# Required-Stop: $remote_fs $syslog
# Default-Start: 2 3 4 5
# Default-Stop: 0 1 6
# Short-Description: pfeffer config
# Description: Turns off some functions of the laptop and starts pure data in respective mode and loads the respective project
### END INIT INFO

##### turn off unneeded devices #####
##### TODOs: Disconnect internal WiFi #####
# rfkill block bluetooth &

##### open pure data with the pfeffer project#####
##### TODOs: Realtime Application  #####
pd -alsa -r 48000 -audiooutdev "3" -noadc -blocksize 64 -audiobuf 2 -open ~/puredata/Pfeffer_Project/pfeffer_playground.pd &
