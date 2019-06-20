# Common Linux, ROS and MAVproxy Commands

## Common Linux Commands  
```
cat    : [filename] Display file’s contents to the standard output device (usually your monitor). 
cd     : /directorypath Change to directory. 
chmod  : [options] mode filename Change a file’s permissions. 
chown  : [options] filename Change who owns a file. 
clear  : Clear a command line screen/window for a fresh start. 
cp     : [options] source destination Copy files and directories. 
date   : [options] Display or set the system date and time.
df     : [options] Display used and available disk space. 
du     : [options] Show how much space each file takes up. 
file   : [options] filename Determine what type of data is within a file. 
find   : [pathname] [expression] Search for files matching a provided pattern. 
grep   : [options] pattern [filesname] Search files or output for a particular pattern. 
kill   : [options] pid Stop a process. If the process refuses to stop, use kill :9 pid. 
less   : [options] [filename] View the contents of a file one page at a time. 
ln     : [options] source [destination] Create a shortcut. locate filename Search a copy of your filesystem for the specified filename. 
lpr    : [options] Send a print job. 
ls     : [options] List directory contents. 
man    : [command] Display the help information for the specified command. 
mkdir  : [options] directory Create a new directory. 
mv     : [options] source destination Rename or move file(s) or directories. 
passwd : [name [password]] Change the password or allow (for the system administrator) to change any password. 
ps     : [options] Display a snapshot of the currently running processes. 
pwd    : Display the pathname for the current directory. 
rm     : [options] directory Remove (delete) file(s) and/or directories. 
rmdir  : [options] directory Delete empty directories. 
ssh    : [options] user@machine Remotely log in to another Linux machine, over the network.
Leave  : an ssh session by typing exit. 
su     : [options] [user [arguments]] Switch to another user account.
tail   : [options] [filename] Display the last n lines of a file (the default is 10). 
tar    : [options] filename Store and extract files from a tarfile (.tar) or tarball (.tar.gz or .tgz). 
top    : Displays the resources being used on your system. Press q to exit. 
touch  : filename Create an empty file with the specified name. 
who    : [options] Display who is logged on.
```

## Common ROS Commands
```
rostopic list                       : show active ROS topics 
rostopic list -v <topic (optional)> : show active rostopics and the type of ros message 
rostopic echo  <topic>              : show data currently being published 
rosmsg show <message type>          : show the structure of a rosmsg
```

## Common MAVproxy Commands

```
alias           : command aliases: usage: alias <add|remove|list>
arm             : arm motors: usage: arm <check|uncheck|list|throttle|safetyon|safetyoff>
auxopt          : select option for aux switches on CH7 and CH8 (ArduCopter only): Usage: auxopt set|show|reset|list 
disarm          : disarm motors
land            : auto land
log             : log file handling: usage: log <list|download|erase|resume|status|cancel>
mode            : mode change
module          : module commands: usage: module <list|load|reload|unload>
param           : parameter handling: Usage: param <fetch|save|set|show|load|preload|forceload|diff|download|help>
position        : position: Usage: position x y z (meters)
rc              : RC input control: Usage: rc <channel|all> <pwmvalue>
reboot          : reboot autopilot
repeat          : repeat a command at regular intervals: Usage: repeat <add|remove|clean>
script          : run a script of MAVProxy commands
setspeed        : do_change_speed: Usage: setspeed SPEED_VALUE
setyaw          : condition_yaw: Usage: yaw ANGLE ANGULAR_SPEED MODE:[0 absolute / 1 relative]
takeoff         : takeoff: Usage: takeoff ALTITUDE_IN_METERS
```
Run `help` to see a full list of commands 


