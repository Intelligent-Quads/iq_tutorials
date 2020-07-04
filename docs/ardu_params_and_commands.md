# ArduCopter Parameters and Helpful MAVproxy Commands

## Video Tutorial at https://youtu.be/A-JaRgtljLg

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

Helpful Commands Cheat Sheet [Here](helpful_commands.md) 

# ArduCopter Important Parameters

### Waypoint Nav Parameters 

#### WPNAV_SPEED: Waypoint Horizontal Speed Target

Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
```
Range 	    Increment 	Units
20 - 2000 	50 	        centimeters per second
```
#### WPNAV_RADIUS: Waypoint Radius

Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
```
Range 	    Increment 	Units
5 - 1000 	1           centimeters
```
#### WPNAV_SPEED_UP: Waypoint Climb Speed Target

Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
```
Range 	    Increment 	Units
10 - 1000 	50 	        centimeters per second
```

#### WPNAV_SPEED_DN: Waypoint Descent Speed Target

Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
```
Range 	    Increment 	Units
10 - 500 	10 	        centimeters per second
```
#### WPNAV_ACCEL: Waypoint Acceleration
Defines the horizontal acceleration in cm/s/s used during missions
```
Range 	    Increment 	Units
50 - 500 	10 	        centimeters per square second
```
#### WPNAV_ACCEL_Z: Waypoint Vertical Acceleration

Defines the vertical acceleration in cm/s/s used during missions
```
Range 	    Increment 	Units
50 - 500 	10 	        centimeters per square second
```
#### WPNAV_RFND_USE: Waypoint missions use rangefinder for terrain following
Note: This parameter is for advanced users

This controls if waypoint missions use rangefinder for terrain following
Values
```
Value 	Meaning
0 	    Disable
1 	    Enable
```
#### RTL_ALT: RTL Altitude

The minimum alt above home the vehicle will climb to before returning. If the vehicle is flying higher than this value it will return at its current altitude.
```
Range 	Increment 	Units
200 - 8000 	1 	centimeters
```

#### RTL_CONE_SLOPE: RTL cone slope

Defines a cone above home which determines maximum climb
```
Range 	 	
0.5 - 10.0 	

Values
Value 	Meaning
0 	Disabled
1 	Shallow
3 	Steep

Increment
.1
```
#### RTL_SPEED: RTL speed

Defines the speed in cm/s which the aircraft will attempt to maintain horizontally while flying home. If this is set to zero, WPNAV_SPEED will be used instead.

```
Range 	Increment 	Units
0 - 2000 	50 	centimeters per second
```
#### RTL_ALT_FINAL: RTL Final Altitude

This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission. Set to zero to land.
```
Range 	Increment 	Units
-1 - 1000 	1 	centimeters
```
#### RTL_CLIMB_MIN: RTL minimum climb

The vehicle will climb this many cm during the initial climb portion of the RTL
```
Range 	Increment 	Units
0 - 3000 	10 	centimeters
```
#### RTL_LOIT_TIME: RTL loiter time

Time (in milliseconds) to loiter above home before beginning final descent
```
Ran66666ge 	Increment 	Units
0 - 60000 	1000 	milliseconds
```



---
**References**

[ArduCopter Full Parameter List](http://ardupilot.org/copter/docs/parameters.html)
