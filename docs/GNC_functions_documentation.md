# GNC Functions Documentation

## void set_heading(float heading)
```
Returns 	n/a
```
This function is used to specify the drone’s heading in the local reference frame. Psi is a counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

## void set_destination(float x, float y, float z, float psi)
```
Returns 	n/a
```
This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

## int set_speed(float speed__mps)
```
returns 0 for success
```
This function is used to change the speed of the vehicle in guided mode. it takes the speed in meters per second as a float as the input

## void spinOffSetPub()
```
Returns 	n/a
```
This function publishes the angle between the ENU frame and the local reference frame specified by initialize_local_frame(). This usefull if you have other nodes that need this information.

## int wait4connect()
```
Returns 	0 - connected to fcu
```
Wait for connect is a function that will hold the program until communication with the FCU is established.

## int wait4start()
```
Returns 	0 - mission started
```
Wait for strat will hold the program until the user signals the FCU to enther mode guided. This is typically done from a switch on the safety pilot’s remote or from the ground control station.

## int initialize_local_frame()
```
Returns 	0 - frame initialized
```
This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to.

## int takeoff(float takeoff_alt)
```
Returns 	0 - nominal takeoff
```
The takeoff function will arm the drone and put the drone in a hover above the initial position.

## int check_waypoint_reached(float tolerance)
```
Returns 	1 - waypoint reached
```
This function returns an int of 1 or 0. THis function can be used to check when to request the next waypoint in the mission. The tolerance parameter is used to enforce how close the drone must be to reach a waypoint successfully. A small tollerance may take a long time to reach as the drone makes small slow corrections the closer it is to the waypoint.

## int land()
```
Returns 	1 - mode change successful
```
this function changes the mode of the drone to land

## int init_publisher_subscriber(ros::NodeHandle controlnode)
```
Returns 	n/a
```
This function is called at the beginning of a program and will start of the communication links to the FCU. The function requires the program's ros nodehandle as an input
