# Guidance Navigation and Control

The following tutorial will show you how to make a simple program that allows you to send your drone to waypoints. This tutorial uses my API, which has a bunch of high level functions that handle the various flight operations including, takeoff, land, waypoint nav and all the reference frames associated with the navigation. The documentation for these GNC functions are available **[here](GNC_functions_documentation.md)**

### Video Tutorial Part 1 at https://youtu.be/eRAfeC8OFfs 

## Make sure you have a text editor
As this is the first tutorial that we will be coding please make sure you have a text editor. My prefered text editor is sublime. You can download it by running the below commands
```
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-text
```

## Clone the IQ GNC ROS package

First, we will clone the IQ GNC ROS package. This ROS package comes with my GNC API that will make scripting the drone easy. It will also come with a worked out solution to this tutorial 
```
git clone https://github.com/Intelligent-Quads/iq_gnc.git
```

this package contains the file `gnc_functions.hpp`. This file contains a bunch of useful functions for creating intelligent drone applications.

## Write a small program to navigate your drone

Once you have cloned the `iq_gnc` package. Create a new file called `square.cpp` in `Mission8_OutOfControls/src`. Then open the file `CMakeLists.txt` and add the following to the bottom.
```
add_executable(square src/square.cpp)
target_link_libraries(square ${catkin_LIBRARIES})
```

First we will include our control functions
```
#include <gnc_functions.hpp>
```
This will allow us to use all of our control functions and structures 

Next add the main function and initialize ros
```
int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node;


	//Rest of code here


}
```

We will then add the function `init_publisher_subscriber()`. This function takes our ros node handle as an input and initializes subcribers that will collect the necessary information from our autopilot. Add the following

```
//initialize control publisher/subscribers
init_publisher_subscriber(gnc_node);
```

we will then add the following functions to handle preflight opperations 
```
	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();
```
The function `wait4connect()` will loop until the node can communicate with the flight control unit (FCU). Once the connection with the FCU is established then we will use the function `wait4start()` to hold the program until the pilot executes the program by switching the FCU flight mode to GUIDED. This can be done from a ground control stattion (GCS) such as Mission Planner or QGroundControl or from a switch on a radio controller. Finally, once the command to execute the mission is sent, you will use the function `initialize_local_frame()` to create your navigation frame. This function creates the local reference frame based on the starting location of the drone. 

Next we will request takeoff. Using the function `takeoff(float takeOffHieght)`. Add
```
	//request takeoff
	takeoff(3);
```

The GNC API contains the structure `gnc_api_waypoint` this structure contains the variables `x y z psi` which you can use to set locations and orientations of your drone. 

To make your drone fly in a square specify the following waypoints 
```
	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = -90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 180;
	waypointList.push_back(nextWayPoint);
```

Finally we will add our control loop
```
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		if(check_waypoint_reached() == 1)
		{
			if (counter < waypointList.size())
			{
				set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
				counter++;	
			}else{
				//land after all waypoints are reached
				land();
			}	
		}	
		
	}
	return 0;
}
```
This loop will continually send the requested destination to the FCU via the `set_destination(x, y, z, psi)` function. The loop uses the function `check_waypoint_reached()` to determained when the drone has arrived at the requested destination. The function return 1 or 0. 1 denotes the drone has arrived. Each time the waypoint is reached the vector of waypoints is iterated to request the next waypoint via the variable `counter`. Finally, the drone will land via the land function `land()`


Your program should like like the following 
src/controlAPIExample.cpp 

```
#include <gnc_functions.hpp>
//include API 

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node;
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(3);

	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = -90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 180;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		if(check_waypoint_reached(.3) == 1)
		{
			if (counter < waypointList.size())
			{
				set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
				counter++;	
			}else{
				//land after all waypoints are reached
				land();
			}	
		}	
		
	}
	return 0;
}


```
## Build code
```
cd ~/catkin_ws
catkin build
source ~/.bashrc
```

## Run example code

```
roslaunch iq_sim runway.launch
# New Terminal
./startsitl.sh
# New Terminal
roslaunch iq_sim apm.launch
# New Terminal 
rosrun iq_gnc square
```
NOTE** you can tile gnome terminals by pressing `ctrl + shift + t`

Finally run the mission by changing the flight mode to guided in the MAVproxy terminal by running 
```
mode guided 
```

You should now have a basic understanding of the functions available for controling your drone. You should be able to use these functions to help you make more complex navigation code.




