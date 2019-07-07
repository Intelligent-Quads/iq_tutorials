# Simple Search and Rescue Program

In this tutorial, we will be combining all of our skills together to make a drone that will autonomously look for a missing hiker in the hills. We will use YOLO to identify the person and we will use a modified version of our waypoint program to control where the drone. 

## Create a New File in iq_gnc 

Create a file in `iq_gnc/src` called `sr.cpp`

## Add the cpp File to CMakeLists

add the following lines to the `CMakeLists.txt`, so catkin can build our program

```
add_executable(sr src/sr.cpp)
target_link_libraries(sr ${catkin_LIBRARIES})
```

## Combine sub.cpp and square.cpp

copy in sub.cpp 
```
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	for( int i=0; i < msg->bounding_boxes.size(); i++)
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());	
	}	

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "detection_sub");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);

	ros::spin();

	return 0;
}
```

delete the line
```
ros::spin();
```

then add square.cpp headers
```
#include <gnc_functions.hpp>
``` 

add bellow lines from square.cpp 
```
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
```

be sure to change `init_publisher_subscriber(gnc_node);` to `init_publisher_subscriber(n);`

now we will generate a search waypoint pattern. add 
```
//specify some waypoints 
std::vector<gnc_api_waypoint> waypointList;
gnc_api_waypoint nextWayPoint;
float range = 50;
float spacing = 10;
int rows = 5; 
int row;
for(int i=0; i<5; i++)
{
	row = i*2; 
	nextWayPoint.x = row*spacing;
	nextWayPoint.y = 0;
	nextWayPoint.z = 10;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);	
	
	nextWayPoint.x = row*spacing;
	nextWayPoint.y = range;
	nextWayPoint.z = 10;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);	
	
	nextWayPoint.x = (row+1)*spacing;
	nextWayPoint.y = range;
	nextWayPoint.z = 10;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);

	nextWayPoint.x = (row+1)*spacing;
	nextWayPoint.y = 0;
	nextWayPoint.z = 10;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);	
}
```

then add square.cpp's control loop 
```
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		if(check_waypoint_reached(.3) == 1)
		{
			if(counter < waypointList.size())
			{
				set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
				counter++;
			}else{
				land();
			}
		}
	}
```

## Introduction to FLight Modes

It is common in robotics for the platform to perform different tasks based on the mode it is currently in. For our program we will have to modes. `mode - 0` will be a search mode where the drone searches for the hiker, and `mode - 1` will be resuce mode where the drone delivers the rescue supplies (ex first aid kid, food, water).  

we will add a global variable above the detection callback called `mode_g`. I use `_g` to denote my global variables.
```
int mode_g = 0;
```

## Flight mode Check 

Add a couple if statements to check which flight mode the drone is in, so that the while loop looks like so. 

```
	while(ros::ok())
	{
		if(mode_g == 0)
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
		if(mode_g == 1)
		{
			//rescue operation
		}	
		
	}
```

## Rescue Flight Mode

We want the drone to land and deliver the rescue supplies when we sind the missing person, so lets add that functionallity real quick.

```
	land();
	ROS_INFO("Landing Started");
	break;
```

## Completed Program

```
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <gnc_functions.hpp>

// mode_g denotes the flight opperations
//		0 - search
//		1 - rescue 
int mode_g = 0;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	for( int i=0; i < msg->bounding_boxes.size(); i++)
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());	
		if(msg->bounding_boxes[i].Class == "person")
		{
			mode_g = 1; 
			ROS_INFO("Person found. Starting Rescue Operation");
		}
	}	

}


int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);
	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(10);


	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	float range = 50;
	float spacing = 10;
	int rows = 5; 
	int row;
	for(int i=0; i<5; i++)
	{
		row = i*2; 
		nextWayPoint.x = row*spacing;
		nextWayPoint.y = 0;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
		
		nextWayPoint.x = row*spacing;
		nextWayPoint.y = range;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
		
		nextWayPoint.x = (row+1)*spacing;
		nextWayPoint.y = range;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);

		nextWayPoint.x = (row+1)*spacing;
		nextWayPoint.y = 0;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
	}
	
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		if(mode_g == 0)
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
		if(mode_g == 1)
		{
			land();
			ROS_INFO("Landing Started");
			break;
		}	
		
	}

	return 0;
}
```

