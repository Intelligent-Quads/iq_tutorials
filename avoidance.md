# Drone Obstacle Avoidance

This tutorial will teach how to use a 2d lidar to detect and avoid obstacles using the potential field method.

## Create Obstacle Avoidance Program and add to CMakeLists

create a file called `avoidance.cpp` in `iq_gnc/src`

then add the following to the iq_gnc CMakeLists

```
add_executable(avoidance src/avoidance.cpp)
target_link_libraries(avoidance ${catkin_LIBRARIES})
```

## Setup a Genaric C++ ROS node

```
#include <ros/ros.h>
#include <gnc_functions.hpp>

int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "avoidance_node");
	ros::NodeHandle n;

	//add rest of code here 

	return 0;
}
``` 
here we add the include files we need and define our ros node similar to previous tutorials 

## Add subscriber for lidar

first add the include file for the message of the lidar we will be using
```
#include <sensor_msgs/LaserScan.h>
```

then in our main function define the ros subscriber as so
```
ros::Subscriber collision_sub = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, scan_cb);
```
we will be using a call back function to access the lidar data. Lets add that between the includes and the main function
```
void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{

}
``` 
we will file in the avoidance logic latter

## Takeoff and add control loop
```
//initialize control publisher/subscribers
init_publisher_subscriber(n);

// wait for FCU connection
wait4connect();

//wait for user to switch to mode GUIDED
wait4start();

//create local reference frame 
initialize_local_frame();

//request takeoff
takeoff(2);

set_destination(0,0,2,0);

ros::Rate rate(2.0);
int counter = 0;
while(ros::ok())
{
	
	ros::spinOnce();
	rate.sleep();
	
}


```
this will make our drone take off and hold postion. 

## Parse the Lidar Data

we are going to go through the returns of the lidar and create a direction and magnitude in which the drone will maneuver. We will use a version of the potential field method seen in this [paper](http://users.isr.ist.utl.pt/~mir/pub/ObstacleAvoidance.pdf)

```

sensor_msgs::LaserScan current_2D_scan;
current_2D_scan = *msg;
float avoidance_vector_x = 0; 
float avoidance_vector_y = 0;
bool avoid = false;

for(int i=1; i<current_2D_scan.ranges.size(); i++)
{
	float d0 = 3; 
	float k = 0.5;

	if(current_2D_scan.ranges[i] < d0 && current_2D_scan.ranges[i] > .35)
	{
		avoid = true;
		float x = cos(current_2D_scan.angle_increment*i);
		float y = sin(current_2D_scan.angle_increment*i);
		float U = -.5*k*pow(((1/current_2D_scan.ranges[i]) - (1/d0)), 2);	

		avoidance_vector_x = avoidance_vector_x + x*U;
		avoidance_vector_y = avoidance_vector_y + y*U;

	}
}
```

The following code generates the avoidance waypoint in the correct reference frame and scales it

```
float current_heading = get_current_heading();
float deg2rad = (M_PI/180);
avoidance_vector_x = avoidance_vector_x*cos((current_heading)*deg2rad) - avoidance_vector_y*sin((current_heading)*deg2rad);
avoidance_vector_y = avoidance_vector_x*sin((current_heading)*deg2rad) + avoidance_vector_y*cos((current_heading)*deg2rad);

if(avoid)
{
	if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 3)
	{
		avoidance_vector_x = 3 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
		avoidance_vector_y = 3 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
	}
	geometry_msgs::Point current_pos;
	current_pos = get_current_loaction();
	set_destination(avoidance_vector_x + current_pos.x, avoidance_vector_y + current_pos.y, 2, 0);	
}


```




## Finished Program


```
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <gnc_functions.hpp>




void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
	sensor_msgs::LaserScan current_2D_scan;
  	current_2D_scan = *msg;
	float avoidance_vector_x = 0; 
	float avoidance_vector_y = 0;
	bool avoid = false;
	
	for(int i=1; i<current_2D_scan.ranges.size(); i++)
	{
		float d0 = 3; 
		float k = 0.5;

		if(current_2D_scan.ranges[i] < d0 && current_2D_scan.ranges[i] > .35)
		{
			avoid = true;
			float x = cos(current_2D_scan.angle_increment*i);
			float y = sin(current_2D_scan.angle_increment*i);
			float U = -.5*k*pow(((1/current_2D_scan.ranges[i]) - (1/d0)), 2);	

			avoidance_vector_x = avoidance_vector_x + x*U;
			avoidance_vector_y = avoidance_vector_y + y*U;

		}
	}
	float current_heading = get_current_heading();
	float deg2rad = (M_PI/180);
	avoidance_vector_x = avoidance_vector_x*cos((current_heading)*deg2rad) - avoidance_vector_y*sin((current_heading)*deg2rad);
	avoidance_vector_y = avoidance_vector_x*sin((current_heading)*deg2rad) + avoidance_vector_y*cos((current_heading)*deg2rad);

	if(avoid)
	{
		if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 3)
		{
			avoidance_vector_x = 3 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
			avoidance_vector_y = 3 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
		}
		geometry_msgs::Point current_pos;
		current_pos = get_current_loaction();
		set_destination(avoidance_vector_x + current_pos.x, avoidance_vector_y + current_pos.y, 2, 0);	
	}
	

}

int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle n;
	ros::Subscriber collision_sub = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, scan_cb);
	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(2);


	set_destination(0,0,2,0);
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		
		ros::spinOnce();
		rate.sleep();
		
	
	
	}

	return 0;
}


```

---
### References 

http://users.isr.ist.utl.pt/~mir/pub/ObstacleAvoidance.pdf


