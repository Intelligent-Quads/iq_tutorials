# First ROS Subscriber

## Video Tutorial at https://youtu.be/iueRUQCvJXw

In this tutorial I will be showing you how to write a ROS subscriber. A ROS subscriber allows you to use data being published on a ROS topic in your own C++ ROS node. 

## Create a New C++ File

in `iq_gnc/src` create a new file and call it `sub.cpp`

## Add C++ File to CMakeLists

Next open up the `CMakeLists.txt` in `iq_gnc` and add the following lines to the bottom of the file 
```
add_executable(sub src/sub.cpp)
target_link_libraries(sub ${catkin_LIBRARIES})
``` 
these will allow catkin to build our program when we next run `catkin build`

## Add Includes and Main Function

in every ros program we make, we will need to add the ros include as follows
```
#include <ros/ros.h>
``` 

then below, we will add our main function. 
```
int main(int argc, char **argv)
{

	//rest of code will go here 

	return 0;
}

```

## Initialize ROS

```
	ros::init(argc, argv, "detection_sub");
	ros::NodeHandle n;
```

## Declare our Subscriber

subscribers are declared in the form 
```
	ros::Subscriber sub = <nodehandle>.subscribe("<topic>", <# of msg buffered>, <name of callback function>);
```

We want to subscribe to the detection data coming from darknet, so lets see which topics might have the data we need. run
```
roslaunch darknet_ros darknet_ros.launch 
```
then 
```
rostopic list 
```
we see the following topics 
```
/darknet_ros/bounding_boxes
/darknet_ros/check_for_objects/cancel
/darknet_ros/check_for_objects/feedback
/darknet_ros/check_for_objects/goal
/darknet_ros/check_for_objects/result
/darknet_ros/check_for_objects/status
/darknet_ros/detection_image
/darknet_ros/found_object
```
Lets subscribe to `/darknet_ros/bounding_boxes`, but fist we need to see what type of ROS message this is. 
run 
```
rostopic list -v /darknet_ros/bounding_boxes
```
we see 
```
Published topics:
 * /darknet_ros/bounding_boxes [darknet_ros_msgs/BoundingBoxes] 1 publisher

Subscribed topics:
```
this means that the topic is publishing data in the form of `darknet_ros_msgs/BoundingBoxes`

Now we should have enough info to declare our subscriber. write
```
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);
```
then add `ros::spin();` this way the subscriber continues to get the latest data as it becomes available.
```
ros::spin();
```
we will also need to include the ros message to our include section. At the top of the program add
```
#include <darknet_ros_msgs/BoundingBoxes.h>
```

## Write our Callback Function 

we are going to call our call back function `detection_cb`. go ahead and add the following code
```
void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{	
	//rest of callback function code
}
```

now we want to get the name of the object we are seeing out of the message, so lets learn more about the structure of this message. run the following
```
rosmsg show darknet_ros_msgs/BoundingBoxes
```
you will see 
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
std_msgs/Header image_header
  uint32 seq
  time stamp
  string frame_id
darknet_ros_msgs/BoundingBox[] bounding_boxes
  string Class            <---- the information we want
  float64 probability
  int64 xmin
  int64 ymin
  int64 xmax
  int64 ymax
```
we can access the above information by dereferencing the ROS message as follows `msg->bounding_boxes[i].Class`.  add the following code to the callback function to print out all the objects detected.

```
	for( int i=0; i < msg->bounding_boxes.size(); i++)
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());	
	}	
```

## Build and Run Program 

build 
```
catkin build 
```

then run our simulation and takeoff the drone

we can run our new program by running 
```
rosrun iq_gnc sub 
```
we should see the objects seen by the drone being printed in the console

---
### References

http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
