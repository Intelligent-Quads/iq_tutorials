# Mavros Basics

**WORK IN PROGRESS**

this tutorial assumes you have completed 

video tutorials 1-4 in the drone programming playlist

## MAVROS Documentation
http://wiki.ros.org/mavros


## What is MAVROS

MAVROS is a ROS wrapper for the communication protocol MAVlink. Mavlink is a light weight communication protocol with a set of message definitions designed to help facilitate communication with Micro Aerial Vehicles (MAV). Defining and sending these definitions can be complex, so by wrapping the communication link in ROS, we can easily integrate the link with our existing ROS nodes. 


## Connecting to FCU 

To launch a connection to your Flight Control Unit (FCU) running Ardupilot, you can use the launch file provided with mavros `apm.launch`, this is located in `mavros/mavros/launch/`. Lets take a closer look at this file 


apm.launch
```
<launch>
	<!-- vim: set ft=xml noet : -->
	<!-- example launch script for ArduPilotMega based FCU's -->

	<arg name="fcu_url" default="/dev/ttyACM0:57600" />
	<arg name="gcs_url" default="" />
	<arg name="tgt_system" default="1" />
	<arg name="tgt_component" default="1" />
	<arg name="log_output" default="screen" />
	<arg name="fcu_protocol" default="v2.0" />
	<arg name="respawn_mavros" default="false" />

	<include file="$(find mavros)/launch/node.launch">
		<arg name="pluginlists_yaml" value="$(find mavros)/launch/apm_pluginlists.yaml" />
		<arg name="config_yaml" value="$(find mavros)/launch/apm_config.yaml" />

		<arg name="fcu_url" value="$(arg fcu_url)" />
		<arg name="gcs_url" value="$(arg gcs_url)" />
		<arg name="tgt_system" value="$(arg tgt_system)" />
		<arg name="tgt_component" value="$(arg tgt_component)" />
		<arg name="log_output" value="$(arg log_output)" />
		<arg name="fcu_protocol" value="$(arg fcu_protocol)" />
		<arg name="respawn_mavros" default="$(arg respawn_mavros)" />
	</include>
</launch>
```

lets a take a look at the following parameters

- fcu_url
This is the url where the mavlink stream is coming into our computer. This can either a serial link like the default (/dev/ttyACM0:57600), or it can be an ip link using UDP. 

- gcs_url 
this is an optional parameter that can be used to facilate a link from your companion computer to your Ground Control Station (GCS)

---

### Explaination of MAVLink Parameters

there are two key mavlink parameters you need to understand in order to under stand the following MAVROS parameters.

system_id - this is a value between 1 and 255 that is used to denote GCSs, MAVs in a MAVlink network. Typically MAV will get low numbers close to 0 and GCSs will get high numbers close to 255 

component_ids - in some mavlink networks there will be redundant streams published from either the same MAV or the same GCS. For example, my drone may have a telemetry radio and a wifi link that both publish the same mavlink messages. In this case the MAVLink message headers for the radio would show a system id of 1 and a component id of 1, where as the IP link would have a system id of 1 (because it comes from the same FCU data), but have a component id of 2 (since the data comes from a different link). 

---

- tgt_system_id
tgt_system tells the MAVROS wrapper what the system id of the source of the MAVLink stream should be. 

- tgt_component_id 
tgt_system tells the MAVROS wrapper what the component id of the source of the MAVLink stream should be.

- log_output 
specify if the mavros output should be logged to a file or shown on a console screen

- fcu_protocol
specifies which version of mavlink you are running. for modern Ardupilot builds this should be set to "v2.0" 

- respawn_mavros 
secifies if the ros node should try to relaunch if it dies, or is unable to make a connection initially

- pluginlists_yaml
this is the path to the yaml file, which can be used to control which mavros plugins are used in your instance of mavros

- config_yaml 
this is a path to the config file, which will control parameters for the various plugins you enable. we will talk more about the parameter in this file in a later tutorial



the apm.launch file is a that is genaric configuration for ardupilot vehicles, that passes parameters to the launch file `node.launch`. if you would like to make a custom launch file it might be easier to start with by editing the node.launch file

```
<launch>
	<!-- vim: set ft=xml noet : -->
	<!-- base node launch file-->

	<arg name="fcu_url" />
	<arg name="gcs_url" />
	<arg name="tgt_system" />
	<arg name="tgt_component" />
	<arg name="pluginlists_yaml" />
	<arg name="config_yaml" />
	<arg name="log_output" default="screen" />
	<arg name="fcu_protocol" default="v2.0" />
	<arg name="respawn_mavros" default="false" />

	<node pkg="mavros" type="mavros_node" name="mavros" required="$(eval not respawn_mavros)" clear_params="true" output="$(arg log_output)" respawn="$(arg respawn_mavros)">
		<param name="fcu_url" value="$(arg fcu_url)" />
		<param name="gcs_url" value="$(arg gcs_url)" />
		<param name="target_system_id" value="$(arg tgt_system)" />
		<param name="target_component_id" value="$(arg tgt_component)" />
		<param name="fcu_protocol" value="$(arg fcu_protocol)" />

		<!-- load blacklist, config -->
		<rosparam command="load" file="$(arg pluginlists_yaml)" />
		<rosparam command="load" file="$(arg config_yaml)" />
	</node>
</launch>
```

## Launch MAVROS SITL 

change the fcu_url parameter to `udp://127.0.0.1:14551@14555` 

run
```
roslaunch mavros apm.launch
```

## View at Telemetry Data
after running our MAVROS node we will notice a bunch of mavros topics when we run

```
rostopic list
```

we can see the data on these topics by running 
```
rostopic echo <topic name>
```

# Takeoff Program Using MAVROS 


```
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>

mavros_msgs:State current_state_g

//get vehicle state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state_g = *msg;
}

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle controlnode("~");

    state_sub = controlnode.subscribe<mavros_msgs::State>((ros_namespace + "/mavros/state").c_str(), 10, state_cb);
    arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

	// arming
	ROS_INFO("Arming drone");
	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = true;
	while (!current_state_g.armed && !arm_request.response.success && ros::ok())
	{
		ros::Duration(.1).sleep();
		arming_client.call(arm_request);
		local_pos_pub.publish(waypoint_g);
	}
	if(arm_request.response.success)
	{
		ROS_INFO("Arming Successful");	
	}else{
		ROS_INFO("Arming failed with %d", arm_request.response.success);
		return -1;	
	}

	//request takeoff
	
	mavros_msgs::CommandTOL srv_takeoff;
	srv_takeoff.request.altitude = takeoff_alt;
	if(takeoff_client.call(srv_takeoff)){
		sleep(3);
		ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
	}else{
		ROS_ERROR("Failed Takeoff");
		return -2;
	}

}
```

create the file `mavros_tutorial.cpp`

we will be 
