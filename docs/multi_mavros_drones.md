# Controlling Multiple Ardupilot Drones Via MAVROS



In this tutorial you will learn how to use the iq_gnc functions to control multiple drones on the same ros network.

## Launching a MAVROS Instance for Each Drone

Each ardupilot SITL instance creates a couple of unique MAVlink streams. This is shown in your mavproxy terminal when you launch your ardupilot instance. For example when launching ardupilot with the below launch options 

```
sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1
```
the output mavlink streams are show as follows
```
SIM_VEHICLE: "mavproxy.py" "--master" "tcp:127.0.0.1:5770" "--sitl" "127.0.0.1:5511" "--out" "127.0.0.1:14560" "--out" "127.0.0.1:14561"
```

We will need to tell MAVROS which udp interface to use for the drone we want MAVROS to connect to. For this vehicle the interface we will be using is 14561. To tell MAVROS to use this port we will launch with the following 
```
roslaunch iq_sim apm.launch fcu_url:=udp://127.0.0.1:14561@14565 mavros_ns:=/drone2 tgt_system:=2
```

You will notice in the command above the argument `mavros_ns:=/drone2`. This is to create a unique mavros topic for each vehicle. For example  
```
/mavros/global_position/local
```
will become 
```
/drone1/mavros/global_position/local
```

you will also notice the launch argument `tgt_system:=2` is passed. This corresponds to the SYSID_THISMAV parameter we set in the `gazebo-droneX.parm` files in the previous tutorial. 


## Executing a Multi-Drone Swarm Mission

**note** if you have not updated your iq_sim and iq_gnc repos lately, go ahead and run a `git pull` in both `iq_sim` and `iq_gnc`

In this example we will use our previously created `square.cpp` program in conjunction with the `multi_drone.world` gazebo sim.

first make a copy of square.cpp and call it multi_square.cpp.

We will be creating a drone swarm where each drone flies a synchronized square pattern. Instead of having to change each drone to GUIDED manually, we will be having the program initiate on start. To do this we will be replacing `wait4start();` with `set_mode("GUIDED");`. This is the only code change that will be needed.

Next we are going to setup a new launch file to launch each drone's gnc code. Create a new file in launch called `multi_square.launch` 
```
<launch>
	<group>
		<node pkg="iq_gnc" type="square" name="square1" output="screen" ns="/drone1">
			<param name="namespace" value="/drone1"/>
			<param name="use_sim_time"  value="true" />
		</node>
	</group>

	<group>
		
		<node pkg="iq_gnc" type="square" name="square2" output="screen" ns="/drone2">
			<param name="namespace" value="/drone2"/>
			<param name="use_sim_time"  value="true" />
		</node>
	</group>
	<group>	
		<node pkg="iq_gnc" type="square" name="square3" output="screen" ns="/drone3">
			<param name="namespace" value="/drone3"/>
			<param name="use_sim_time"  value="true" />
		</node>
	</group>
	<group>	
		<node pkg="iq_gnc" type="square" name="square4" output="screen" ns="/drone4">
			<param name="namespace" value="/drone4"/>
			<param name="use_sim_time"  value="true" />
		</node>
	</group>
</launch>
```

you will notice the below line 
```
<param name="namespace" value="/drone4"/>
```
this parameter tells the gnc_functions what namespace this program's mavros topics fall under. 

The next thing we will need to do is setup a mavros launch file to create the needed links between the drone and the companion computer ros environment. 

create the file `multi-am.launch` within `iq_sim/launch` 

add the below 
```
<launch>
	<node pkg="mavros" type="mavros_node" name="mavros" required="false" clear_params="true" output="screen" respawn="true" ns="/drone1">
		<param name="fcu_url" value="udp://127.0.0.1:14551@14555" />
		<param name="gcs_url" value="" />
		<param name="target_system_id" value="1" />
		<param name="target_component_id" value="1" />
		<param name="fcu_protocol" value="v2.0" />

		<!-- load blacklist, config -->
		<rosparam command="load" file="$(find mavros)/launch/apm_pluginlists.yaml" />
		<rosparam command="load" file="$(find mavros)/launch/apm_config.yaml" />
	</node>

	<node pkg="mavros" type="mavros_node" name="mavros" required="false" clear_params="true" output="screen" respawn="true" ns="/drone2">
		<param name="fcu_url" value="udp://127.0.0.1:14561@14565" />
		<param name="gcs_url" value="" />
		<param name="target_system_id" value="2" />
		<param name="target_component_id" value="1" />
		<param name="fcu_protocol" value="v2.0" />

		<!-- load blacklist, config -->
		<rosparam command="load" file="$(find mavros)/launch/apm_pluginlists.yaml" />
		<rosparam command="load" file="$(find mavros)/launch/apm_config.yaml" />
	</node>
	<node pkg="mavros" type="mavros_node" name="mavros" required="false" clear_params="true" output="screen" respawn="true" ns="/drone3">
		<param name="fcu_url" value="udp://127.0.0.1:14571@14575" />
		<param name="gcs_url" value="" />
		<param name="target_system_id" value="3" />
		<param name="target_component_id" value="1" />
		<param name="fcu_protocol" value="v2.0" />

		<!-- load blacklist, config -->
		<rosparam command="load" file="$(find mavros)/launch/apm_pluginlists.yaml" />
		<rosparam command="load" file="$(find mavros)/launch/apm_config.yaml" />
	</node>
</launch>
```

- you will notice that the `fcu_url` increments by 10 for each corresponding udp port for the drones. 
- the `target_system_id` increments by 1 for each instance. 
- you will notice that each node has a unique ns (namespace). this will put a unique topic prefix for each mavros topic

You will notice that launching each ardupilot instance might be tedious, so it might be helpful to create the below shell script as `multi-ardupilot.sh`

```
#!/bin/bash

gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I2" \
```

## Running the Program

Run the gazebo sim 
```
roslaunch iq_sim multi_drone.launch
```
Run the ardupilot instances
```
./multi-ardupilot.sh
```
Run the mavros instances
```
roslaunch iq_sim multi-apm.launch 
```
Run the guidance program 
```
roslaunch iq_gnc multi_square.launch
```



