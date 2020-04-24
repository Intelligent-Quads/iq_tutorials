# Introduction to Ros for Autonomous Drones 

### Video Tutorial at https://youtu.be/N4XvVldWlXk

Although ROS (Robot Operating System) is not an operating system, it provides services designed for a heterogeneous computer cluster such as hardware abstraction, low-level device control, implementation of commonly used functionality, message-passing between processes, and package management.

We will mostly be using the message passing functionality. To demonstrate this we will launch our simulator again and run a few commands. 

## Make sure Install ROS plugins for Gazebo:
```
sudo apt install ros-melodic-gazebo-ros ros-melodic-gazebo-plugins
```

## Launch Gazebo World 
this time we will launch our gazebo world by launching it with ROS as follows 
```
roslaunch iq_sim runway.launch
```
We will also launch the ArduCopter simulator as well. I have made a nice script so that you don't have to remember the big command `cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console` from the previous tutorials. I recommend you move the script to your home folder for ease of access. Do this by running the command below. 

```
cp ~/catkin_ws/src/iq_sim/scripts/startsitl.sh ~
```
now we can launch the ardupilot sitl by running 
```
~/startsitl.sh
```

## Intoduction to ROS Commandline Tools   

In a new terminal (ctrl+shift+T)

```
rostopic list
```
you should see the following 
```
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/rosout
/rosout_agg
/webcam/camera_info
/webcam/image_raw
/webcam/image_raw/compressed
/webcam/image_raw/compressed/parameter_descriptions
/webcam/image_raw/compressed/parameter_updates
/webcam/image_raw/compressedDepth
/webcam/image_raw/compressedDepth/parameter_descriptions
/webcam/image_raw/compressedDepth/parameter_updates
/webcam/image_raw/theora
/webcam/image_raw/theora/parameter_descriptions
/webcam/image_raw/theora/parameter_updates
/webcam/parameter_descriptions
/webcam/parameter_updates

```
this shows the different topics currently being published. These topics contains data about the source of the data such as images from a camera. 

we can see what data is being published by running the following 
```
rostopic echo /gazebo/model_states
```

now if we fly the drone, we will be able to see changes in position as we fly around.

fly the drone by running the following in the mavproxy terminal as we did in the previous tutorial.

```
mode guided
arm throttle
takeoff 15
```

## Using MAVROS to get telemetry data from the FCU

Now the the topic `/gazebo/model_states` is the true model position in the simulator. This isn't something we can use in real life. In real life, we have to use the estimate of the drone's position which is formulated from a combination of its sensors. This position is trasmitted using a communication protocol called mavlink. These messages are stripped down and are optimized for radio transmission. MAVROS is a middle man which translates the MAVlink messages into ROS messages, which are easy to use and common between different robot systems. To start mavros run

```
roslaunch iq_sim apm.launch
```
when you run `rostopic list` you should see a bunch of mavros topics 
```
/clock
/diagnostics
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/mavlink/from
/mavlink/to
/mavros/adsb/send
/mavros/adsb/vehicle
/mavros/battery
/mavros/cam_imu_sync/cam_imu_stamp
/mavros/companion_process/status
/mavros/distance_sensor/rangefinder_pub
/mavros/distance_sensor/rangefinder_sub
/mavros/extended_state
/mavros/fake_gps/mocap/tf
/mavros/global_position/compass_hdg
/mavros/global_position/global
/mavros/global_position/gp_lp_offset
/mavros/global_position/gp_origin
/mavros/global_position/home
/mavros/global_position/local
/mavros/global_position/raw/fix
/mavros/global_position/raw/gps_vel
/mavros/global_position/rel_alt
/mavros/global_position/set_gp_origin
/mavros/gps_rtk/send_rtcm
/mavros/home_position/home
/mavros/home_position/set
/mavros/imu/data
/mavros/imu/data_raw
/mavros/imu/diff_pressure
/mavros/imu/mag
/mavros/imu/static_pressure
/mavros/imu/temperature_baro
/mavros/imu/temperature_imu
/mavros/landing_target/lt_marker
/mavros/landing_target/pose
/mavros/landing_target/pose_in
/mavros/local_position/accel
/mavros/local_position/odom
/mavros/local_position/pose
/mavros/local_position/pose_cov
/mavros/local_position/velocity_body
/mavros/local_position/velocity_body_cov
/mavros/local_position/velocity_local
/mavros/log_transfer/raw/log_data
/mavros/log_transfer/raw/log_entry
/mavros/manual_control/control
/mavros/manual_control/send
/mavros/mission/reached
/mavros/mission/waypoints
/mavros/mocap/pose
/mavros/obstacle/send
/mavros/odometry/in
/mavros/odometry/out
/mavros/param/param_value
/mavros/radio_status
/mavros/rangefinder/rangefinder
/mavros/rc/in
/mavros/rc/out
/mavros/rc/override
/mavros/setpoint_accel/accel
/mavros/setpoint_attitude/cmd_vel
/mavros/setpoint_attitude/thrust
/mavros/setpoint_position/global
/mavros/setpoint_position/local
/mavros/setpoint_raw/attitude
/mavros/setpoint_raw/global
/mavros/setpoint_raw/local
/mavros/setpoint_raw/target_attitude
/mavros/setpoint_raw/target_global
/mavros/setpoint_raw/target_local
/mavros/setpoint_velocity/cmd_vel
/mavros/setpoint_velocity/cmd_vel_unstamped
/mavros/state
/mavros/statustext/recv
/mavros/statustext/send
/mavros/time_reference
/mavros/timesync_status
/mavros/trajectory/desired
/mavros/trajectory/generated
/mavros/trajectory/path
/mavros/vfr_hud
/mavros/vision_pose/pose
/mavros/vision_pose/pose_cov
/mavros/wind_estimation
/rosout
/rosout_agg
/tf
/tf_static
/webcam/camera_info
/webcam/image_raw
/webcam/image_raw/compressed
/webcam/image_raw/compressed/parameter_descriptions
/webcam/image_raw/compressed/parameter_updates
/webcam/image_raw/compressedDepth
/webcam/image_raw/compressedDepth/parameter_descriptions
/webcam/image_raw/compressedDepth/parameter_updates
/webcam/image_raw/theora
/webcam/image_raw/theora/parameter_descriptions
/webcam/image_raw/theora/parameter_updates
/webcam/parameter_descriptions
/webcam/parameter_updates
```
Now we can see the drones position in it's local frame by running 

```
rostopic echo /mavros/global_position/local
```

In the following tutorials we will be accessing the data on these topics in our C++ programs. To see the type of message being published run 
```
rostopic list -v /mavros/global_position/local
```
we see that the topic is publisng the message in the form of `nav_msgs/Odometry`

to see the structure of the message you can run the following 
```
rosmsg show nav_msgs/Odometry
```
This will be usefull when writing publishers and subscribers in the future 





