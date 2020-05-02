# Swarming Using Ardupilot

This tutorial shows you how to model and control a swarming using ardupilot and Gazebo

## Pre-rec

add the models folder in the iq_sim repo to the gazebo models path
```
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc
```


## Connecting Multiple Vehicles SITL to Gazebo

You should think about ardupilot as purely a control system. It takes sensor inputs and outputs commands to actuators. Gazebo is playing the role of a Flight Dynamics Model (FDM). The FDM encompasses all of the following, sensor models, actuator models and the dynamics of the vehicle. In real life ardupilot would communicate with your sensors via serial connections, but when you run SITL with gazebo ardupilot talks to the sensors and actuators via UDP connections (an IP protocol). Because of this we will need to specify different UDP ports in our robot models such that these streams do not conflict. 

Lets start by copying the drone1 folder in `iq_sim/models` and paste it as a copy. rename the folder to drone2.

then navigate to the drone2 folder open model.config and change the `<name>` tag to 
```
<name>drone2</name>
```

open the model.sdf and change the model name to `drone2` as well

scroll down to the ardupilot plugin and change
```
      <fdm_port_in>9002</fdm_port_in>
      <fdm_port_out>9003</fdm_port_out>
```
to 
```
      <fdm_port_in>9012</fdm_port_in>
      <fdm_port_out>9013</fdm_port_out>
```

Each successive drone fdm port should be incremented by 10 ie. 

drone3  
```
      <fdm_port_in>9022</fdm_port_in>
      <fdm_port_out>9023</fdm_port_out>
```
drone4
```
      <fdm_port_in>9032</fdm_port_in>
      <fdm_port_out>9033</fdm_port_out>
```
ect...


add the drone to `runway.world`
```
    <model name="drone2">
    <pose>10 0 0 0 0 0</pose>
     <include>
        <uri>model://drone2</uri>
      </include>
    </model>
```
launch the world 
```
roslaunch iq_sim runway.launch 
```

## launch ardupilot terminals 

to tell the ardupilot instance to incrament it's UPD connection use 
```
-I0 for drone1
-I1 for drone2
-I2 for drone3
ect ...
```

launch with 
```
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0
```
```
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I1
```

## Connecting Multiple Drones to qgroundcontrol

In order to connect multiple unique vehicles to a ground station, you will need to make sure that the TCP or UDP connection ports do not conflict. For this example we will be using a TCP connection. The first thing we need to do is relaunch our SITL ardupilot instances with a unique in/out TCP port for our GCS. 

```
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0 --out=tcpin:0.0.0.0:8100 
```
```
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I1 --out=tcpin:0.0.0.0:8200 
```

- note 0.0.0.0 allows any device on out local network to connect to the ardupilot instance 

### setup qgroundcontrol to accept multiple vehicles

navigate to the settings tab and click on `Comm Links`. then find the 

fill in each as bellow for the vehicle's unique TCP ports

![qg](imgs/qg_comms.png)

- note you can connect from a different device on the same network, by entering the ip address of the host computer in the host address box

