# Basic GPS Denied Navigation

This tutorial will focus on how to use an optical flow sensor as a means of GPS denied navigation

**Note: Optical Flow Sensors are not robust, and should only be used to augment another method of position estimation such as SLAM**

## Simulation Setup 


ardupilot has a built in optical flow simulated sensor. To use this we will launch the ardupilot sitl and set `SIM_FLOW` to 1 by runnning 

```
param set SIM_FLOW 1
```

Disable the simulated GPS by typing
```
param set SIM_GPS_ENABLE 0
```

## Vehicle Setup 

first we must enable the drone's EKF to use the optical flow estimates. to do this type
```
param set EK2_GPS_TYPE 3
``` 

Position estimation from optical flow is a methos that integrates velocity at each time step to form a position estimate. In order to do this we must tell the drone where to start integrating from. This will be done by setting the EKF origin. This can be done 2 different ways 

-1.) Via a ground station 
-2.) Programically



