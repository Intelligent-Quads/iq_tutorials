# Intalling QGroundControl 

### Video tutorial (18.04) at https://youtu.be/qLQQbhKDQ6M

### Video Tutorial (20.04) at https://youtu.be/1FpJvUVPxL0

## Overview 

QGroundControl provides full flight control and vehicle setup for PX4 or ArduPilot powered vehicles. It provides easy and straightforward usage for beginners, while still delivering high end feature support for experienced users.

### Key Features:

- Full setup/configuration of ArduPilot and PX4 Pro powered vehicles.
- Flight support for vehicles running PX4 and ArduPilot (or any other autopilot that communicates using the MAVLink protocol).
- Mission planning for autonomous flight.
- Flight map display showing vehicle position, flight track, waypoints and vehicle instruments.
- Video streaming with instrument display overlays.
- Support for managing multiple vehicles.
- QGC runs on Windows, OS X, Linux platforms, iOS and Android devices.

for more detailed information please visit http://qgroundcontrol.com/

## Install QGroundControl for Ubuntu Linux 16.04 LTS or later:

Add current user accout to dialout group and remove modemmanager
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager
```
Install QGroundControl dependencies:

```
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
```


Download QGroundControl.AppImage 
```
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
```
Change permissions and run 
```
chmod +x ./QGroundControl.AppImage 
./QGroundControl.AppImage  (or double click)
```

## Run SITL and connect with Q Ground

```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py
```

