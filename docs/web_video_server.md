# Video Web Server

This tutorial is highlighting a great opensource project that is super convinient for viewing cameras on a robot. The packege essentially sets up a webserver on your robot that allows you to select and view image streams from your web browser. Becuase of the web based nature anyone on the same network will have the ability to monitor cameras. This could really aid in the easy of monitoring and dubugging programs in a lab setting. This combined with a VPN could allow team members to view real time video data from anywhere in the world!

## Installation

```
cd ~/catkin/src
git clone https://github.com/sfalexrog/async_web_server_cpp.git
cd async_web_server
git checkout noetic-devel
```

```
cd ~/catkin/src
git clone https://github.com/RobotWebTools/web_video_server.git
```

```
catkin build
```

## Run 
```
rosrun web_video_server web_video_server
```

