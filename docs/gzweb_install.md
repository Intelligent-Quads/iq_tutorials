# Installing and Using GZweb

This tutorial will go over how to install a Gazebo web server and how to use gazebo from your web browser. I am currently looking at ways to adapt to the new normal. This is leading me to explore web based tools to create a virtual lab environment allowing engineers to work without having to ever meet in person. This imo is a very exciting prospect as it has the potential to liberate robotics engineers from having to work in one location. Hopefully digital nomadicy will be an option for us roboticists in the future. 

This tutorial was made and tested with ***Ubuntu 20.04***. 

## Install Dependencies 

```
sudo apt install gazebo11 libgazebo11-dev
```

```
sudo apt install libjansson-dev nodejs npm libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
```

## Clone GZweb
```
cd ~; git clone https://github.com/osrf/gzweb
```
there is a problem with gzweb building on 20.04 with gazebo11. this is fixed on the below dev branch. I will do my best to update the tutorial when the branch is finally merged.
```
cd ~/gzweb
git checkout fix_build_gz11
```
The first time you build, you'll need to gather all the Gazebo models which you want to simulate in the right directory ('http/client/assets') and prepare them for the web.

Before running the deploy script, it's important to source the Gazebo setup.sh file:
```
source /usr/share/gazebo/setup.sh
```
## Installing n 
gzweb is and older package that is not currently building on the latest version of node. To install an older version of node I use `n` which can be installed as follows 
```
sudo npm install -g n
```

for more information take a look at the n github https://github.com/tj/n

## Installing node 8.0.0
```
sudo n 8.14.0
```

## Set Python to run as Python 2 Temporarily 

Python 3 is now the default python version on Ubuntu 20.04. For this reason I am using `update-alternatives` to switch between python versions.

to switch python version to 2 run 
```
sudo update-alternatives --config python
``` 
select `1` to change to python 2 

## Build GZweb
Run the deploy script, this downloads models from the web and may take a couple of minutes, see more options below.
```
npm run deploy --- -m
```
    Note: the -m flag tells the deploy script to grab all the models from the model database and any other models in your GAZEBO_MODEL_PATH. For all subsequent builds, the -m flag will not be needed

# gzweb
first launch a gazebo world for example 
```
roslaunch iq_sim runway.launch
```

then launch gzweb
```
cd ~/gzweb
npm start
```
# Luanching Gazebo without the gui

In order to launch a gazebo world remotely we don't want to launch a gzclient instance on the server. to avoid the we will need to add the `gui` argument to our gazebo roslaunch files. for example we will need to add the following to the runway.launch file in iq_sim
```
<arg name="gui" default="true"/>
```
and 
```
<arg name="gui" value="$(arg gui)"/>
```
our `runway.launch` would now look like  
```
<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <arg name="gui" default="true"/>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find iq_sim)/worlds/runway.world"/>
    <arg name="gui" value="$(arg gui)"/>
    <!-- more default parameters can be changed here -->
  </include>
</launch>

```

# Optimization 
There are a variety of commands that can be used to optimize gazebo web for your application. be sure to take a look at http://gazebosim.org/tutorials?tut=gzweb_install&cat=gzweb to get the most out of your install 

