# Installing and Using GZweb

This tutorial will go over how to install a Gazebo web server and how to use gazebo from your web browser. I am currently looking at ways to adapt to the new normal. This is leading me to explore web based tools to create a virtual lab enviorment allowing engineers to work without having to ever meet in person. This imo is a very exciting prospect as it has the potential to liberate robotics engineers from having to work in one location. Hopfully digital nomadicy will be an option for us roboticists in the future. 

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
checkout release branch
```
cd ~/gzweb
git checkout gzweb_1.4.0
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
sudo n 8.0.0
```

## Build GZweb
Run the deploy script, this downloads models from the web and may take a couple of minutes, see more options below.
```
npm run deploy --- -m
```
    Note: the -m flag tells the deploy script to grab all the models from the model database and any other models in your GAZEBO_MODEL_PATH. For all subsequent builds, the -m flag will not be needed