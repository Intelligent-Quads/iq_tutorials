# Introduction to YOLO/Darknet Image Recognition 

## Video Tutorial at https://youtu.be/SiVexS6Zrr8

## Install CUDA 
Cuda is a library that allows programs to take advantage of your GPU as a computing resource. YOLO will run without Cuda, but the algorithm is up to 500 x more quick with Cuda. To install Cuda, run 

```
sudo apt install nvidia-cuda-toolkit
```

## **Ubuntu 18.04**
### Clone Darknet/YOLO 

Clone the darknet repo into our catkin_ws

```
cd ~/catkin_ws/src
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
```

### Build Darknet 
```
catkin build -DCMAKE_BUILD_TYPE=Release 
```
on Ubuntu 18.04 you may need to run 
```
catkin build -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-6
```

## **Ubuntu 20.04**
```
cd ~/catkin_ws/src
git clone https://github.com/kunaltyagi/darknet_ros.git
git checkout opencv4
git submodule update --init --recursive
```
### Build Darknet 
```
catkin build -DCMAKE_BUILD_TYPE=Release 
```
if you run into errors try running the following 
```
catkin build -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-8
```


## Configure YOLO/Darknet

in the file `ros.yaml` specifies ros parameters. You can find this file under `darknet_ros/darknet_ros/config`. You will need to change the image topic from `/camera/rgb/image_raw` to 

```
/webcam/image_raw
```

The file `darknet_ros.launch` will launch the darknet/yolo ros node. You can find this file under `darknet_ros/darknet_ros/launch`

in this file you can choose which version of yolo you would like to run by changing 
```
<arg name="network_param_file"         default="$(find darknet_ros)/config/yolov2-tiny.yaml"/>
```
the options are as follows

- yolov1: Not recommended. this model is old 
- yolov2: more accurate, and faster. 
- yolov3: about as fast as v2, but more accurate. Yolo v3 has a high GPU ram requirement to train and run. If your graphics card does not have enough ram, use yolo v2 
- tiny-yolo: Very fast yolo model. Would recommend for application where speed is most important. Works very well on Nvidia Jetson

---
### References 

https://pjreddie.com/darknet/yolo/

